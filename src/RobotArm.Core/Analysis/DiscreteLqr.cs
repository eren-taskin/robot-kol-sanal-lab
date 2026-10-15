using System.Globalization;
using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Analysis;

/// <summary>
/// The PD gains taken from the diagonal blocks of an LQR gain, and what they do as a decentralised law. SPEC §12.6.
/// </summary>
/// <param name="Kp">Kp,i = K[i, i], N·m/rad for the arm.</param>
/// <param name="Kd">Kd,i = K[i, n + i], N·m·s/rad for the arm.</param>
/// <param name="Poles">Poles of x_{k+1} = (Φ − Γ [diag(Kp) | diag(Kd)]) x_k.</param>
public sealed record LqrProjection(IReadOnlyList<double> Kp, IReadOnlyList<double> Kd, PoleSet Poles)
{
    /// <summary>ρ of the projected loop.</summary>
    public double SpectralRadius => Poles.SpectralRadius;
}

/// <summary>Result of the discrete LQR design. SPEC §12.6.</summary>
/// <param name="StateCount">nx.</param>
/// <param name="InputCount">nu.</param>
/// <param name="Gain">K = (R + ΓᵀPΓ)⁻¹ ΓᵀPΦ, nu × nx row-major: u_k = −K x_k.</param>
/// <param name="Riccati">P, the stabilising solution of the DARE, nx × nx row-major.</param>
/// <param name="Poles">Poles of Φ − ΓK.</param>
/// <param name="Residual">max |ΦᵀPΦ − P − ΦᵀPΓ(R + ΓᵀPΓ)⁻¹ΓᵀPΦ + Q| divided by max |P|.</param>
/// <param name="Iterations">Doubling steps taken.</param>
/// <param name="Projection">The diagonal projection, or null when the states are not [positions; speeds] (nx ≠ 2·nu).</param>
public sealed record LqrResult(int StateCount, int InputCount, IReadOnlyList<double> Gain, IReadOnlyList<double> Riccati,
    PoleSet Poles, double Residual, int Iterations, LqrProjection? Projection);

/// <summary>
/// Discrete-time LQR for the zero-order-hold plant x_{k+1} = Φ x_k + Γ u_k with cost Σ (xᵀQx + uᵀRu), Q and R
/// diagonal. SPEC §12.6; docs/DECISIONS.md D9.
/// </summary>
/// <remarks>
/// <para>
/// The discrete algebraic Riccati equation P = ΦᵀPΦ − ΦᵀPΓ(R + ΓᵀPΓ)⁻¹ΓᵀPΦ + Q is solved by the structure-preserving
/// doubling algorithm: A₀ = Φ, G₀ = ΓR⁻¹Γᵀ, H₀ = Q and, with S = (I + G_k H_k)⁻¹,
/// A_{k+1} = A_k S A_k, G_{k+1} = G_k + A_k S G_k A_kᵀ, H_{k+1} = H_k + A_kᵀ H_k S A_k. H_k → P quadratically (step k
/// covers 2^k steps of the plain Riccati iteration). It needs (Φ, Γ) stabilisable and (Φ, Q^½) detectable; with every
/// diagonal weight of Q positive the latter always holds. Two plain Riccati iterations then remove the rounding.
/// </para>
/// <para>
/// For the arm (<see cref="ForArm"/>) Φ, Γ are those of <see cref="SampledLoopModel"/>: the plant linearised at rest
/// with its gravity gradient G and friction slope, the input u the full joint torque deviation. The LQR gain therefore
/// couples the joints and contains its own gravity-gradient compensation. Its diagonal projection keeps only each
/// joint's own Kp,i and Kd,i, applied as an independent-joint PD to the same plant; the difference between the two
/// pole sets is what the decentralised structure loses.
/// </para>
/// </remarks>
public static class DiscreteLqr
{
    /// <summary>Largest number of doubling steps.</summary>
    public const int MaxIterations = 80;

    /// <summary>
    /// The LQR design for Φ (nx × nx) and Γ (nx × nu), diagonal Q (<paramref name="stateWeights"/>, nx values ≥ 0)
    /// and R (<paramref name="inputWeights"/>, nu values &gt; 0). SPEC §12.6.
    /// </summary>
    /// <exception cref="ArgumentException">A size is wrong, or a weight is negative (R: not positive) or not finite.</exception>
    /// <exception cref="InvalidOperationException">The doubling did not converge (not stabilisable or not detectable).</exception>
    public static LqrResult Solve(ReadOnlySpan<double> phi, ReadOnlySpan<double> gamma, int nx, int nu,
        ReadOnlySpan<double> stateWeights, ReadOnlySpan<double> inputWeights, double samplePeriod)
    {
        DenseMatrix.CheckSquare(phi, nx, nameof(phi));
        if (nu <= 0 || gamma.Length != nx * nu)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Γ must have {nx * nu} entries."), nameof(gamma));
        if (stateWeights.Length != nx)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Q needs {nx} diagonal weights."), nameof(stateWeights));
        if (inputWeights.Length != nu)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"R needs {nu} diagonal weights."), nameof(inputWeights));
        foreach (var q in stateWeights)
            if (!double.IsFinite(q) || q < 0.0) throw new ArgumentException("Q weights must be finite and at least 0.", nameof(stateWeights));
        foreach (var r in inputWeights)
            if (!double.IsFinite(r) || r <= 0.0) throw new ArgumentException("R weights must be finite and greater than 0.", nameof(inputWeights));

        var qm = new double[nx * nx];
        for (var i = 0; i < nx; i++) qm[i * nx + i] = stateWeights[i];
        var rm = new double[nu * nu];
        for (var i = 0; i < nu; i++) rm[i * nu + i] = inputWeights[i];
        var phiM = phi.ToArray();
        var gammaM = gamma.ToArray();
        var gammaT = DenseMatrix.Transpose(gammaM, nx, nu);

        // Structure-preserving doubling.
        var a = (double[])phiM.Clone();
        var rInvGammaT = DenseMatrix.Multiply(Diagonal(inputWeights, invert: true), gammaT, nu, nu, nx);
        var g = DenseMatrix.Multiply(gammaM, rInvGammaT, nx, nu, nx);
        var h = (double[])qm.Clone();
        var identity = DenseMatrix.Identity(nx);
        var iterations = 0;
        var converged = false;
        while (iterations < MaxIterations)
        {
            iterations++;
            var w = DenseMatrix.Add(identity, DenseMatrix.Multiply(g, h, nx, nx, nx));
            var sa = DenseMatrix.Solve(w, a, nx, nx);
            var sg = DenseMatrix.Solve(w, g, nx, nx);
            var aT = DenseMatrix.Transpose(a, nx, nx);
            var nextA = DenseMatrix.Multiply(a, sa, nx, nx, nx);
            var nextG = DenseMatrix.Add(g, DenseMatrix.Multiply(DenseMatrix.Multiply(a, sg, nx, nx, nx), aT, nx, nx, nx));
            var nextH = DenseMatrix.Add(h, DenseMatrix.Multiply(DenseMatrix.Multiply(aT, h, nx, nx, nx), sa, nx, nx, nx));
            Symmetrise(nextG, nx);
            Symmetrise(nextH, nx);
            var change = DenseMatrix.MaxAbs(DenseMatrix.Add(nextH, h, -1.0));
            (a, g, h) = (nextA, nextG, nextH);
            if (!h.All(double.IsFinite))
                break;
            if (change <= 1e-13 * DenseMatrix.MaxAbs(h))
            {
                converged = true;
                break;
            }
        }
        if (!converged)
            throw new InvalidOperationException("The Riccati doubling did not converge; is (Φ, Γ) stabilisable?");

        var p = h;
        for (var polish = 0; polish < 2; polish++)
        {
            p = DenseMatrix.Add(qm, RiccatiStep(phiM, gammaM, gammaT, rm, p, nx, nu, out _));
            Symmetrise(p, nx);
        }
        var residualMatrix = DenseMatrix.Add(DenseMatrix.Add(qm, RiccatiStep(phiM, gammaM, gammaT, rm, p, nx, nu, out var k)), p, -1.0);
        var residual = DenseMatrix.MaxAbs(residualMatrix) / Math.Max(DenseMatrix.MaxAbs(p), double.Epsilon);

        var closed = DenseMatrix.Add(phiM, DenseMatrix.Multiply(gammaM, k, nx, nu, nx), -1.0);
        var poles = PoleSet.FromMatrix(closed, nx, samplePeriod);

        LqrProjection? projection = null;
        if (nx == 2 * nu)
        {
            var kp = new double[nu];
            var kd = new double[nu];
            var kDiag = new double[nu * nx];
            for (var i = 0; i < nu; i++)
            {
                kp[i] = k[i * nx + i];
                kd[i] = k[i * nx + nu + i];
                kDiag[i * nx + i] = kp[i];
                kDiag[i * nx + nu + i] = kd[i];
            }
            var closedDiag = DenseMatrix.Add(phiM, DenseMatrix.Multiply(gammaM, kDiag, nx, nu, nx), -1.0);
            projection = new LqrProjection(Array.AsReadOnly(kp), Array.AsReadOnly(kd), PoleSet.FromMatrix(closedDiag, nx, samplePeriod));
        }
        return new LqrResult(nx, nu, Array.AsReadOnly(k), Array.AsReadOnly(p), poles, residual, iterations, projection);
    }

    /// <summary>
    /// LQR for the arm linearised at rest at <paramref name="q0"/> and sampled every Ts, with Q = diag(position
    /// weights, velocity weights) and R = diag(input weights). SPEC §12.6.
    /// </summary>
    public static LqrResult ForArm(ArmModel arm, Planet planet, double payloadKg, ReadOnlySpan<double> q0, double samplePeriod,
        ReadOnlySpan<double> positionWeights, ReadOnlySpan<double> velocityWeights, ReadOnlySpan<double> inputWeights)
    {
        var plant = LinearSystem.ArmAtRest(arm, planet, payloadKg, q0);
        var (phi, gamma) = SampledLoopStability.Discretise(plant, samplePeriod);
        var n = plant.InputCount;
        if (positionWeights.Length != n || velocityWeights.Length != n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected {n} position and {n} velocity weights."));
        var q = new double[2 * n];
        positionWeights.CopyTo(q);
        velocityWeights.CopyTo(q.AsSpan(n));
        return Solve(phi, gamma, plant.StateCount, n, q, inputWeights, samplePeriod);
    }

    /// <summary>ΦᵀPΦ − ΦᵀPΓ(R + ΓᵀPΓ)⁻¹ΓᵀPΦ, and the gain K = (R + ΓᵀPΓ)⁻¹ΓᵀPΦ.</summary>
    private static double[] RiccatiStep(double[] phi, double[] gamma, double[] gammaT, double[] r, double[] p, int nx, int nu, out double[] k)
    {
        var phiT = DenseMatrix.Transpose(phi, nx, nx);
        var pPhi = DenseMatrix.Multiply(p, phi, nx, nx, nx);
        var gtpPhi = DenseMatrix.Multiply(gammaT, pPhi, nu, nx, nx);
        var gtpG = DenseMatrix.Add(r, DenseMatrix.Multiply(gammaT, DenseMatrix.Multiply(p, gamma, nx, nx, nu), nu, nx, nu));
        k = DenseMatrix.Solve(gtpG, gtpPhi, nu, nx);
        var step = DenseMatrix.Add(DenseMatrix.Multiply(phiT, pPhi, nx, nx, nx),
            DenseMatrix.Multiply(DenseMatrix.Transpose(gtpPhi, nu, nx), k, nx, nu, nx), -1.0);
        Symmetrise(step, nx);
        return step;
    }

    private static double[] Diagonal(ReadOnlySpan<double> values, bool invert)
    {
        var n = values.Length;
        var m = new double[n * n];
        for (var i = 0; i < n; i++) m[i * n + i] = invert ? 1.0 / values[i] : values[i];
        return m;
    }

    private static void Symmetrise(double[] m, int n)
    {
        for (var i = 0; i < n; i++)
            for (var j = i + 1; j < n; j++)
            {
                var v = 0.5 * (m[i * n + j] + m[j * n + i]);
                m[i * n + j] = m[j * n + i] = v;
            }
    }
}
