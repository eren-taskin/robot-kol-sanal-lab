using System.Globalization;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Simulation;

/// <summary>
/// Largest RK4 step that is stable near rest. SPEC §6.2 ("Step-size stability"); docs/DECISIONS.md D3.
/// </summary>
/// <remarks>
/// <para>
/// Near q̇ = 0 the friction of SPEC §5.4 is linear damping with di = fv,i + fc,i/ωs,i on joint i (each joint's
/// own actuator, <see cref="ArmModel.Actuators"/>), so the speeds
/// obey q̈ ≈ −M(q)⁻¹ D q̇ with D = diag(d + extra damping). The fastest mode decays at λmax, the largest
/// eigenvalue of M⁻¹D, and RK4 is stable on the negative real axis only for h·λmax &lt; 2.78.
/// </para>
/// <para>
/// M⁻¹D is similar to the symmetric matrix D^½ M⁻¹ D^½, so its eigenvalues are real and not negative, and
/// λmax = 1 / λmin(D^−½ M D^−½). The smallest eigenvalue is found by the Jacobi method.
/// The methods allocate; they are meant for the user interface, not for once per step.
/// </para>
/// <para>
/// Controller gains do not enter this limit: the solver holds the input u constant over each step (and the
/// controller holds it over Ts ≥ h), so inside the RK4 stages a controller is a constant input. Gains decide a
/// different stability, that of the sampled closed loop (SPEC §6.3, <see cref="Control.SampledLoopStability"/>;
/// docs/DECISIONS.md D6).
/// </para>
/// <para>
/// Limitations (docs/DECISIONS.md D3): the estimate holds at one pose q only and ignores the gravity gradient
/// ∂g/∂q, so it models only the damped velocity modes.
/// </para>
/// </remarks>
public static class StabilityEstimate
{
    /// <summary>
    /// RK4 stability limit on the negative real axis, h·λ for |R(hλ)| = 1, rounded down from 2.785.
    /// SPEC §6.2.
    /// </summary>
    public const double Rk4StabilityLimit = 2.78;

    /// <summary>
    /// 2.78 / λmax(M(q)⁻¹ D) in s, with D = diag(fv,i + fc,i/ωs,i + extra damping i). SPEC §6.2. Returns
    /// <see cref="double.PositiveInfinity"/> when there is no damping at all.
    /// </summary>
    /// <param name="model">The arm, including any payload; M includes N²·Jm (SPEC §5.2).</param>
    /// <param name="q">Joint angles, rad.</param>
    /// <param name="extraDamping">
    /// Additional continuous damping per joint in N·m·s/rad, for example physical damping that the actuator
    /// parameters leave out; empty for none. Not a controller's derivative gains, which act through the held input
    /// (docs/DECISIONS.md D6).
    /// </param>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> is null.</exception>
    /// <exception cref="ArgumentException">A span has the wrong length, or an angle in <paramref name="q"/> is not finite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An extra damping value is negative or not finite.</exception>
    public static double MaxStableStep(ArmModel model, ReadOnlySpan<double> q, ReadOnlySpan<double> extraDamping)
    {
        var lambda = MaxDampingRate(model, q, extraDamping);
        return lambda > 0.0 ? Rk4StabilityLimit / lambda : double.PositiveInfinity;
    }

    /// <summary>
    /// <see cref="MaxStableStep(ArmModel, ReadOnlySpan{double}, ReadOnlySpan{double})"/> without extra damping:
    /// friction only. SPEC §6.2.
    /// </summary>
    public static double MaxStableStep(ArmModel model, ReadOnlySpan<double> q) =>
        MaxStableStep(model, q, ReadOnlySpan<double>.Empty);

    /// <summary>
    /// λmax, the largest eigenvalue of M(q)⁻¹ D in 1/s, with D = diag(fv,i + fc,i/ωs,i + extra damping i). SPEC §6.2.
    /// </summary>
    /// <exception cref="ArgumentNullException"><paramref name="model"/> is null.</exception>
    /// <exception cref="ArgumentException">A span has the wrong length, or an angle in <paramref name="q"/> is not finite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">An extra damping value is negative or not finite.</exception>
    public static double MaxDampingRate(ArmModel model, ReadOnlySpan<double> q, ReadOnlySpan<double> extraDamping)
    {
        ArgumentNullException.ThrowIfNull(model);
        var n = model.DegreesOfFreedom;
        if (q.Length != n)
            throw new ArgumentException($"Expected {n} joint angles, got {q.Length} (SPEC §6.2).", nameof(q));
        if (extraDamping.Length != 0 && extraDamping.Length != n)
            throw new ArgumentException($"Expected 0 or {n} extra damping values, got {extraDamping.Length} (SPEC §6.2).", nameof(extraDamping));
        for (var i = 0; i < n; i++)
            if (!double.IsFinite(q[i]))
                throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                    $"Joint angle q[{i}] = {q[i]} is not finite; the step-size estimate needs a finite pose (SPEC §6.2)."), nameof(q));

        var scale = new double[n]; // D^−½, or 0 for an undamped joint
        var anyDamping = false;
        for (var i = 0; i < n; i++)
        {
            var extra = extraDamping.Length == 0 ? 0.0 : extraDamping[i];
            if (!double.IsFinite(extra) || extra < 0.0)
                throw new ArgumentOutOfRangeException(nameof(extraDamping), extra,
                    "Extra damping must be a finite number of at least 0 N·m·s/rad (SPEC §6.2).");
            var actuator = model.Actuators[i];
            var d = actuator.ViscousFriction + actuator.CoulombFriction / actuator.SmoothingSpeed + extra;
            scale[i] = d > 0.0 ? 1.0 / Math.Sqrt(d) : 0.0;
            anyDamping |= d > 0.0;
        }
        if (!anyDamping)
            return 0.0;

        var m = new double[n * n];
        new RecursiveNewtonEuler(model).InertiaMatrix(q, m);

        // Undamped joints contribute zero eigenvalues of M⁻¹D. Restrict to the damped joints: the nonzero
        // eigenvalues of M⁻¹D are those of D_d^½ (M⁻¹)_dd D_d^½, so use the Schur complement of M instead.
        // With every joint damped (the usual case, fv or fc > 0) this is simply D^−½ M D^−½.
        var a = ScaledSchurComplement(m, n, scale);
        var lambdaMin = SmallestEigenvalue(a, a.GetLength(0));
        return 1.0 / lambdaMin;
    }

    /// <summary>
    /// D_d^−½ S D_d^−½, where S = M_dd − M_du M_uu⁻¹ M_ud is the Schur complement of M on the damped joints
    /// (d) against the undamped ones (u); (M⁻¹)_dd = S⁻¹.
    /// </summary>
    private static double[,] ScaledSchurComplement(double[] m, int n, double[] scale)
    {
        var damped = new List<int>();
        var undamped = new List<int>();
        for (var i = 0; i < n; i++)
            (scale[i] > 0.0 ? damped : undamped).Add(i);

        var k = damped.Count;
        var s = new double[k, k];
        for (var r = 0; r < k; r++)
            for (var c = 0; c < k; c++)
                s[r, c] = m[damped[r] * n + damped[c]];

        var u = undamped.Count;
        if (u > 0)
        {
            var muu = new double[u * u];
            for (var r = 0; r < u; r++)
                for (var c = 0; c < u; c++)
                    muu[r * u + c] = m[undamped[r] * n + undamped[c]];
            Cholesky.Factor(muu, u);
            var column = new double[u];
            for (var c = 0; c < k; c++)
            {
                for (var r = 0; r < u; r++) column[r] = m[undamped[r] * n + damped[c]];
                Cholesky.Solve(muu, u, column); // M_uu⁻¹ M_ud, column c
                for (var r = 0; r < k; r++)
                {
                    var sum = 0.0;
                    for (var j = 0; j < u; j++) sum += m[damped[r] * n + undamped[j]] * column[j];
                    s[r, c] -= sum;
                }
            }
        }

        for (var r = 0; r < k; r++)
            for (var c = 0; c < k; c++)
                s[r, c] *= scale[damped[r]] * scale[damped[c]];
        return s;
    }

    /// <summary>Smallest eigenvalue of a symmetric matrix by cyclic Jacobi rotations. Overwrites <paramref name="a"/>.</summary>
    private static double SmallestEigenvalue(double[,] a, int n)
    {
        for (var sweep = 0; sweep < 100; sweep++)
        {
            var off = 0.0;
            var diag = 0.0;
            for (var p = 0; p < n; p++)
            {
                diag += a[p, p] * a[p, p];
                for (var q = p + 1; q < n; q++) off += a[p, q] * a[p, q];
            }
            if (off <= 1e-30 * diag)
                break;

            for (var p = 0; p < n - 1; p++)
            {
                for (var q = p + 1; q < n; q++)
                {
                    if (a[p, q] == 0.0) continue;
                    var theta = (a[q, q] - a[p, p]) / (2.0 * a[p, q]);
                    var t = Math.Sign(theta == 0.0 ? 1.0 : theta) / (Math.Abs(theta) + Math.Sqrt(theta * theta + 1.0));
                    var c = 1.0 / Math.Sqrt(t * t + 1.0);
                    var s = t * c;
                    for (var k = 0; k < n; k++)
                    {
                        var akp = a[k, p];
                        var akq = a[k, q];
                        a[k, p] = c * akp - s * akq;
                        a[k, q] = s * akp + c * akq;
                    }
                    for (var k = 0; k < n; k++)
                    {
                        var apk = a[p, k];
                        var aqk = a[q, k];
                        a[p, k] = c * apk - s * aqk;
                        a[q, k] = s * apk + c * aqk;
                    }
                }
            }
        }

        var min = double.PositiveInfinity;
        for (var i = 0; i < n; i++) min = Math.Min(min, a[i, i]);
        return min;
    }
}
