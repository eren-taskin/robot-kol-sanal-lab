using System.Globalization;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Core.Control;

/// <summary>
/// A continuous-time linear system ẋ = A x + B u with constant matrices, stored row-major. SPEC §7;
/// docs/DECISIONS.md D6. Used for the sampled closed-loop analysis of <see cref="SampledLoopStability"/>.
/// </summary>
public sealed class LinearSystem
{
    private readonly double[] _a;
    private readonly double[] _b;

    /// <summary>Creates the system; the matrices are copied.</summary>
    /// <param name="stateCount">Number of states nx.</param>
    /// <param name="inputCount">Number of inputs nu.</param>
    /// <param name="a">A, nx × nx, row-major.</param>
    /// <param name="b">B, nx × nu, row-major.</param>
    /// <exception cref="ArgumentOutOfRangeException">A size is not greater than 0, or an entry is not finite.</exception>
    /// <exception cref="ArgumentException">A matrix has the wrong number of entries.</exception>
    public LinearSystem(int stateCount, int inputCount, ReadOnlySpan<double> a, ReadOnlySpan<double> b)
    {
        if (stateCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(stateCount), stateCount, "State count must be greater than 0.");
        if (inputCount <= 0)
            throw new ArgumentOutOfRangeException(nameof(inputCount), inputCount, "Input count must be greater than 0.");
        if (a.Length != stateCount * stateCount)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"A must have {stateCount * stateCount} entries, got {a.Length}."), nameof(a));
        if (b.Length != stateCount * inputCount)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture,
                $"B must have {stateCount * inputCount} entries, got {b.Length}."), nameof(b));
        foreach (var v in a)
            if (!double.IsFinite(v)) throw new ArgumentOutOfRangeException(nameof(a), v, "A must be finite.");
        foreach (var v in b)
            if (!double.IsFinite(v)) throw new ArgumentOutOfRangeException(nameof(b), v, "B must be finite.");

        StateCount = stateCount;
        InputCount = inputCount;
        _a = a.ToArray();
        _b = b.ToArray();
    }

    /// <summary>Number of states nx.</summary>
    public int StateCount { get; }

    /// <summary>Number of inputs nu.</summary>
    public int InputCount { get; }

    /// <summary>A, nx × nx, row-major.</summary>
    public ReadOnlySpan<double> A => _a;

    /// <summary>B, nx × nu, row-major.</summary>
    public ReadOnlySpan<double> B => _b;

    /// <summary>
    /// The arm linearised at rest at <paramref name="q0"/> (q̇ = 0) on <paramref name="planet"/> with
    /// <paramref name="payloadKg"/>: x = [δq; δq̇], u = δτ, SPEC §6.1 linearised. docs/DECISIONS.md D6.
    /// </summary>
    /// <remarks>
    /// <code>
    /// A = [[0, I], [−M⁻¹G, −M⁻¹D]],   B = [[0], [M⁻¹]]
    /// </code>
    /// M = M(q0) with N²·Jm (SPEC §5.2); G = ∂g/∂q at q0 by central differences of the bias torque with q̇ = 0
    /// (step 1e-6 rad); D = diag(fv,i + fc,i/ωs,i), the slope of the friction of SPEC §5.4 at q̇ = 0. Saturation is
    /// left out (it is not linear). Allocates.
    /// </remarks>
    /// <exception cref="ArgumentNullException">The model or planet is null.</exception>
    /// <exception cref="ArgumentException"><paramref name="q0"/> has the wrong length or a value that is not finite.</exception>
    /// <exception cref="ArgumentOutOfRangeException">The payload is negative or not finite.</exception>
    public static LinearSystem ArmAtRest(ArmModel model, Planet planet, double payloadKg, ReadOnlySpan<double> q0)
    {
        ArgumentNullException.ThrowIfNull(model);
        ArgumentNullException.ThrowIfNull(planet);
        var loaded = model.WithPayload(payloadKg);
        var n = loaded.DegreesOfFreedom;
        CheckPose(n, q0);

        var rnea = new RecursiveNewtonEuler(loaded);
        var m = new double[n * n];
        rnea.InertiaMatrix(q0, m);
        var g = GravityGradient(rnea, planet.GravityVector, q0);

        // M⁻¹ by Cholesky solves on the unit columns; M⁻¹G and M⁻¹D by the same solves.
        var factor = (double[])m.Clone();
        Cholesky.Factor(factor, n);
        var column = new double[n];
        var minvG = new double[n * n];
        var minvD = new double[n * n];
        var minv = new double[n * n];
        for (var j = 0; j < n; j++)
        {
            for (var i = 0; i < n; i++) column[i] = g[i * n + j];
            Cholesky.Solve(factor, n, column);
            for (var i = 0; i < n; i++) minvG[i * n + j] = column[i];

            Array.Clear(column);
            column[j] = 1.0;
            Cholesky.Solve(factor, n, column);
            var actuator = loaded.Actuators[j];
            var d = actuator.ViscousFriction + actuator.CoulombFriction / actuator.SmoothingSpeed;
            for (var i = 0; i < n; i++)
            {
                minv[i * n + j] = column[i];
                minvD[i * n + j] = column[i] * d;
            }
        }

        var nx = 2 * n;
        var a = new double[nx * nx];
        var b = new double[nx * n];
        for (var i = 0; i < n; i++)
        {
            a[i * nx + n + i] = 1.0;
            for (var j = 0; j < n; j++)
            {
                a[(n + i) * nx + j] = -minvG[i * n + j];
                a[(n + i) * nx + n + j] = -minvD[i * n + j];
                b[(n + i) * n + j] = minv[i * n + j];
            }
        }
        return new LinearSystem(nx, n, a, b);
    }

    /// <summary>
    /// G = ∂g/∂q at <paramref name="q0"/> by central differences (step 1e-6 rad) of the bias torque with q̇ = 0,
    /// n × n row-major: G[i, j] = ∂gi/∂qj. SPEC §5.3; docs/DECISIONS.md D6. Allocates.
    /// </summary>
    internal static double[] GravityGradient(RecursiveNewtonEuler rnea, Vector3d gravity, ReadOnlySpan<double> q0)
    {
        const double delta = 1e-6;
        var n = rnea.DegreesOfFreedom;
        CheckPose(n, q0);
        var q = q0.ToArray();
        var zero = new double[n];
        var plus = new double[n];
        var minus = new double[n];
        var gradient = new double[n * n];
        for (var j = 0; j < n; j++)
        {
            q[j] = q0[j] + delta;
            rnea.BiasTorque(q, zero, gravity, plus);
            q[j] = q0[j] - delta;
            rnea.BiasTorque(q, zero, gravity, minus);
            q[j] = q0[j];
            for (var i = 0; i < n; i++)
                gradient[i * n + j] = (plus[i] - minus[i]) / (2.0 * delta);
        }
        return gradient;
    }

    private static void CheckPose(int n, ReadOnlySpan<double> q0)
    {
        if (q0.Length != n)
            throw new ArgumentException(string.Create(CultureInfo.InvariantCulture, $"Expected {n} joint angles, got {q0.Length}."), nameof(q0));
        foreach (var v in q0)
            if (!double.IsFinite(v))
                throw new ArgumentException("The pose must be finite.", nameof(q0));
    }
}
