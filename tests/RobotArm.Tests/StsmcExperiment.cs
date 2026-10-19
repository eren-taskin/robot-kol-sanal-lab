using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Logging;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using RobotArm.Core.Tasks;

namespace RobotArm.Tests;

/// <summary>One row of the S1 comparison (docs/results/stsmc_comparison.csv, docs/DECISIONS.md D13).</summary>
/// <param name="Case">"underestimate" (plant 0.3 kg, belief 0 kg) or "overestimate" (plant 0 kg, belief 0.3 kg).</param>
/// <param name="Controller">CT, CT+I, STSMC (ε = 0) or STSMC (ε &gt; 0).</param>
/// <param name="Rms">RMS joint error per joint while moving (samples with t &lt; 1 s), rad.</param>
/// <param name="Peak">Peak joint error per joint over the run, up to 1.5 s or the fault, rad.</param>
/// <param name="Final">|q_target − q| per joint at the end of the run (after the hold, or at the fault), rad.</param>
/// <param name="Chatter">Mean |u_k − u_(k−1)| per joint over the logged samples, N·m.</param>
/// <param name="ClampedPercent">Percentage of samples in which the controller clamped that joint.</param>
/// <param name="AnyClampedPercent">Percentage of samples with at least one joint clamped.</param>
/// <param name="Fault">The fault that stopped the run, or null.</param>
internal sealed record StsmcComparisonRow(string Case, string Controller, double[] Rms, double[] Peak, double[] Final,
    double[] Chatter, double[] ClampedPercent, double AnyClampedPercent, Fault? Fault);

/// <summary>
/// The S1 experiments on the path of V29 (1 s quintic [0, −0.4, −0.3, 1.1] → [0, 0.1, −0.8, 0.6] rad, then a
/// 0.5 s hold, Earth): the disturbance bound L and the controller comparison (docs/DECISIONS.md D13).
/// </summary>
internal static class StsmcExperiment
{
    /// <summary>End of the comparison runs: 1 s move + 0.5 s hold, as V29.</summary>
    public const double Duration = 1.5;

    /// <summary>The boundary layer ε of the "STSMC (ε > 0)" runs, rad/s.</summary>
    public const double SmoothBoundaryLayer = 0.5;

    /// <summary>The library scenario "Yanlış model – STSMC, 0,3 kg habersiz yük".</summary>
    public static Scenario Library => ScenarioLibrary.All.Single(s => s.Name == "Yanlış model – STSMC, 0,3 kg habersiz yük");

    /// <summary>The V29 path with a payload mismatch and the given controller, run to <see cref="Duration"/>.</summary>
    public static Scenario Case(bool underestimate, string controller)
    {
        var s = Library with
        {
            Duration = Duration,
            PayloadKg = underestimate ? 0.3 : 0.0,
            Belief = new BeliefSettings { PayloadKg = underestimate ? 0.0 : 0.3, Knowledge = PayloadKnowledge.Unaware }
        };
        return s with
        {
            Controller = controller switch
            {
                "CT" => new ControllerSettings { Method = ControlMethod.CT },
                "CT+I" => new ControllerSettings { Method = ControlMethod.CT, IntegralEnabled = true },
                "STSMC (ε = 0)" => new ControllerSettings { Method = ControlMethod.STSMC },
                "STSMC (ε > 0)" => new ControllerSettings { Method = ControlMethod.STSMC, BoundaryLayer = SmoothBoundaryLayer },
                _ => throw new ArgumentOutOfRangeException(nameof(controller))
            }
        };
    }

    public static readonly string[] Controllers = ["CT", "CT+I", "STSMC (ε = 0)", "STSMC (ε > 0)"];

    /// <summary>Runs one case and summarises it as a comparison row.</summary>
    public static StsmcComparisonRow Compare(bool underestimate, string controller) =>
        Compare(Case(underestimate, controller), underestimate ? "underestimate" : "overestimate", controller);

    /// <summary>Runs <paramref name="scenario"/> and summarises it as a comparison row.</summary>
    public static StsmcComparisonRow Compare(Scenario scenario, string caseName, string controller)
    {
        var result = new ExperimentRunner().Run(scenario);
        var log = result.Log;
        var target = TaskTarget(scenario);
        double[] rms = new double[4], peak = new double[4], final = new double[4], chatter = new double[4], clamped = new double[4];
        var moving = 0; var any = 0;
        for (var k = 0; k < log.Count; k++)
        {
            var row = log[k];
            if (row.ClampedMask != 0) any++;
            for (var i = 0; i < 4; i++)
            {
                var e = Math.Abs(row.Reference[i] - row.Q[i]);
                peak[i] = Math.Max(peak[i], e);
                if (row.Time < scenario.Task.MotionDuration - 1e-9) rms[i] += e * e;
                if (k > 0) chatter[i] += Math.Abs(row.Command[i] - log[k - 1].Command[i]);
                if ((row.ClampedMask & (1 << i)) != 0) clamped[i]++;
            }
            if (row.Time < scenario.Task.MotionDuration - 1e-9) moving++;
        }
        for (var i = 0; i < 4; i++)
        {
            // A fault before the end: the peak also covers the true state at the fault.
            peak[i] = Math.Max(peak[i], result.Fault is null ? 0 : Math.Abs(log[^1].Reference[i] - result.Final.Q[i]));
            rms[i] = Math.Sqrt(rms[i] / Math.Max(1, moving));
            final[i] = Math.Abs(target[i] - result.Final.Q[i]);
            chatter[i] /= Math.Max(1, log.Count - 1);
            clamped[i] = 100.0 * clamped[i] / log.Count;
        }
        return new(caseName, controller, rms, peak, final, chatter, clamped,
            100.0 * any / log.Count, result.Fault);
    }

    /// <summary>The joint target of the Reach task (IK of the target point and pitch).</summary>
    public static double[] TaskTarget(Scenario s)
    {
        var q = RobotArm.Core.Kinematics.InverseKinematics.Solve(s.Task.Target, s.Task.Pitch, s.Task.Branch).Q;
        return [q[0], q[1], q[2], q[3]];
    }

    /// <summary>
    /// The perturbation of the sliding variable caused by the model error, at every controller sample of a run:
    /// δ = M̂⁻¹(τ − ĥ − τ̂f) − q̈, the commanded acceleration the believed model would give for the applied torque τ
    /// minus the true acceleration q̈ = M⁻¹(τ − h − τf). With an exact model δ = 0. Evaluated either on the
    /// reference (<paramref name="onReference"/>: q = q_d, q̇ = q̇_d, and τ = M q̈_d + h + τf, the torque the true
    /// plant needs to follow the path: then δ = M̂⁻¹[(M − M̂) q̈_d + (h − ĥ) + (τf − τ̂f)], the equivalent control
    /// the integral w must supply while s = 0), or on the logged true state and applied torque of the run.
    /// Returns δ per sample (rows) and joint (columns).
    /// </summary>
    public static double[][] Perturbation(IReadOnlyList<LogRow> log, double truePayload, double believedPayload, bool onReference)
    {
        var arm = ArmModel.OpenManipulatorX();
        var truth = new RecursiveNewtonEuler(arm.WithPayload(truePayload));
        var belief = new RecursiveNewtonEuler(arm.WithPayload(believedPayload));
        var gravity = Planet.Earth.GravityVector;
        var result = new double[log.Count][];
        Span<double> q = stackalloc double[4], qd = stackalloc double[4], qdd = stackalloc double[4], tau = stackalloc double[4];
        Span<double> h = stackalloc double[4], hHat = stackalloc double[4], m = stackalloc double[16], mHat = stackalloc double[16];
        Span<double> commanded = stackalloc double[4];
        for (var k = 0; k < log.Count; k++)
        {
            var row = log[k];
            (onReference ? row.Reference : row.Q).CopyTo(q);
            (onReference ? row.ReferenceVelocity : row.QDot).CopyTo(qd);
            truth.BiasTorque(q, qd, gravity, h);
            belief.BiasTorque(q, qd, gravity, hHat);
            truth.InertiaMatrix(q, m);
            belief.InertiaMatrix(q, mHat);
            for (var i = 0; i < 4; i++)
            {
                var a = arm.Actuators[i];
                var friction = a.ViscousFriction * qd[i] + a.CoulombFriction * Math.Tanh(qd[i] / a.SmoothingSpeed);
                h[i] += friction; hHat[i] += friction; // same actuators in plant and belief
            }
            if (onReference)
            {
                row.ReferenceAcceleration.CopyTo(qdd);
                for (var i = 0; i < 4; i++) { tau[i] = h[i]; for (var j = 0; j < 4; j++) tau[i] += m[i * 4 + j] * qdd[j]; }
            }
            else
            {
                row.AppliedTorque.CopyTo(tau);
                for (var i = 0; i < 4; i++) qdd[i] = tau[i] - h[i];
                Cholesky.FactorAndSolve(m, 4, qdd); // true q̈
            }
            for (var i = 0; i < 4; i++) commanded[i] = tau[i] - hHat[i];
            Cholesky.FactorAndSolve(mHat, 4, commanded); // M̂⁻¹(τ − ĥ − τ̂f)
            result[k] = new double[4];
            for (var i = 0; i < 4; i++) result[k][i] = commanded[i] - qdd[i];
        }
        return result;
    }

    /// <summary>max over samples of |δ_k − δ_(k−1)| / (t_k − t_(k−1)) per joint: the measured L, rad/s³.</summary>
    public static double[] Rate(IReadOnlyList<LogRow> log, double[][] delta)
    {
        var l = new double[4];
        for (var k = 1; k < delta.Length; k++)
            for (var i = 0; i < 4; i++)
                l[i] = Math.Max(l[i], Math.Abs(delta[k][i] - delta[k - 1][i]) / (log[k].Time - log[k - 1].Time));
        return l;
    }
}
