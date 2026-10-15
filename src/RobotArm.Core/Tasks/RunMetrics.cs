using RobotArm.Core.Logging;
using RobotArm.Core.Model;

namespace RobotArm.Core.Tasks;

/// <summary>Metrics computed only from logged controller samples. SPEC §9.</summary>
/// <param name="RmsJointError">Time-weighted RMS error per joint, rad.</param>
/// <param name="MaxJointError">Peak absolute error per joint, rad.</param>
/// <param name="RmsToolError">Time-weighted RMS tool-reference distance, m.</param>
/// <param name="MaxToolError">Maximum tool-reference distance, m.</param>
/// <param name="FinalToolError">Last logged tool-reference distance, m.</param>
/// <param name="SettlingTime">Absolute time after which the tool stays within 2 mm of the final target, from the final hold onward; null if never settled.</param>
/// <param name="Overshoot">Maximum distance beyond the final target projected on the start-to-target direction, m; zero for zero-length moves.</param>
/// <param name="PeakTorque">Peak absolute applied motor torque per joint, N·m.</param>
/// <param name="ClampedPercent">Time percentage clamped by the controller, per joint.</param>
/// <param name="SaturatedPercent">Time percentage physically saturated, per joint.</param>
/// <param name="AnyClampedPercent">Time percentage with at least one controller clamp.</param>
/// <param name="MechanicalEnergy">Left-rectangle integral of sum |tau*qdot|, J.</param>
/// <param name="TrackingRmsToolError">RMS during the moving part of Track, excluding its final hold, m.</param>
/// <param name="PayloadAttachments">Logged detached-to-attached transitions.</param>
/// <param name="PayloadReleases">Logged attached-to-detached transitions.</param>
/// <param name="MaxPayloadKg">Maximum logged attached payload, kg.</param>
public sealed record RunMetrics(JointVector RmsJointError, JointVector MaxJointError, double RmsToolError,
    double MaxToolError, double FinalToolError, double? SettlingTime, double Overshoot, JointVector PeakTorque,
    JointVector ClampedPercent, JointVector SaturatedPercent, double AnyClampedPercent, double MechanicalEnergy,
    double TrackingRmsToolError, int PayloadAttachments, int PayloadReleases, double MaxPayloadKg)
{
    /// <summary>Computes metrics with each row's actual held interval as weight. SPEC §9.</summary>
    public static RunMetrics Compute(IReadOnlyList<LogRow> log, Vector3d finalTarget, double settlingFromTime, double motionDuration)
    {
        Span<double> square = stackalloc double[4]; Span<double> max = stackalloc double[4];
        Span<double> peak = stackalloc double[4]; Span<double> clamp = stackalloc double[4]; Span<double> saturated = stackalloc double[4];
        square.Clear(); max.Clear(); peak.Clear(); clamp.Clear(); saturated.Clear();
        double time = 0, toolSquare = 0, maxTool = 0, finalTool = 0, anyClamp = 0, energy = 0, trackingSquare = 0, trackingTime = 0, overshoot = 0, payload = 0;
        double? settling = null;
        var attach = 0; var release = 0; var previousAttached = false;
        var direction = log.Count == 0 ? Vector3d.Zero : finalTarget - log[0].Tool;
        if (direction.Length > 1e-12) direction /= direction.Length; else direction = Vector3d.Zero;
        for (var k = 0; k < log.Count; k++)
        {
            var row = log[k]; var dt = row.Interval; time += dt;
            var eTool = (row.Tool - row.ReferenceTool).Length;
            toolSquare += eTool * eTool * dt; maxTool = Math.Max(maxTool, eTool); finalTool = eTool;
            var trackDt = Math.Max(0, Math.Min(dt, motionDuration - row.Time));
            trackingSquare += eTool * eTool * trackDt; trackingTime += trackDt;
            if (row.Time + 1e-10 >= settlingFromTime)
            {
                if ((row.Tool - finalTarget).Length > 0.002) settling = null;
                else settling ??= row.Time;
            }
            overshoot = Math.Max(overshoot, Vector3d.Dot(row.Tool - finalTarget, direction));
            if (row.ClampedMask != 0) anyClamp += dt;
            for (var i = 0; i < 4; i++)
            {
                var e = Math.Abs(row.Reference[i] - row.Q[i]);
                square[i] += e * e * dt; max[i] = Math.Max(max[i], e);
                peak[i] = Math.Max(peak[i], Math.Abs(row.AppliedTorque[i]));
                if ((row.ClampedMask & (1 << i)) != 0) clamp[i] += dt;
                if ((row.SaturatedMask & (1 << i)) != 0) saturated[i] += dt;
                energy += Math.Abs(row.AppliedTorque[i] * row.QDot[i]) * dt;
            }
            if (k > 0 && row.PayloadAttached != previousAttached) { if (row.PayloadAttached) attach++; else release++; }
            previousAttached = row.PayloadAttached; payload = Math.Max(payload, row.PayloadKg);
        }
        for (var i = 0; i < 4; i++)
        {
            square[i] = time > 0 ? Math.Sqrt(square[i] / time) : 0;
            clamp[i] = time > 0 ? 100 * clamp[i] / time : 0;
            saturated[i] = time > 0 ? 100 * saturated[i] / time : 0;
        }
        return new(JointVector.From(square), JointVector.From(max), time > 0 ? Math.Sqrt(toolSquare / time) : 0,
            maxTool, finalTool, settling, overshoot, JointVector.From(peak), JointVector.From(clamp), JointVector.From(saturated),
            time > 0 ? 100 * anyClamp / time : 0, energy, trackingTime > 0 ? Math.Sqrt(trackingSquare / trackingTime) : 0, attach, release, payload);
    }
}
