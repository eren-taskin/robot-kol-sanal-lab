using System.Globalization;
using RobotArm.Core.Tasks;

namespace RobotArm.Core.Logging;

/// <summary>CSV export to a caller-owned TextWriter, without file access. SPEC §10.</summary>
public static class CsvLogWriter
{
    /// <summary>Writes a header and exactly one row for each controller sample; scenario JSON is in RunResult. SPEC §10.</summary>
    public static void Write(TextWriter writer, IReadOnlyList<LogRow> log)
    {
        ArgumentNullException.ThrowIfNull(writer); ArgumentNullException.ThrowIfNull(log);
        writer.Write("t,interval");
        foreach (var name in new[] { "q", "qdot", "qd", "qd_dot", "qd_ddot", "u", "tau", "clamped", "saturated" })
            for (var i = 1; i <= 4; i++) { writer.Write(','); writer.Write(name); writer.Write(i.ToString(CultureInfo.InvariantCulture)); }
        writer.WriteLine(",tool_x,tool_y,tool_z,ref_x,ref_y,ref_z,payload_attached,payload_kg,belief_payload_kg,phase,object_x,object_y,object_z,disturbance1,disturbance2,disturbance3,disturbance4");
        foreach (var row in log)
        {
            writer.Write(row.Time.ToString("R", CultureInfo.InvariantCulture)); Number(writer, row.Interval);
            Values(writer, row.Q); Values(writer, row.QDot); Values(writer, row.Reference); Values(writer, row.ReferenceVelocity);
            Values(writer, row.ReferenceAcceleration); Values(writer, row.Command); Values(writer, row.AppliedTorque);
            for (var i = 0; i < 4; i++) Number(writer, (row.ClampedMask >> i) & 1);
            for (var i = 0; i < 4; i++) Number(writer, (row.SaturatedMask >> i) & 1);
            Number(writer, row.Tool.X); Number(writer, row.Tool.Y); Number(writer, row.Tool.Z);
            Number(writer, row.ReferenceTool.X); Number(writer, row.ReferenceTool.Y); Number(writer, row.ReferenceTool.Z);
            Number(writer, row.PayloadAttached ? 1 : 0); Number(writer, row.PayloadKg); Number(writer, row.BeliefPayloadKg);
            writer.Write(','); writer.Write(row.Phase.ToString());
            Number(writer, row.ObjectPosition.X); Number(writer, row.ObjectPosition.Y); Number(writer, row.ObjectPosition.Z);
            Values(writer, row.Disturbance); writer.WriteLine();
        }
    }
    private static void Values(TextWriter writer, JointVector values) { for (var i = 0; i < 4; i++) Number(writer, values[i]); }
    private static void Number(TextWriter writer, double value) { writer.Write(','); writer.Write(value.ToString("R", CultureInfo.InvariantCulture)); }
}
