using RobotArm.Core.Model;
using RobotArm.Core.Tasks;

namespace RobotArm.Core.Logging;

/// <summary>One controller sample at Time; torques/flags apply for Interval seconds after it. SPEC §10.</summary>
/// <param name="Time">Sample time, s.</param>
/// <param name="Interval">Actual simulated hold duration, shortened at the run end or a fault, s.</param>
/// <param name="Q">True joint angles, rad.</param>
/// <param name="QDot">True joint speeds, rad/s.</param>
/// <param name="Reference">Reference joint angles, rad.</param>
/// <param name="ReferenceVelocity">Reference joint speeds, rad/s.</param>
/// <param name="ReferenceAcceleration">Reference joint accelerations, rad/s².</param>
/// <param name="Command">Controller motor command, N·m.</param>
/// <param name="AppliedTorque">Motor torque after physical saturation, N·m.</param>
/// <param name="ClampedMask">Controller clamp flags, bit i for joint i+1.</param>
/// <param name="SaturatedMask">Plant saturation flags, bit i for joint i+1.</param>
/// <param name="Tool">True tool position, m.</param>
/// <param name="ReferenceTool">Tool position of the reference, m.</param>
/// <param name="PayloadAttached">Whether the object is attached, including zero-mass objects.</param>
/// <param name="PayloadKg">True attached mass, kg.</param>
/// <param name="BeliefPayloadKg">Controller's payload belief, kg.</param>
/// <param name="Phase">Task phase at the sample.</param>
/// <param name="ObjectPosition">PickPlace object position, m.</param>
/// <param name="Disturbance">External torque at the sample, excluded from motor torque, N·m.</param>
public readonly record struct LogRow(double Time, double Interval, JointVector Q, JointVector QDot,
    JointVector Reference, JointVector ReferenceVelocity, JointVector ReferenceAcceleration,
    JointVector Command, JointVector AppliedTorque, int ClampedMask, int SaturatedMask,
    Vector3d Tool, Vector3d ReferenceTool, bool PayloadAttached, double PayloadKg, double BeliefPayloadKg,
    TaskPhase Phase, Vector3d ObjectPosition, JointVector Disturbance);
