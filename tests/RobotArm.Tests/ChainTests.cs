using RobotArm.Core.Model;

namespace RobotArm.Tests;

/// <summary>SPEC §4.1: the kinematic chain table as data.</summary>
public sealed class ChainTests
{
    private static readonly ArmModel s_arm = ArmModel.OpenManipulatorX();

    [Fact]
    public void Joints_OriginsAndAxes_EqualSpecTable()
    {
        Assert.Equal(4, s_arm.DegreesOfFreedom);
        Assert.Equal(new Vector3d(0, 0, 0), s_arm.Joints[0].Origin);
        Assert.Equal(new Vector3d(0, 0, 0.0595), s_arm.Joints[1].Origin);
        Assert.Equal(new Vector3d(0.024, 0, 0.128), s_arm.Joints[2].Origin);
        Assert.Equal(new Vector3d(0.124, 0, 0), s_arm.Joints[3].Origin);

        Assert.Equal(JointAxis.Z, s_arm.Joints[0].Axis);
        Assert.Equal(JointAxis.Y, s_arm.Joints[1].Axis);
        Assert.Equal(JointAxis.Y, s_arm.Joints[2].Axis);
        Assert.Equal(JointAxis.Y, s_arm.Joints[3].Axis);
        Assert.Equal(Vector3d.UnitZ, s_arm.Joints[0].AxisVector);
        Assert.Equal(Vector3d.UnitY, s_arm.Joints[3].AxisVector);
    }

    [Fact]
    public void ToolPoint_IsL4AlongLastFrameX()
    {
        Assert.Equal(new Vector3d(0.126, 0, 0), s_arm.ToolPoint);
    }

    [Fact]
    public void Factory_ReturnsEqualModelsEachCall()
    {
        var other = ArmModel.OpenManipulatorX();
        Assert.NotSame(s_arm, other);
        Assert.Equal(s_arm, other);
        Assert.Equal(s_arm.GetHashCode(), other.GetHashCode());
    }

    [Fact]
    public void Constructor_CopiesTheCallersArrays()
    {
        var joints = s_arm.Joints.ToArray();
        var bodies = s_arm.Bodies.ToArray();
        var actuators = s_arm.Actuators.ToArray();
        var model = new ArmModel(joints, bodies, s_arm.ToolPoint, actuators);

        joints[0] = new JointDefinition(Vector3d.Zero, JointAxis.Y, -1, 1);
        bodies[0] = new BodyParameters(1.0, 0.0, 0.0, 1e-3, 1e-3, 1e-3, 0.0);
        actuators[0] = new ActuatorParameters { StallTorque = 1.0 };

        Assert.Equal(s_arm, model);
        Assert.False(model.Joints is JointDefinition[]);
    }

    [Fact]
    public void Constructor_MismatchedJointAndBodyCounts_Throws()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new ArmModel(s_arm.Joints, s_arm.Bodies.Take(3), s_arm.ToolPoint, s_arm.Actuators[0]));
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new ArmModel([], [], s_arm.ToolPoint, s_arm.Actuators[0]));
    }

    [Fact]
    public void Constructor_NeedsOneActuatorPerJoint()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            new ArmModel(s_arm.Joints, s_arm.Bodies, s_arm.ToolPoint, s_arm.Actuators.Take(3)));
        Assert.Throws<ArgumentNullException>(() =>
            new ArmModel(s_arm.Joints, s_arm.Bodies, s_arm.ToolPoint, [.. s_arm.Actuators.Take(3), null!]));
        Assert.Throws<ArgumentNullException>(() =>
            new ArmModel(s_arm.Joints, s_arm.Bodies, s_arm.ToolPoint, (ActuatorParameters)null!));
    }

    [Fact]
    public void Actuators_ArePerJoint_AndPartOfEquality()
    {
        var actuators = s_arm.Actuators.ToArray();
        actuators[2] = actuators[2] with { StallTorque = 2.0 };
        var model = new ArmModel(s_arm.Joints, s_arm.Bodies, s_arm.ToolPoint, actuators);

        Assert.Equal(2.0, model.Actuators[2].StallTorque);
        Assert.Equal(4.1, model.Actuators[1].StallTorque);
        Assert.NotEqual(s_arm, model);
        Assert.False(model.Actuators is ActuatorParameters[]);
    }

    [Fact]
    public void JointAxisX_IsTheRollAxis_RxMatchesTheRightHandRule()
    {
        // Rx(a) = [[1,0,0],[0,c,−s],[0,s,c]]: a positive angle turns local y towards local z.
        var joint = new JointDefinition(Vector3d.Zero, JointAxis.X, -1, 1);
        Assert.Equal(Vector3d.UnitX, joint.AxisVector);
        var r = RobotArm.Core.Kinematics.Rotations.About(JointAxis.X, 0.3);
        Assert.Equal(RobotArm.Core.Kinematics.Rotations.Rx(0.3), r);
        Assert.Equal(new Vector3d(1, 0, 0), r * Vector3d.UnitX);
        var y = r * Vector3d.UnitY;
        Assert.Equal(0.0, y.X);
        Assert.Equal(Math.Cos(0.3), y.Y, 1e-15);
        Assert.Equal(Math.Sin(0.3), y.Z, 1e-15);
    }

    [Fact]
    public void Joint_LowerNotBelowUpper_Throws()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointDefinition(Vector3d.Zero, JointAxis.Y, 1.0, 1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointDefinition(Vector3d.Zero, JointAxis.Y, 1.0, -1.0));
        Assert.Throws<ArgumentOutOfRangeException>(() => new JointDefinition(Vector3d.Zero, (JointAxis)7, -1.0, 1.0));
    }

    [Fact]
    public void Chain_IsDataNotCode_SixJointChainCanBeBuilt()
    {
        var body = new BodyParameters(0.1, 0.02, 0.0, 1e-5, 1e-5, 1e-5, 0.0);
        var joints = Enumerable.Range(0, 6)
            .Select(i => new JointDefinition(new Vector3d(0.05, 0, 0), i % 2 == 0 ? JointAxis.Z : JointAxis.Y, -2, 2));
        var model = new ArmModel(joints, Enumerable.Repeat(body, 6), new Vector3d(0.05, 0, 0), new ActuatorParameters());
        Assert.Equal(6, model.DegreesOfFreedom);
    }
}
