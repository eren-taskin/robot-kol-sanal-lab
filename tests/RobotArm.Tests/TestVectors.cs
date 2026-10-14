using System.Text.Json;
using System.Text.Json.Serialization;

namespace RobotArm.Tests;

/// <summary>
/// Typed view of reference/test_vectors.json, produced by reference/arm_reference.py.
/// The file is copied next to the test assembly by the project file.
/// </summary>
public sealed record TestVectors(
    [property: JsonPropertyName("description")] string Description,
    [property: JsonPropertyName("geometry")] GeometryVectors Geometry,
    [property: JsonPropertyName("bodies")] IReadOnlyList<BodyVectors> Bodies,
    [property: JsonPropertyName("planets")] IReadOnlyDictionary<string, double> Planets,
    [property: JsonPropertyName("poses")] IReadOnlyList<PoseVectors> Poses,
    [property: JsonPropertyName("inverse_kinematics")] IReadOnlyList<InverseKinematicsVectors> InverseKinematics,
    [property: JsonPropertyName("free_motion")] FreeMotionVectors FreeMotion,
    [property: JsonPropertyName("horizontal_reach_pose")] double[] HorizontalReachPose,
    [property: JsonPropertyName("gravity_torque_horizontal_reach")] IReadOnlyDictionary<string, double[]> GravityTorqueHorizontalReach,
    [property: JsonPropertyName("stall_torque")] double StallTorque)
{
    /// <summary>File name of the vectors in the test output directory.</summary>
    public const string FileName = "test_vectors.json";

    private static readonly Lazy<TestVectors> s_instance = new(() => Load(Path.Combine(AppContext.BaseDirectory, FileName)));

    /// <summary>The vectors loaded once from the test output directory.</summary>
    public static TestVectors Instance => s_instance.Value;

    /// <summary>Loads and deserialises the vectors from <paramref name="path"/>.</summary>
    public static TestVectors Load(string path)
    {
        using var stream = File.OpenRead(path);
        return JsonSerializer.Deserialize<TestVectors>(stream)
            ?? throw new InvalidDataException($"{path} deserialised to null.");
    }
}

/// <summary>SPEC §3.1 geometry as written by the reference model.</summary>
public sealed record GeometryVectors(
    [property: JsonPropertyName("d1")] double D1,
    [property: JsonPropertyName("L2")] double L2,
    [property: JsonPropertyName("beta")] double Beta,
    [property: JsonPropertyName("L3")] double L3,
    [property: JsonPropertyName("L4")] double L4);

/// <summary>SPEC §3.2 body row; eps in rad.</summary>
public sealed record BodyVectors(
    [property: JsonPropertyName("m")] double Mass,
    [property: JsonPropertyName("ell")] double Ell,
    [property: JsonPropertyName("eps")] double Eps,
    [property: JsonPropertyName("Ixx")] double Ixx,
    [property: JsonPropertyName("Iyy")] double Iyy,
    [property: JsonPropertyName("Izz")] double Izz,
    [property: JsonPropertyName("Ixz")] double Ixz);

/// <summary>One reference pose: FK, inertia matrix and Earth gravity torque.</summary>
public sealed record PoseVectors(
    [property: JsonPropertyName("payload_kg")] double PayloadKg,
    [property: JsonPropertyName("q")] double[] Q,
    [property: JsonPropertyName("tool_position")] double[] ToolPosition,
    [property: JsonPropertyName("tool_pitch")] double ToolPitch,
    [property: JsonPropertyName("M")] double[][] M,
    [property: JsonPropertyName("gravity_earth")] double[] GravityEarth);

/// <summary>One inverse-kinematics case with both branches.</summary>
public sealed record InverseKinematicsVectors(
    [property: JsonPropertyName("target_position")] double[] TargetPosition,
    [property: JsonPropertyName("target_pitch")] double TargetPitch,
    [property: JsonPropertyName("elbow_up")] double[] ElbowUp,
    [property: JsonPropertyName("elbow_down")] double[] ElbowDown);

/// <summary>Free-motion reference run; reference states are keyed by time in seconds.</summary>
public sealed record FreeMotionVectors(
    [property: JsonPropertyName("planet")] string Planet,
    [property: JsonPropertyName("g")] double G,
    [property: JsonPropertyName("x0")] double[] X0,
    [property: JsonPropertyName("energy0")] double Energy0,
    [property: JsonPropertyName("reference_states")] IReadOnlyDictionary<string, double[]> ReferenceStates,
    [property: JsonPropertyName("note")] string Note,
    [property: JsonPropertyName("error_h1ms_vs_reference")] double ErrorH1msVsReference);
