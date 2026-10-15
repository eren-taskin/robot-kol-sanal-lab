using RobotArm.Core.Control;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using Xunit.Abstractions;
using static System.FormattableString;
using static RobotArm.Tests.ControlTestHelpers;

namespace RobotArm.Tests;

/// <summary>
/// SPEC §6.3 (sampled control, sensors) and V16 of SPEC §11: Ts as a multiple of h, the held command,
/// determinism, the encoder model, and no allocation per sample (PROJECT.md §3).
/// </summary>
public sealed class SampledControlTests(ITestOutputHelper output)
{
    private static readonly double s_encoderResolution = 2.0 * Math.PI / 4096.0;

    [Theory]
    [InlineData(5e-4, 1.2e-3)]
    [InlineData(5e-4, 2.25e-3)]
    [InlineData(1e-3, 5e-4)]
    [InlineData(5e-4, 0.0)]
    [InlineData(5e-4, double.NaN)]
    public void SamplePeriod_NotAPositiveIntegerMultipleOfTheStep_Throws(double h, double ts)
    {
        var ex = Assert.Throws<ArgumentOutOfRangeException>(() =>
            Loop(new PdGravity(ControllerBelief.OpenManipulatorX(Planet.Earth), PdGravity.DefaultGains),
                new PerfectSensor(4), new ConstantReference(PoseA), Planet.Earth, 0.0, PoseA, h, ts));
        Assert.Equal("samplePeriod", ex.ParamName);
    }

    [Fact]
    public void Step_OutsideTheRangeOfSpec62_Throws()
    {
        Assert.Throws<ArgumentOutOfRangeException>(() =>
            Loop(new PdGravity(ControllerBelief.OpenManipulatorX(Planet.Earth), PdGravity.DefaultGains),
                new PerfectSensor(4), new ConstantReference(PoseA), Planet.Earth, 0.0, PoseA, 1e-5, 1e-3));
    }

    [Theory]
    [InlineData(5e-4, 2e-3, 4)]
    [InlineData(1e-4, 2e-3, 20)]
    [InlineData(2e-3, 2e-3, 1)]
    [InlineData(1e-3, 0.01, 10)]
    public void SamplePeriod_IntegerMultiple_RunsThatManyPlantStepsPerSample(double h, double ts, int expected)
    {
        var loop = Loop(new PdGravity(ControllerBelief.OpenManipulatorX(Planet.Earth), PdGravity.DefaultGains),
            new PerfectSensor(4), new ConstantReference(PoseB), Planet.Earth, 0.0, PoseA, h, ts);
        Assert.Equal(expected, loop.StepsPerSample);
        for (var k = 0; k < 7; k++) Assert.True(loop.AdvanceSample());
        Assert.Equal(7L * expected, loop.Simulation.StepCount);
        Assert.Equal(7 * ts, loop.Time, 1e-15);
        Assert.Equal(6 * ts, loop.LastSampleTime, 1e-15);
    }

    /// <summary>The controller is called once per Ts, at t = k·Ts, and its command is held over the k plant steps.</summary>
    [Fact]
    public void Command_IsComputedOncePerSample_AndHeldBetweenSamples()
    {
        var recorder = new RecordingController(4);
        var loop = Loop(recorder, new PerfectSensor(4), new ConstantReference(PoseA), Planet.Earth, 0.0, PoseA, 5e-4, 2e-3);
        for (var k = 0; k < 10; k++) loop.AdvanceSample();

        Assert.Equal(10, recorder.Times.Count);
        for (var k = 0; k < 10; k++) Assert.Equal(k * 2e-3, recorder.Times[k], 1e-15);
        // The plant applied the command of the last sample in every one of its four steps.
        Assert.Equal(recorder.LastCommand, loop.Simulation.AppliedTorque.ToArray());
    }

    /// <summary>V16: the same scenario and seed give identical logs; a different seed gives a different log.</summary>
    [Fact]
    public void V16_SameScenarioAndSeed_GiveIdenticalLogs()
    {
        var first = Log(seed: 7);
        var second = Log(seed: 7);
        var other = Log(seed: 8);

        Assert.Equal(first.Length, second.Length);
        for (var i = 0; i < first.Length; i++)
            Assert.Equal(BitConverter.DoubleToInt64Bits(first[i]), BitConverter.DoubleToInt64Bits(second[i]));
        Assert.NotEqual(first, other);
        output.WriteLine(Invariant($"V16: {first.Length} logged values identical bit for bit"));
    }

    /// <summary>A log row per sample: t, q, q̇, measured q and q̇, u (SPEC §10 in short), with noise, filter and PID.</summary>
    private static double[] Log(int seed)
    {
        var options = new EncoderSensorOptions { NoiseStdDev = 1e-3, FilterTimeConstant = 5e-3, Seed = seed };
        var ctl = new Pid(ControllerBelief.OpenManipulatorX(Planet.Mars, 0.2), Pid.DefaultGains);
        var loop = Loop(ctl, new EncoderSensor(ArmModel.OpenManipulatorX(), options),
            new QuinticReference(PoseA, PoseB, 0.5), Planet.Mars, 0.2, PoseA);
        var log = new List<double>();
        while (loop.Time < 1.0 - 1e-9)
        {
            Assert.True(loop.AdvanceSample());
            log.Add(loop.LastSampleTime);
            log.AddRange(loop.Simulation.State.ToArray());
            log.AddRange(loop.MeasuredQ.ToArray());
            log.AddRange(loop.MeasuredQDot.ToArray());
            log.AddRange(loop.Command.ToArray());
        }
        return [.. log];
    }

    [Fact]
    public void PerfectSensor_ReturnsTheTrueState()
    {
        var sensor = new PerfectSensor(4);
        var q = new double[4];
        var qd = new double[4];
        sensor.Measure(0.0, PoseC, PoseB, q, qd);
        Assert.Equal(PoseC, q);
        Assert.Equal(PoseB, qd);
    }

    [Fact]
    public void EncoderSensor_QuantisesTo2PiOver4096_AndDifferencesTheQuantisedAngles()
    {
        var sensor = new EncoderSensor(ArmModel.OpenManipulatorX());
        Assert.All(sensor.Resolution.ToArray(), r => Assert.Equal(s_encoderResolution, r, 1e-18));
        Assert.Equal(1.53e-3, s_encoderResolution, 1e-5);

        var r = s_encoderResolution;
        double[] q0 = [0.0, 10.4 * r, -3.6 * r, 0.49 * r];
        double[] q1 = [0.0, 12.6 * r, -3.6 * r, 0.51 * r];
        var zero = new double[4];
        var qm = new double[4];
        var vm = new double[4];

        sensor.Measure(0.0, q0, zero, qm, vm);
        Assert.Equal([0.0, 10 * r, -4 * r, 0.0], qm);
        Assert.Equal(zero, vm); // no previous sample yet

        sensor.Measure(0.002, q1, zero, qm, vm);
        Assert.Equal([0.0, 13 * r, -4 * r, 1 * r], qm);
        Assert.Equal(0.0, vm[0]);
        Assert.Equal(3 * r / 0.002, vm[1], 1e-12);
        Assert.Equal(0.0, vm[2]);
        Assert.Equal(r / 0.002, vm[3], 1e-12); // a 0.02-count move reads as a whole count: the speed noise

        sensor.Reset();
        sensor.Measure(1.0, q1, zero, qm, vm);
        Assert.Equal(zero, vm);
    }

    [Fact]
    public void EncoderSensor_Filter_IsFirstOrderLowPassOfTheDifferencedSpeed()
    {
        var r = s_encoderResolution;
        const double tau = 0.01;
        const double dt = 0.002;
        var sensor = new EncoderSensor(ArmModel.OpenManipulatorX(), new EncoderSensorOptions { FilterTimeConstant = tau });
        var zero = new double[4];
        var qm = new double[4];
        var vm = new double[4];
        sensor.Measure(0.0, zero, zero, qm, vm);
        double[] q = [5 * r, 0, 0, 0];
        sensor.Measure(dt, q, zero, qm, vm);
        var alpha = dt / (tau + dt);
        Assert.Equal(alpha * 5 * r / dt, vm[0], 1e-12);
        sensor.Measure(2 * dt, q, zero, qm, vm);
        Assert.Equal((1 - alpha) * alpha * 5 * r / dt, vm[0], 1e-12);
    }

    [Fact]
    public void EncoderSensor_Noise_IsSeededAndHasTheRequestedSpread()
    {
        const double sigma = 0.01; // large against the resolution, so quantisation hardly changes the spread
        var options = new EncoderSensorOptions { NoiseStdDev = sigma, Seed = 3 };
        var a = new EncoderSensor(ArmModel.OpenManipulatorX(), options);
        var b = new EncoderSensor(ArmModel.OpenManipulatorX(), options);
        var zero = new double[4];
        var qa = new double[4];
        var qb = new double[4];
        var v = new double[4];
        var sum = 0.0;
        var sumSq = 0.0;
        const int samples = 20_000;
        for (var k = 0; k < samples; k++)
        {
            a.Measure(k * 0.002, zero, zero, qa, v);
            b.Measure(k * 0.002, zero, zero, qb, v);
            Assert.Equal(qa, qb);
            foreach (var x in qa) { sum += x; sumSq += x * x; }
        }
        var n = 4.0 * samples;
        var std = Math.Sqrt(sumSq / n - (sum / n) * (sum / n));
        output.WriteLine(Invariant($"noise σ = {sigma}: measured spread {std:E4}, mean {sum / n:E2}"));
        Assert.InRange(std, 0.98 * sigma, 1.02 * sigma);
        Assert.InRange(sum / n, -2e-4, 2e-4);
    }

    /// <summary>
    /// T07 item D: with the encoder (quantisation 2π/4096 = 1.53e-3 rad, larger than V17's 1e-3 rad) the step of
    /// SPEC §7 still ends within 2 counts of the set point on every joint. CT needs the speed filter (see the next
    /// test).
    /// </summary>
    [Theory]
    [InlineData(ControllerKind.PdGravity, 0.0)]
    [InlineData(ControllerKind.PdGravity, 5e-3)]
    [InlineData(ControllerKind.Pid, 0.0)]
    [InlineData(ControllerKind.Pid, 5e-3)]
    [InlineData(ControllerKind.ComputedTorque, 5e-3)]
    public void EncoderSensor_StepResponse_EndsWithin2Counts(ControllerKind kind, double filterTimeConstant)
    {
        var sensor = new EncoderSensor(ArmModel.OpenManipulatorX(), new EncoderSensorOptions { FilterTimeConstant = filterTimeConstant });
        var r = StepResponse(kind, Planet.Earth, 0.0, PoseA, PoseB, duration: 3.0, sensor: sensor);
        output.WriteLine(Invariant($"{kind}, filter τ = {filterTimeConstant} s: final error {r.FinalError:E2} rad = {r.FinalError / s_encoderResolution:F2} counts, settled at {r.SettlingTime:F3} s"));
        Assert.Null(r.Fault);
        Assert.True(r.FinalError <= 2 * s_encoderResolution, Invariant($"{r.FinalError / s_encoderResolution} counts"));
    }

    /// <summary>
    /// A documented limitation (docs/DECISIONS.md D6): CT cancels the friction model at the measured speed, but
    /// without a filter the differenced encoder speed is 0 or at least one count per sample (0.77 rad/s at
    /// Ts = 2 ms), far coarser than the friction band ωs = 0.1 rad/s. The friction is then not cancelled at low
    /// speed, and the light wrist, whose CT stiffness is only M44·Kp ≈ 0.05 N·m/rad, creeps for seconds.
    /// </summary>
    [Fact]
    public void EncoderSensor_WithoutFilter_ComputedTorqueCreeps_Regression()
    {
        var sensor = new EncoderSensor(ArmModel.OpenManipulatorX());
        var r = StepResponse(ControllerKind.ComputedTorque, Planet.Earth, 0.0, PoseA, PoseB, duration: 3.0, sensor: sensor);
        output.WriteLine(Invariant($"CT, unfiltered encoder speed: error after 3 s {r.FinalError:E2} rad = {r.FinalError / s_encoderResolution:F1} counts"));
        Assert.Null(r.Fault);
        Assert.True(r.FinalError > 2 * s_encoderResolution);
    }

    /// <summary>
    /// T07 item D: the speed estimate's error is visible (it reaches a sizeable part of one count per sample) but
    /// bounded: |v − (q_k − q_{k−1})/Δt| ≤ r/Δt exactly, because each quantised angle is within r/2 of the true one.
    /// The filter reduces the error's RMS.
    /// </summary>
    [Fact]
    public void EncoderSensor_SpeedEstimateNoise_IsVisibleButBounded()
    {
        var (maxRaw, rmsRaw) = SpeedEstimateError(0.0, checkBound: true);
        var (maxFiltered, rmsFiltered) = SpeedEstimateError(5e-3, checkBound: false);
        var count = s_encoderResolution / ControlLoop.DefaultSamplePeriod;
        output.WriteLine(Invariant($"one count per sample = {count:F3} rad/s"));
        output.WriteLine(Invariant($"unfiltered speed error: max {maxRaw:F3} rad/s ({maxRaw / count:F2} counts/sample), RMS {rmsRaw:F3} rad/s"));
        output.WriteLine(Invariant($"filtered (τ = 5 ms):    max {maxFiltered:F3} rad/s, RMS {rmsFiltered:F3} rad/s (against the true speed)"));

        Assert.True(maxRaw > 0.5 * count, "the quantisation noise of the speed estimate should be visible");
        Assert.True(rmsFiltered < rmsRaw);
    }

    /// <summary>
    /// Runs the PDG quintic A → B with the encoder and compares the speed estimate with the true average speed over
    /// the last sample (unfiltered, to check the exact bound) or with the true speed (filtered). Returns max and RMS.
    /// </summary>
    private static (double Max, double Rms) SpeedEstimateError(double tau, bool checkBound)
    {
        var sensor = new EncoderSensor(ArmModel.OpenManipulatorX(), new EncoderSensorOptions { FilterTimeConstant = tau });
        var ctl = new PdGravity(ControllerBelief.OpenManipulatorX(Planet.Earth), PdGravity.DefaultGains);
        var loop = Loop(ctl, sensor, new QuinticReference(PoseA, PoseB, 1.0), Planet.Earth, 0.0, PoseA);
        var previous = loop.Simulation.Q.ToArray();
        var max = 0.0;
        var sumSq = 0.0;
        var count = 0;
        var ts = loop.SamplePeriod;
        var bound = s_encoderResolution / ts;
        for (var k = 0; k < 750; k++)
        {
            var q = loop.Simulation.Q.ToArray();
            var qd = loop.Simulation.QDot.ToArray();
            Assert.True(loop.AdvanceSample());
            if (k == 0) { previous = q; continue; }
            for (var i = 1; i < 4; i++)
            {
                var truth = checkBound ? (q[i] - previous[i]) / ts : qd[i];
                var error = Math.Abs(loop.MeasuredQDot[i] - truth);
                if (checkBound)
                    Assert.True(error <= bound * (1 + 1e-9), Invariant($"joint {i + 1}, sample {k}: {error} > {bound}"));
                max = Math.Max(max, error);
                sumSq += error * error;
                count++;
            }
            previous = q;
        }
        return (max, Math.Sqrt(sumSq / count));
    }

    /// <summary>PROJECT.md §3: Compute, Measure and AdvanceSample allocate nothing after warm-up.</summary>
    [Fact]
    public void ComputeMeasureAndAdvanceSample_AllocateNothing_AfterWarmUp()
    {
        var belief = ControllerBelief.OpenManipulatorX(Planet.Earth, 0.3);
        IController[] controllers =
        [
            new Pid(belief, Pid.DefaultGains),
            new PdGravity(belief, PdGravity.DefaultGains),
            new ComputedTorque(belief, ComputedTorque.DefaultGains),
        ];
        var sensor = new EncoderSensor(ArmModel.OpenManipulatorX(), new EncoderSensorOptions { NoiseStdDev = 1e-3, FilterTimeConstant = 4e-3, Seed = 1 });
        var loop = Loop(new ComputedTorque(belief, ComputedTorque.DefaultGains), sensor,
            new QuinticReference(PoseA, PoseB, 1.0), Planet.Earth, 0.3, PoseA);
        double[] zero = [0.0, 0.0, 0.0, 0.0];
        var u = new double[4];
        var qm = new double[4];
        var vm = new double[4];
        var probe = new EncoderSensor(ArmModel.OpenManipulatorX(), new EncoderSensorOptions { NoiseStdDev = 1e-3, FilterTimeConstant = 4e-3 });

        void Exercise(int k)
        {
            var reference = new JointReference(PoseB, zero, zero);
            foreach (var c in controllers) c.Compute(k * 0.002, PoseC, zero, reference, u);
            probe.Measure(k * 0.002, PoseC, zero, qm, vm);
        }

        for (var k = 0; k < 200; k++) { Exercise(k); loop.AdvanceSample(); }
        var before = GC.GetAllocatedBytesForCurrentThread();
        for (var k = 200; k < 400; k++) { Exercise(k); loop.AdvanceSample(); }
        var allocated = GC.GetAllocatedBytesForCurrentThread() - before;
        output.WriteLine(Invariant($"Compute ×3, Measure and AdvanceSample, 200 samples: {allocated} bytes"));
        Assert.False(loop.Simulation.IsStopped);
        Assert.Equal(0, allocated);
    }

    /// <summary>Records the time of every call and returns a constant command; for the hold test.</summary>
    private sealed class RecordingController(int n) : IController
    {
        private readonly bool[] _clamped = new bool[n];

        public List<double> Times { get; } = [];

        public double[] LastCommand { get; } = new double[n];

        public int DegreesOfFreedom => n;

        public ReadOnlySpan<bool> Clamped => _clamped;

        public void Reset() => Times.Clear();

        public void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
        {
            Times.Add(t);
            for (var i = 0; i < n; i++) u[i] = LastCommand[i] = 0.1 * (Times.Count % 3) + 0.01 * i;
        }
    }
}
