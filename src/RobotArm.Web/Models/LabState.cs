using System.Diagnostics;
using System.Globalization;
using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Kinematics;
using RobotArm.Core.Model;
using RobotArm.Core.Simulation;
using RobotArm.Core.Tasks;
using Sim = RobotArm.Core.Simulation.Simulation;

namespace RobotArm.Web.Models;

/// <summary>
/// Rolling 1-second performance metrics for the Performans overlay.
/// </summary>
public sealed record PerfMetrics(
    double AdvanceByMsPerFrame,
    double RenderMsPerFrame,
    double SimSecondsPerFrame,
    double RealTimeFactor,
    double LastStabilityMs);

/// <summary>
/// Stability and torque-budget analysis at the scenario's start pose.
/// Recomputed whenever a setup or gain parameter changes.
/// </summary>
public sealed record StabilityAnalysis(
    bool IsValid,
    string? ErrorMessage,
    double StepSeconds,
    double MaxStableStepSeconds,
    bool StepWarning,
    double SamplePeriodSeconds,
    double SpectralRadius,
    double MaxStableSamplePeriodSeconds,
    bool SampledLoopUnstable,
    bool ExcludesIntegralNote,
    double[] GravityTorqueNm,
    double[] StallTorqueNm,
    double[] TorqueUsagePercent,
    bool TorqueBudgetWarning);

/// <summary>
/// Holds the editable scenario state, validation errors, and stability estimates for the UI.
/// Numbers parsed or exported always use <see cref="CultureInfo.InvariantCulture"/>.
/// </summary>
public sealed class LabState
{
    public string SelectedLibraryName { get; private set; } = "";
    public string Name { get; set; } = "Deney";
    public string Description { get; set; } = "";
    public bool ExpectedSuccess { get; set; } = true;

    public string PlanetKey { get; set; } = Planet.Earth.Key;
    public double CustomGravity { get; set; } = Planet.Earth.G;
    public double PayloadKg { get; set; }

    public TaskKind TaskKind { get; set; } = TaskKind.Reach;
    public double[] StartQ { get; } = [0, -0.4, -0.3, 1.1];
    public double[] TargetMeters { get; } = [0.2023, 0.0, 0.2321];
    public double PitchRad { get; set; } = -0.1;
    public double MotionDuration { get; set; } = 2.0;
    public double[] ObjectPositionMeters { get; } = [0.2023, 0.0, 0.2321];
    public double PickPitchRad { get; set; } = -0.1;
    public double EventTimeout { get; set; } = 2.0;
    public TrackShape TrackShape { get; set; } = TrackShape.Circle;
    public double CircleRadiusMeters { get; set; } = 0.02;
    public double TrackingToleranceMeters { get; set; } = 0.002;
    public double[] DisturbanceNm { get; } = [0, 0.2, 0, 0];
    public double PulseStart { get; set; } = 1.0;
    public double PulseDuration { get; set; } = 0.1;

    public ControlMethod ControlMethod { get; set; } = ControlMethod.PDG;
    public bool IntegralEnabled { get; set; }
    public bool PredictHalfSample { get; set; } = true;
    public double[] Kp { get; } = [6.0, 6.0, 6.0, 2.0];
    public double[] Kd { get; } = [0.3, 0.6, 0.3, 0.05];
    public double[] Ki { get; } = [5.0, 20.0, 20.0, 8.0];

    public PayloadKnowledge Knowledge { get; set; } = PayloadKnowledge.Informed;
    public bool UseBeliefPayloadOverride { get; set; }
    public double BeliefPayloadKg { get; set; }
    public bool UseBeliefGravityOverride { get; set; }
    public double BeliefGravity { get; set; } = Planet.Earth.G;
    public double BeliefMassScale { get; set; } = 1.0;

    public double StepSeconds { get; set; } = Sim.DefaultStep;
    public double SamplePeriodSeconds { get; set; } = ControlLoop.DefaultSamplePeriod;
    public bool UseEncoder { get; set; }
    public double FilterTimeConstantSeconds { get; set; } = 0.005;
    public double NoiseStdDevRad { get; set; }
    public int Seed { get; set; } = 445;
    public double DurationSeconds { get; set; } = 10.0;

    // Design helper per-joint natural frequency ωn (rad/s) and damping ratio ζ
    public double[] DesignOmegaN { get; } = [12.0, 12.0, 12.0, 12.0];
    public double[] DesignZeta { get; } = [1.0, 1.0, 1.0, 1.0];

    private readonly record struct StabilityCoreEntry(
        double MaxStableStepSeconds,
        double MaxStableSamplePeriodSeconds,
        LinearSystem System,
        ILinearisableController Controller,
        double[] GravityTorqueNm,
        double[] StallTorqueNm,
        double[] TorqueUsagePercent,
        bool TorqueBudgetWarning);

    private static readonly ArmModel _baseModel = ArmModel.OpenManipulatorX();
    private readonly Dictionary<string, StabilityAnalysis> _stabilityCache = new(StringComparer.Ordinal);
    private readonly Dictionary<string, StabilityCoreEntry> _stabilityCoreCache = new(StringComparer.Ordinal);
    private string _currentStabilityKey = "";
    private string _cachedInertiaKey = "";
    private double[] _cachedInertiaDiag = [0.00247, 0.01433, 0.00490, 0.00037];

    public Dictionary<string, string> ValidationErrors { get; } = new(StringComparer.Ordinal);
    public StabilityAnalysis Stability { get; private set; } = DefaultStability();
    public double LastStabilityRecomputeMs { get; private set; }
    public bool IsStabilityCurrent => string.Equals(_currentStabilityKey, GetFullStabilityKey(), StringComparison.Ordinal);

    public LabState()
    {
        LoadScenario(ScenarioLibrary.All[0], recomputeStabilityNow: true);
    }

    public string ControllerKey =>
        ControlMethod == ControlMethod.CT && IntegralEnabled ? "CT+I" : ControlMethod.ToString();

    public void SetControllerByKey(string key, bool loadDefaultGains = true)
    {
        switch (key)
        {
            case "PID":
                ControlMethod = ControlMethod.PID;
                IntegralEnabled = false;
                break;
            case "PDG":
                ControlMethod = ControlMethod.PDG;
                IntegralEnabled = false;
                break;
            case "CT":
                ControlMethod = ControlMethod.CT;
                IntegralEnabled = false;
                break;
            case "CT+I":
                ControlMethod = ControlMethod.CT;
                IntegralEnabled = true;
                break;
        }
        if (loadDefaultGains)
        {
            ApplyDefaultGains();
        }
        RecomputeValidation();
    }

    public void ApplyDefaultGains()
    {
        if (ControlMethod == ControlMethod.PID)
        {
            for (var i = 0; i < 4; i++)
            {
                Kp[i] = Pid.DefaultGains.Kp[i];
                Kd[i] = Pid.DefaultGains.Kd[i];
                Ki[i] = Pid.DefaultGains.Ki[i];
            }
        }
        else if (ControlMethod == ControlMethod.PDG)
        {
            for (var i = 0; i < 4; i++)
            {
                Kp[i] = PdGravity.DefaultGains.Kp[i];
                Kd[i] = PdGravity.DefaultGains.Kd[i];
            }
        }
        else if (ControlMethod == ControlMethod.CT && !IntegralEnabled)
        {
            for (var i = 0; i < 4; i++)
            {
                Kp[i] = ComputedTorque.DefaultGains.Kp[i];
                Kd[i] = ComputedTorque.DefaultGains.Kd[i];
            }
        }
        else
        {
            for (var i = 0; i < 4; i++)
            {
                Kp[i] = ComputedTorque.DefaultIntegralPdGains.Kp[i];
                Kd[i] = ComputedTorque.DefaultIntegralPdGains.Kd[i];
                Ki[i] = ComputedTorque.DefaultIntegralGains[i];
            }
        }
    }

    public void LoadScenario(Scenario s, bool recomputeStabilityNow = false)
    {
        SelectedLibraryName = s.Name;
        Name = s.Name;
        Description = s.Description;
        ExpectedSuccess = s.ExpectedSuccess;
        PlanetKey = s.PlanetKey;
        CustomGravity = s.CustomGravity;
        PayloadKg = s.PayloadKg;

        TaskKind = s.Task.Kind;
        s.Task.Start.CopyTo(StartQ);
        TargetMeters[0] = s.Task.Target.X;
        TargetMeters[1] = s.Task.Target.Y;
        TargetMeters[2] = s.Task.Target.Z;
        PitchRad = s.Task.Pitch;
        MotionDuration = s.Task.MotionDuration;
        ObjectPositionMeters[0] = s.Task.ObjectPosition.X;
        ObjectPositionMeters[1] = s.Task.ObjectPosition.Y;
        ObjectPositionMeters[2] = s.Task.ObjectPosition.Z;
        PickPitchRad = s.Task.PickPitch;
        EventTimeout = s.Task.EventTimeout;
        TrackShape = s.Task.Shape;
        CircleRadiusMeters = s.Task.CircleRadius;
        TrackingToleranceMeters = s.Task.TrackingTolerance;
        s.Task.Disturbance.CopyTo(DisturbanceNm);
        PulseStart = s.Task.PulseStart;
        PulseDuration = s.Task.PulseDuration;

        ControlMethod = s.Controller.Method;
        IntegralEnabled = s.Controller.IntegralEnabled;
        PredictHalfSample = s.Controller.PredictHalfSample;
        ApplyDefaultGains();
        if (s.Controller.Kp is JointVector kp) kp.CopyTo(Kp);
        if (s.Controller.Kd is JointVector kd) kd.CopyTo(Kd);
        if (s.Controller.Ki is JointVector ki) ki.CopyTo(Ki);

        Knowledge = s.Belief.Knowledge;
        UseBeliefPayloadOverride = s.Belief.PayloadKg.HasValue;
        BeliefPayloadKg = s.Belief.PayloadKg ?? (s.Task.Kind == TaskKind.PickPlace ? 0.0 : s.PayloadKg);
        UseBeliefGravityOverride = s.Belief.Gravity.HasValue;
        BeliefGravity = s.Belief.Gravity ?? s.GetPlanet().G;
        BeliefMassScale = s.Belief.MassScale;

        StepSeconds = s.Step;
        SamplePeriodSeconds = s.SamplePeriod;
        UseEncoder = s.UseEncoder;
        FilterTimeConstantSeconds = s.FilterTimeConstant;
        NoiseStdDevRad = s.NoiseStdDev;
        Seed = s.Seed;
        DurationSeconds = s.Duration;

        RecomputeValidation();
        if (recomputeStabilityNow)
        {
            RecomputeStability();
        }
    }

    public Scenario BuildScenario()
    {
        var kiVector = (ControlMethod == ControlMethod.PID || (ControlMethod == ControlMethod.CT && IntegralEnabled))
            ? JointVector.From(Ki)
            : (JointVector?)null;

        return new Scenario
        {
            Name = Name,
            Description = Description,
            ExpectedSuccess = ExpectedSuccess,
            PlanetKey = PlanetKey,
            CustomGravity = CustomGravity,
            PayloadKg = PayloadKg,
            Task = new TaskSettings
            {
                Kind = TaskKind,
                Start = JointVector.From(StartQ),
                Target = new Vector3d(TargetMeters[0], TargetMeters[1], TargetMeters[2]),
                Pitch = PitchRad,
                Branch = ElbowBranch.Up,
                MotionDuration = MotionDuration,
                ObjectPosition = new Vector3d(ObjectPositionMeters[0], ObjectPositionMeters[1], ObjectPositionMeters[2]),
                PickPitch = PickPitchRad,
                EventTimeout = EventTimeout,
                Shape = TrackShape,
                CircleRadius = CircleRadiusMeters,
                TrackingTolerance = TrackingToleranceMeters,
                Disturbance = JointVector.From(DisturbanceNm),
                PulseStart = PulseStart,
                PulseDuration = PulseDuration
            },
            Controller = new ControllerSettings
            {
                Method = ControlMethod,
                IntegralEnabled = ControlMethod == ControlMethod.CT && IntegralEnabled,
                PredictHalfSample = PredictHalfSample,
                Kp = JointVector.From(Kp),
                Kd = JointVector.From(Kd),
                Ki = kiVector
            },
            Belief = new BeliefSettings
            {
                Knowledge = Knowledge,
                PayloadKg = UseBeliefPayloadOverride ? BeliefPayloadKg : null,
                Gravity = UseBeliefGravityOverride ? BeliefGravity : null,
                MassScale = BeliefMassScale
            },
            Step = StepSeconds,
            SamplePeriod = SamplePeriodSeconds,
            UseEncoder = UseEncoder,
            FilterTimeConstant = FilterTimeConstantSeconds,
            NoiseStdDev = NoiseStdDevRad,
            Seed = Seed,
            Duration = DurationSeconds
        };
    }

    /// <summary>
    /// Computes (and caches per belief/pose input set) the diagonal entries M̂ii (kg·m²)
    /// of the controller belief's inertia matrix at the start pose.
    /// </summary>
    public double[] GetBelievedInertiaDiagonalAtStart()
    {
        var key = FormattableString.Invariant(
            $"{StartQ[0]:R}|{StartQ[1]:R}|{StartQ[2]:R}|{StartQ[3]:R}|{TaskKind == TaskKind.PickPlace}|{PayloadKg:R}|{UseBeliefPayloadOverride}|{BeliefPayloadKg:R}|{BeliefMassScale:R}");
        if (string.Equals(_cachedInertiaKey, key, StringComparison.Ordinal))
        {
            return _cachedInertiaDiag;
        }

        var diag = new double[4];
        try
        {
            var belief = BuildControllerBeliefSafe();
            var m = new double[16];
            new RecursiveNewtonEuler(belief.Model).InertiaMatrix(StartQ, m);
            for (var i = 0; i < 4; i++)
            {
                diag[i] = m[i * 4 + i];
            }
        }
        catch
        {
            var m = new double[16];
            new RecursiveNewtonEuler(_baseModel).InertiaMatrix(new double[4], m);
            for (var i = 0; i < 4; i++)
            {
                diag[i] = m[i * 4 + i];
            }
        }

        _cachedInertiaKey = key;
        _cachedInertiaDiag = diag;
        return diag;
    }

    public void ApplyDesignHelperGains()
    {
        var mDiag = GetBelievedInertiaDiagonalAtStart();
        for (var i = 0; i < 4; i++)
        {
            var wn = Math.Max(0.0, DesignOmegaN[i]);
            var zeta = Math.Max(0.0, DesignZeta[i]);
            if (ControlMethod == ControlMethod.CT)
            {
                Kp[i] = Math.Round(wn * wn, 4);
                Kd[i] = Math.Round(2.0 * zeta * wn, 4);
                if (IntegralEnabled)
                {
                    Ki[i] = Math.Round(wn * wn * wn / 10.0, 4);
                }
            }
            else
            {
                Kp[i] = Math.Round(mDiag[i] * wn * wn, 4);
                Kd[i] = Math.Round(2.0 * zeta * wn * mDiag[i], 4);
            }
        }
        RecomputeValidation();
    }

    public void RecomputeValidation()
    {
        ValidationErrors.Clear();
        var model = _baseModel;

        if (PlanetKey == Planet.CustomKey && (!double.IsFinite(CustomGravity) || CustomGravity < 0 || CustomGravity > Planet.MaxCustomG))
        {
            ValidationErrors["CustomGravity"] = "Özel yerçekimi ivmesi g, 0 ile 30 m/s² arasında olmalıdır.";
        }

        if (!double.IsFinite(PayloadKg) || PayloadKg < 0 || PayloadKg > 0.5)
        {
            ValidationErrors["PayloadKg"] = "Yük kütlesi 0 ile 0,5 kg arasında olmalıdır.";
        }

        if (UseBeliefPayloadOverride && (!double.IsFinite(BeliefPayloadKg) || BeliefPayloadKg < 0 || BeliefPayloadKg > 0.5))
        {
            ValidationErrors["BeliefPayloadKg"] = "İnanılan yük kütlesi 0 ile 0,5 kg arasında olmalıdır.";
        }

        if (UseBeliefGravityOverride && (!double.IsFinite(BeliefGravity) || BeliefGravity < 0 || BeliefGravity > Planet.MaxCustomG))
        {
            ValidationErrors["BeliefGravity"] = "İnanılan yerçekimi ivmesi 0 ile 30 m/s² arasında olmalıdır.";
        }

        if (!double.IsFinite(BeliefMassScale) || BeliefMassScale <= 0)
        {
            ValidationErrors["BeliefMassScale"] = "Kütle ölçeği 0'dan büyük sonlu bir sayı olmalıdır.";
        }

        var startPoseValid = true;
        for (var i = 0; i < 4; i++)
        {
            var j = model.Joints[i];
            if (!double.IsFinite(StartQ[i]) || !j.IsWithinLimits(StartQ[i]))
            {
                startPoseValid = false;
                ValidationErrors[$"StartQ_{i}"] =
                    $"q{i + 1} başlangıç açısı [{LabFormat.Fixed(j.LowerLimit, 2)}, {LabFormat.Fixed(j.UpperLimit, 2)}] rad sınırları dışındadır.";
            }
        }

        if (startPoseValid)
        {
            var startTool = ForwardKinematics.ClosedForm(StartQ).Position;
            if (startTool.Z < 0)
            {
                ValidationErrors["StartPoseFloor"] =
                    $"Başlangıç pozunda uç nokta (z = {LabFormat.Fixed(startTool.Z * 1000, 1)} mm) zeminin (0 mm) altındadır.";
            }
        }

        // Validate task targets with IK on ElbowBranch.Up
        if (TaskKind is TaskKind.Reach or TaskKind.PickPlace || (TaskKind == TaskKind.Track && TrackShape == TrackShape.Line))
        {
            var targetVec = new Vector3d(TargetMeters[0], TargetMeters[1], TargetMeters[2]);
            var pitchToCheck = TaskKind == TaskKind.Track
                ? StartQ[1] + StartQ[2] + StartQ[3]
                : PitchRad;
            var ik = InverseKinematics.Solve(targetVec, pitchToCheck, ElbowBranch.Up, checkLimits: true);
            if (!ik.Success)
            {
                ValidationErrors["Target"] = ik.Failure == FailureReason.JointLimit
                    ? $"Hedef konum için ters kinematik çözümü {ik.LimitJointIndex + 1}. eklem sınırını aşıyor."
                    : "Hedef konum ve eğim robot kolun erişim alanı dışındadır (ters kinematik çözülemedi).";
            }
            if (targetVec.Z < 0)
            {
                ValidationErrors["Target"] = "Hedef Z yüksekliği zeminin (0 mm) altında olamaz.";
            }
        }

        if (TaskKind == TaskKind.PickPlace)
        {
            var objVec = new Vector3d(ObjectPositionMeters[0], ObjectPositionMeters[1], ObjectPositionMeters[2]);
            var ikObj = InverseKinematics.Solve(objVec, PickPitchRad, ElbowBranch.Up, checkLimits: true);
            if (!ikObj.Success)
            {
                ValidationErrors["ObjectPosition"] = ikObj.Failure == FailureReason.JointLimit
                    ? $"Nesne konumu için ters kinematik çözümü {ikObj.LimitJointIndex + 1}. eklem sınırını aşıyor."
                    : "Nesne konumu ve alma eğimi robot kolun erişim alanı dışındadır (ters kinematik çözülemedi).";
            }
            if (objVec.Z < 0)
            {
                ValidationErrors["ObjectPosition"] = "Nesne Z yüksekliği zeminin (0 mm) altında olamaz.";
            }
            if (!double.IsFinite(EventTimeout) || EventTimeout <= 0)
            {
                ValidationErrors["EventTimeout"] = "Yakalama/bırakma zaman aşımı 0 s'den büyük olmalıdır.";
            }
        }

        if (TaskKind == TaskKind.Track && TrackShape == TrackShape.Circle)
        {
            if (!double.IsFinite(CircleRadiusMeters) || CircleRadiusMeters <= 0)
            {
                ValidationErrors["CircleRadius"] = "Çember yarıçapı 0 mm'den büyük olmalıdır.";
            }
            if (!double.IsFinite(TrackingToleranceMeters) || TrackingToleranceMeters <= 0)
            {
                ValidationErrors["TrackingTolerance"] = "İzleme RMS hata eşiği 0 mm'den büyük olmalıdır.";
            }
        }

        if (TaskKind != TaskKind.Hold)
        {
            if (!double.IsFinite(MotionDuration) || MotionDuration <= 0)
            {
                ValidationErrors["MotionDuration"] = "Hareket süresi 0 s'den büyük olmalıdır.";
            }
            else if (double.IsFinite(DurationSeconds) && MotionDuration > DurationSeconds)
            {
                ValidationErrors["MotionDuration"] = "Hareket süresi toplam deney süresinden büyük olamaz.";
            }
        }
        else
        {
            if (!double.IsFinite(PulseStart) || PulseStart < 0 || PulseStart > 60)
            {
                ValidationErrors["PulseStart"] = "Darbe başlangıç zamanı 0 ile 60 s arasında olmalıdır.";
            }
            if (!double.IsFinite(PulseDuration) || PulseDuration < 0 || PulseDuration > 60)
            {
                ValidationErrors["PulseDuration"] = "Darbe süresi 0 ile 60 s arasında olmalıdır.";
            }
        }

        for (var i = 0; i < 4; i++)
        {
            if (!double.IsFinite(Kp[i]) || Kp[i] < 0 || !double.IsFinite(Kd[i]) || Kd[i] < 0 || !double.IsFinite(Ki[i]) || Ki[i] < 0)
            {
                ValidationErrors["Gains"] = "Tüm kontrolcü kazançları (Kp, Kd, Ki) 0 veya daha büyük sonlu sayılar olmalıdır.";
            }
        }

        if (!double.IsFinite(StepSeconds) || StepSeconds < Sim.MinStep || StepSeconds > Sim.MaxStep)
        {
            ValidationErrors["Step"] = "RK4 adım boyu h, 0,1 ms ile 5,0 ms arasında olmalıdır.";
        }

        if (!double.IsFinite(SamplePeriodSeconds) || SamplePeriodSeconds <= 0)
        {
            ValidationErrors["SamplePeriod"] = "Örnekleme periyodu Ts, 0 ms'den büyük olmalıdır.";
        }
        else if (double.IsFinite(StepSeconds) && StepSeconds > 0)
        {
            var ratio = Math.Round(SamplePeriodSeconds / StepSeconds);
            if (ratio < 1.0 || Math.Abs(ratio * StepSeconds - SamplePeriodSeconds) > 1e-9 * SamplePeriodSeconds)
            {
                ValidationErrors["SamplePeriod"] =
                    $"Örnekleme periyodu Ts ({LabFormat.Input(SamplePeriodSeconds * 1000)} ms), adım boyu h ({LabFormat.Input(StepSeconds * 1000)} ms) değerinin tam sayı katı olmalıdır.";
            }
        }

        if (!double.IsFinite(DurationSeconds) || DurationSeconds <= 0 || DurationSeconds > 60)
        {
            ValidationErrors["Duration"] = "Deney süresi (0, 60] s aralığında olmalıdır.";
        }
        else if (double.IsFinite(StepSeconds) && StepSeconds > 0)
        {
            if (Math.Abs(Math.Round(DurationSeconds / StepSeconds) * StepSeconds - DurationSeconds) > 1e-9 * DurationSeconds)
            {
                ValidationErrors["Duration"] = "Deney süresi adım boyu h değerinin tam sayı katı olmalıdır.";
            }
        }

        if (!double.IsFinite(FilterTimeConstantSeconds) || FilterTimeConstantSeconds < 0)
        {
            ValidationErrors["FilterTimeConstant"] = "Filtre zaman sabiti 0 ms veya daha büyük olmalıdır.";
        }

        if (!double.IsFinite(NoiseStdDevRad) || NoiseStdDevRad < 0)
        {
            ValidationErrors["NoiseStdDev"] = "Açı gürültüsü standart sapması 0 rad veya daha büyük olmalıdır.";
        }

        if (ValidationErrors.Count == 0)
        {
            try
            {
                var scenario = BuildScenario();
                scenario.Validate();
            }
            catch (Exception ex)
            {
                ValidationErrors["General"] = TranslateCoreException(ex);
            }
        }
    }

    public bool TryApplyCachedStability()
    {
        var fullKey = GetFullStabilityKey();
        if (string.Equals(_currentStabilityKey, fullKey, StringComparison.Ordinal))
        {
            return true;
        }
        if (_stabilityCache.TryGetValue(fullKey, out var cached))
        {
            Stability = cached;
            _currentStabilityKey = fullKey;
            LastStabilityRecomputeMs = 0.0;
            return true;
        }
        return false;
    }

    public void RecomputeStability()
    {
        var fullKey = GetFullStabilityKey();
        if (string.Equals(_currentStabilityKey, fullKey, StringComparison.Ordinal))
        {
            return;
        }
        if (_stabilityCache.TryGetValue(fullKey, out var cached))
        {
            Stability = cached;
            _currentStabilityKey = fullKey;
            LastStabilityRecomputeMs = 0.0;
            return;
        }

        var sw = Stopwatch.GetTimestamp();
        var computed = ComputeStabilityWithCoreCache(_baseModel);
        LastStabilityRecomputeMs = Stopwatch.GetElapsedTime(sw).TotalMilliseconds;
        Stability = computed;
        _currentStabilityKey = fullKey;
        _stabilityCache[fullKey] = computed;
    }

    private string GetCoreStabilityKey() => FormattableString.Invariant(
        $"{PlanetKey}|{(PlanetKey == Planet.CustomKey ? CustomGravity : 0.0):R}|{TaskKind == TaskKind.PickPlace}|{PayloadKg:R}|{StartQ[0]:R}|{StartQ[1]:R}|{StartQ[2]:R}|{StartQ[3]:R}|{ControlMethod}|{IntegralEnabled}|{PredictHalfSample}|{Kp[0]:R}|{Kp[1]:R}|{Kp[2]:R}|{Kp[3]:R}|{Kd[0]:R}|{Kd[1]:R}|{Kd[2]:R}|{Kd[3]:R}|{UseBeliefPayloadOverride}|{BeliefPayloadKg:R}|{UseBeliefGravityOverride}|{BeliefGravity:R}|{BeliefMassScale:R}");

    private string GetFullStabilityKey() => FormattableString.Invariant(
        $"{GetCoreStabilityKey()}|{StepSeconds:R}|{SamplePeriodSeconds:R}");

    private StabilityAnalysis ComputeStabilityWithCoreCache(ArmModel baseModel)
    {
        try
        {
            for (var i = 0; i < 4; i++)
            {
                if (!double.IsFinite(StartQ[i]))
                {
                    return DefaultStability("Başlangıç eklem açıları sonlu sayılar olmalıdır.");
                }
            }

            var coreKey = GetCoreStabilityKey();
            if (!_stabilityCoreCache.TryGetValue(coreKey, out var core))
            {
                var truePlanet = ResolveTruePlanetSafe();
                var initialPlantPayload = TaskKind == TaskKind.PickPlace ? 0.0 : Math.Clamp(PayloadKg, 0.0, 0.5);
                var startPlantModel = baseModel.WithPayload(initialPlantPayload);

                // 1. RK4 step-size limit at the start pose
                var maxStableStep = StabilityEstimate.MaxStableStep(startPlantModel, StartQ);

                // 2. Sampled closed-loop stability at the start pose
                var system = LinearSystem.ArmAtRest(baseModel, truePlanet, initialPlantPayload, StartQ);
                var belief = BuildControllerBeliefSafe();
                var safeKp = new double[4];
                var safeKd = new double[4];
                var safeKi = new double[4];
                for (var i = 0; i < 4; i++)
                {
                    safeKp[i] = double.IsFinite(Kp[i]) && Kp[i] >= 0 ? Kp[i] : 0.0;
                    safeKd[i] = double.IsFinite(Kd[i]) && Kd[i] >= 0 ? Kd[i] : 0.0;
                    safeKi[i] = double.IsFinite(Ki[i]) && Ki[i] >= 0 ? Ki[i] : 0.0;
                }

                ILinearisableController controller = ControlMethod switch
                {
                    ControlMethod.PID => new Pid(belief, new PidGains(safeKp, safeKi, safeKd)),
                    ControlMethod.PDG => new PdGravity(belief, new PdGains(safeKp, safeKd)),
                    _ => new ComputedTorque(belief, new PdGains(safeKp, safeKd), PredictHalfSample,
                        IntegralEnabled ? safeKi : null)
                };

                var maxStableTs = SampledLoopStability.MaxStableSamplePeriod(system, controller, StartQ);

                // 3. Torque budget at the start pose
                var budgetPayload = Math.Clamp(double.IsFinite(PayloadKg) ? PayloadKg : 0.0, 0.0, 0.5);
                var budgetModel = baseModel.WithPayload(budgetPayload);
                var rnea = new RecursiveNewtonEuler(budgetModel);
                var zeros = new double[4];
                var gTorque = new double[4];
                rnea.BiasTorque(StartQ, zeros, truePlanet, gTorque);

                var stall = new double[4];
                var usage = new double[4];
                var torqueWarning = false;
                for (var i = 0; i < 4; i++)
                {
                    stall[i] = budgetModel.Actuators[i].StallTorque;
                    usage[i] = 100.0 * Math.Abs(gTorque[i]) / stall[i];
                    if (usage[i] > 80.0)
                    {
                        torqueWarning = true;
                    }
                }

                core = new StabilityCoreEntry(
                    maxStableStep,
                    maxStableTs,
                    system,
                    controller,
                    gTorque,
                    stall,
                    usage,
                    torqueWarning);
                _stabilityCoreCache[coreKey] = core;
            }

            var stepWarning = double.IsFinite(StepSeconds) && StepSeconds > core.MaxStableStepSeconds;
            var tsForEval = double.IsFinite(SamplePeriodSeconds) && SamplePeriodSeconds > 0
                ? SamplePeriodSeconds
                : ControlLoop.DefaultSamplePeriod;
            var rho = SampledLoopStability.SpectralRadius(core.System, core.Controller, StartQ, tsForEval);
            var sampledLoopUnstable = !double.IsFinite(rho) || rho >= 1.0;
            var excludesIntegral = ControlMethod == ControlMethod.PID || (ControlMethod == ControlMethod.CT && IntegralEnabled);

            return new StabilityAnalysis(
                IsValid: true,
                ErrorMessage: null,
                StepSeconds: StepSeconds,
                MaxStableStepSeconds: core.MaxStableStepSeconds,
                StepWarning: stepWarning,
                SamplePeriodSeconds: tsForEval,
                SpectralRadius: rho,
                MaxStableSamplePeriodSeconds: core.MaxStableSamplePeriodSeconds,
                SampledLoopUnstable: sampledLoopUnstable,
                ExcludesIntegralNote: excludesIntegral,
                GravityTorqueNm: core.GravityTorqueNm,
                StallTorqueNm: core.StallTorqueNm,
                TorqueUsagePercent: core.TorqueUsagePercent,
                TorqueBudgetWarning: core.TorqueBudgetWarning);
        }
        catch (Exception ex)
        {
            return DefaultStability(ex.Message);
        }
    }

    private Planet ResolveTruePlanetSafe()
    {
        if (PlanetKey == Planet.CustomKey)
        {
            var g = double.IsFinite(CustomGravity) ? Math.Clamp(CustomGravity, 0.0, Planet.MaxCustomG) : Planet.Earth.G;
            return Planet.Custom(g);
        }
        return Planet.All.FirstOrDefault(p => p.Key == PlanetKey) ?? Planet.Earth;
    }

    private ControllerBelief BuildControllerBeliefSafe()
    {
        var baseModel = ArmModel.OpenManipulatorX();
        var scale = double.IsFinite(BeliefMassScale) && BeliefMassScale > 0 ? BeliefMassScale : 1.0;
        var believedBase = scale == 1.0
            ? baseModel
            : new ArmModel(
                baseModel.Joints,
                baseModel.Bodies.Select(b => new BodyParameters(b.Mass * scale, b.CenterOfMass, scale * b.Inertia)),
                baseModel.ToolPoint,
                baseModel.Actuators);

        var truePlanet = ResolveTruePlanetSafe();
        var believedPlanet = UseBeliefGravityOverride && double.IsFinite(BeliefGravity)
            ? Planet.Custom(Math.Clamp(BeliefGravity, 0.0, Planet.MaxCustomG))
            : truePlanet;

        var initialPlantPayload = TaskKind == TaskKind.PickPlace ? 0.0 : Math.Clamp(double.IsFinite(PayloadKg) ? PayloadKg : 0.0, 0.0, 0.5);
        var believedPayload = UseBeliefPayloadOverride && double.IsFinite(BeliefPayloadKg)
            ? Math.Clamp(BeliefPayloadKg, 0.0, 0.5)
            : initialPlantPayload;

        return new ControllerBelief(believedBase, believedPlanet, believedPayload);
    }

    private static StabilityAnalysis DefaultStability(string? error = null) => new(
        IsValid: false,
        ErrorMessage: error,
        StepSeconds: Sim.DefaultStep,
        MaxStableStepSeconds: double.PositiveInfinity,
        StepWarning: false,
        SamplePeriodSeconds: ControlLoop.DefaultSamplePeriod,
        SpectralRadius: 0,
        MaxStableSamplePeriodSeconds: double.PositiveInfinity,
        SampledLoopUnstable: false,
        ExcludesIntegralNote: false,
        GravityTorqueNm: new double[4],
        StallTorqueNm: [4.1, 4.1, 4.1, 4.1],
        TorqueUsagePercent: new double[4],
        TorqueBudgetWarning: false);

    public static string TranslateCoreException(Exception ex)
    {
        var msg = ex.Message;
        if (msg.Contains("integer multiple of the step h", StringComparison.OrdinalIgnoreCase))
            return "Örnekleme periyodu Ts, adım boyu h değerinin tam sayı katı olmalıdır.";
        if (msg.Contains("Duration must be an integer multiple of h", StringComparison.OrdinalIgnoreCase))
            return "Deney süresi adım boyu h değerinin tam sayı katı olmalıdır.";
        if (msg.Contains("Duration must be in (0,60]", StringComparison.OrdinalIgnoreCase))
            return "Deney süresi (0, 60] s aralığında olmalıdır.";
        if (msg.Contains("Initial pose is outside joint limits", StringComparison.OrdinalIgnoreCase))
            return "Başlangıç pozu eklem sınırları dışındadır.";
        if (msg.Contains("Initial tool is below the floor", StringComparison.OrdinalIgnoreCase))
            return "Başlangıç pozunda uç nokta zeminin (z = 0 mm) altındadır.";
        return $"Doğrulama hatası: {msg}";
    }

    public static string TranslateFailureOrFault(RunResult result)
    {
        if (result.Fault is Fault fault)
        {
            var model = ArmModel.OpenManipulatorX();
            return fault.Reason switch
            {
                FaultReason.JointLimit =>
                    $"Eklem Sınırı Aşıldı: t = {LabFormat.Fixed(fault.Time, 4)} s anında Eklem {fault.Joint + 1} açısı ({LabFormat.Fixed(fault.Value, 4)} rad), izin verilen [{LabFormat.Fixed(model.Joints[fault.Joint].LowerLimit, 2)}, {LabFormat.Fixed(model.Joints[fault.Joint].UpperLimit, 2)}] rad aralığının dışına çıktı.",
                FaultReason.Floor =>
                    $"Zemin Çarpışması: t = {LabFormat.Fixed(fault.Time, 4)} s anında uç nokta yüksekliği z = {LabFormat.Fixed(fault.Value * 1000, 2)} mm değerine inerek zeminin (z = 0 mm) altına geçti.",
                FaultReason.NonFinite =>
                    $"Sayısal Kararsızlık: t = {LabFormat.Fixed(fault.Time, 4)} s anında durum vektöründe sonlu olmayan bir değer oluştu.",
                _ => $"Fiziksel Hata: {result.FailureReason}"
            };
        }

        var reason = result.FailureReason;
        if (string.IsNullOrEmpty(reason))
            return "";
        if (reason.Contains("Reach: tool did not stay within 2 mm for 0.5 s", StringComparison.OrdinalIgnoreCase))
            return "Hedefe Ulaşma Başarısız: Uç nokta hedefe 2 mm mesafede en az 0,5 s boyunca kalamadı.";
        if (reason.Contains("PickPlace: pickup conditions not met", StringComparison.OrdinalIgnoreCase))
            return "Al ve Bırak Başarısız: Uç nokta nesneye 10 mm ve < 0,05 m/s yakalama koşulunu zaman aşımı içinde sağlayamadı.";
        if (reason.Contains("PickPlace: release target not reached", StringComparison.OrdinalIgnoreCase) ||
            reason.Contains("PickPlace: object was not released at the target", StringComparison.OrdinalIgnoreCase))
            return "Al ve Bırak Başarısız: Nesne hedef noktaya 5 mm tolerans içinde bırakılamadı.";
        if (reason.Contains("Track: RMS tool error exceeds", StringComparison.OrdinalIgnoreCase))
            return "Yörünge İzleme Başarısız: Uç nokta RMS izleme hatası belirlenen eşiği aştı veya hareket tamamlanmadı.";
        if (reason.Contains("Hold: pose was not held within 2 mm", StringComparison.OrdinalIgnoreCase))
            return "Poz Tutma Başarısız: Uç nokta deney boyunca 2 mm tolerans içinde tutulamadı.";
        if (reason.Contains("Hold: did not recover within 2 mm in 1 s", StringComparison.OrdinalIgnoreCase))
            return "Poz Tutma Başarısız: Bozucu darbe sonrasında uç nokta 1 s içinde 2 mm tolerans bölgesine geri dönemedi.";
        if (reason.Contains("Trajectory: Unreachable", StringComparison.OrdinalIgnoreCase))
            return "Yörünge Hatası: İstenen hedef veya yol üzerinde erişilemeyen nokta var.";
        if (reason.Contains("Trajectory: JointLimit", StringComparison.OrdinalIgnoreCase))
            return "Yörünge Hatası: İstenen hedef veya yol eklem sınırlarını aşıyor.";
        if (reason.Contains("Trajectory: Singular", StringComparison.OrdinalIgnoreCase))
            return "Yörünge Hatası: Kartezyen yol tekillik bölgesinden geçiyor.";
        if (reason.Contains("Trajectory: yaw branch cut", StringComparison.OrdinalIgnoreCase))
            return "Yörünge Hatası: Kartezyen yol negatif X ekseni üzerindeki açı sıçramasından geçiyor.";
        return reason;
    }
}

/// <summary>
/// Formatting and parsing helpers: displays numbers with a decimal comma, parses both ',' and '.' with
/// <see cref="CultureInfo.InvariantCulture"/>, and keeps exports in <see cref="CultureInfo.InvariantCulture"/>.
/// </summary>
public static class LabFormat
{
    public static bool TryParseInvariant(string? input, out double value)
    {
        if (string.IsNullOrWhiteSpace(input))
        {
            value = 0;
            return false;
        }
        var normalized = input.Trim().Replace(',', '.');
        return double.TryParse(normalized, NumberStyles.Float, CultureInfo.InvariantCulture, out value);
    }

    public static bool TryParseIntInvariant(string? input, out int value)
    {
        if (string.IsNullOrWhiteSpace(input))
        {
            value = 0;
            return false;
        }
        return int.TryParse(input.Trim(), NumberStyles.Integer, CultureInfo.InvariantCulture, out value);
    }

    public static string Input(double value, string format = "G6") =>
        double.IsFinite(value)
            ? value.ToString(format, CultureInfo.InvariantCulture).Replace('.', ',')
            : "";

    public static string Fixed(double value, int decimals = 2) =>
        value.ToString($"F{decimals}", CultureInfo.InvariantCulture).Replace('.', ',');
}
