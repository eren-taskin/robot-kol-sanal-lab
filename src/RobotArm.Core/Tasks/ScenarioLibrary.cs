using RobotArm.Core.Control;
using RobotArm.Core.Model;

namespace RobotArm.Core.Tasks;

/// <summary>Eight reproducible teaching experiments with Turkish explanations. SPEC §8.3.</summary>
public static class ScenarioLibrary
{
    /// <summary>Read-only library; every expected outcome is verified by tests. SPEC §8.3.</summary>
    public static IReadOnlyList<Scenario> All { get; } = Array.AsReadOnly(Create());

    /// <summary>Standard 10 s PickPlace experiment used by benchmarks and the report grid. SPEC §8.2.</summary>
    public static Scenario EarthPickPlace => new()
    {
        Name = "Dünya – al ve bırak", PayloadKg = 0.3,
        Task = new TaskSettings { Kind = TaskKind.PickPlace, Target = TaskGeometry.Tool(new(0.4, -0.2, -0.2, 0.3)) }
    };

    private static Scenario[] Create()
    {
        var reach = new Scenario { Controller = new ControllerSettings { Method = ControlMethod.PDG } };
        var mismatch = new Scenario
        {
            PayloadKg = 0.3, Task = new TaskSettings { MotionDuration = 1 },
            Belief = new BeliefSettings { PayloadKg = 0, Knowledge = PayloadKnowledge.Unaware }
        };
        var horizontal = TaskGeometry.Solve(new Vector3d(0.38, 0, OpenManipulatorXGeometry.D1), 0, Kinematics.ElbowBranch.Up);
        return
        [
            reach with { Name = "Dünya – boş kol – noktaya git", Description = "PDG, Dünya'da boş kolu hedefe başarıyla götürür ve yerçekimini dengeler." },
            EarthPickPlace with { Name = "Ay – 0,3 kg – al ve bırak", PlanetKey = "Moon", Description = "Yük bilgisini alan CT, Ay'da 0,3 kg nesneyi başarıyla alıp hedefe bırakır." },
            new Scenario { Name = "Mars – çember izleme", PlanetKey = "Mars", Task = new TaskSettings { Kind = TaskKind.Track, MotionDuration = 5 }, Description = "CT, Mars'ta düzgün bir çemberi RMS hata sınırı içinde başarıyla izler." },
            new Scenario
            {
                Name = "Yüksek g – 0,5 kg – tutma (doyum)", PlanetKey = "HighG", PayloadKg = 0.5,
                Controller = new ControllerSettings { Method = ControlMethod.PDG },
                Task = new TaskSettings { Kind = TaskKind.Hold, Start = horizontal, PulseDuration = 0 },
                ExpectedSuccess = false, Description = "Tutma başarısız olur: 2. eklemin 6,84 N·m yerçekimi torku 4,1 N·m motor sınırını aşar."
            },
            reach with { Name = "Yörünge – noktaya git", PlanetKey = "Orbit", Description = "PDG hedefe başarıyla ulaşır; yerçekimi olmadığından g telafisi sıfırdır ve etkisizdir." },
            mismatch with { Name = "Yanlış model – CT, 0,3 kg habersiz yük", ExpectedSuccess = false, Description = "Yükten habersiz CT'nin bileği sarkar; kalıcı hata yüzünden hedefe ulaşma başarısız olur." },
            mismatch with { Name = "Yanlış model düzeltildi – CT+I, 0,3 kg habersiz yük", Controller = new ControllerSettings { IntegralEnabled = true }, Description = "CT+I, 0,3 kg yükten habersiz olsa da integral ile kalıcı hatayı giderip hedefe başarıyla ulaşır." },
            NearSamplingLimit()
        ];
    }

    private static Scenario NearSamplingLimit()
    {
        var start = new JointVector(0, -0.1, 0.2, 0.2);
        var model = ArmModel.OpenManipulatorX(); var q = start.ToArray();
        var controller = new PdGravity(new ControllerBelief(model, Planet.Earth), PdGravity.DefaultGains);
        var system = LinearSystem.ArmAtRest(model, Planet.Earth, 0, q);
        var ts = 0.9 * SampledLoopStability.MaxStableSamplePeriod(system, controller, q);
        var step = ts / Math.Ceiling(ts / Simulation.Simulation.DefaultStep);
        return new Scenario
        {
            Name = "Örnekleme – Ts sınırına yakın", Description = "Başlangıç pozundaki Ts sınırının %90'ında PDG, küçük bozucudan sonra pozu başarıyla tutar; sınır yereldir.",
            Controller = new ControllerSettings { Method = ControlMethod.PDG }, Step = step, SamplePeriod = ts,
            Duration = Math.Ceiling(4 / step) * step,
            Task = new TaskSettings { Kind = TaskKind.Hold, Start = start, Disturbance = new(0.001, 0, 0, 0) }
        };
    }
}
