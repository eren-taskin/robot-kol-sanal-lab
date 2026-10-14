namespace RobotArm.Core.Model;

/// <summary>
/// An environment: gravity of magnitude <see cref="G"/> acting along −z. SPEC §3.5.
/// </summary>
public sealed record Planet
{
    /// <summary>Largest custom g in m/s². SPEC §3.5.</summary>
    public const double MaxCustomG = 30.0;

    /// <summary>Key used for <see cref="Custom"/> planets.</summary>
    public const string CustomKey = "Custom";

    /// <summary>Stable key: Earth, Moon, Mars, HighG, Orbit or Custom. SPEC §3.5.</summary>
    public string Key { get; }

    /// <summary>Name shown in the user interface (Turkish). SPEC §3.5.</summary>
    public string DisplayName { get; }

    /// <summary>Gravity magnitude g in m/s². SPEC §3.5.</summary>
    public double G { get; }

    private Planet(string key, string displayName, double g)
    {
        Key = key;
        DisplayName = displayName;
        G = g;
    }

    /// <summary>Earth, g = 9.81 m/s². SPEC §3.5.</summary>
    public static Planet Earth { get; } = new("Earth", "Dünya", 9.81);

    /// <summary>Moon, g = 1.62 m/s². SPEC §3.5.</summary>
    public static Planet Moon { get; } = new("Moon", "Ay", 1.62);

    /// <summary>Mars, g = 3.71 m/s². SPEC §3.5.</summary>
    public static Planet Mars { get; } = new("Mars", "Mars", 3.71);

    /// <summary>High gravity at Jupiter level, g = 24.79 m/s². SPEC §3.5.</summary>
    public static Planet HighG { get; } = new("HighG", "Yüksek g (Jüpiter düzeyi)", 24.79);

    /// <summary>Orbit (microgravity), g = 0. SPEC §3.5.</summary>
    public static Planet Orbit { get; } = new("Orbit", "Yörünge (mikro yerçekimi)", 0.0);

    /// <summary>The five fixed environments in the order of SPEC §3.5.</summary>
    public static IReadOnlyList<Planet> All { get; } = Array.AsReadOnly([Earth, Moon, Mars, HighG, Orbit]);

    /// <summary>A custom environment with 0 ≤ g ≤ 30 m/s². SPEC §3.5.</summary>
    /// <exception cref="ArgumentOutOfRangeException">g not finite or outside 0..30 m/s².</exception>
    public static Planet Custom(double g)
    {
        if (!double.IsFinite(g) || g < 0.0 || g > MaxCustomG)
            throw new ArgumentOutOfRangeException(nameof(g), g, $"Custom g must be between 0 and {MaxCustomG} m/s² (SPEC §3.5).");
        return new Planet(CustomKey, "Özel", g);
    }

    /// <summary>Gravity vector (0, 0, −g) in the base frame. SPEC §3.5.</summary>
    public Vector3d GravityVector => new(0.0, 0.0, -G);
}
