using System.Text.Json;

namespace RobotArmLab.Tests;

/// <summary>reference/test_vectors.json (Python reference implementation of SPEC.md) loader.</summary>
internal static class TestVectors
{
    private static JsonDocument _doc;

    public static JsonElement Root
    {
        get
        {
            if (_doc == null)
                _doc = JsonDocument.Parse(File.ReadAllText(Path.Combine(AppContext.BaseDirectory, "test_vectors.json")));
            return _doc.RootElement;
        }
    }

    public static double[] Vector(JsonElement e)
    {
        double[] v = new double[e.GetArrayLength()];
        for (int i = 0; i < v.Length; i++)
            v[i] = e[i].GetDouble();
        return v;
    }

    public static double[,] Matrix(JsonElement e)
    {
        double[,] m = new double[4, 4];
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                m[i, j] = e[i][j].GetDouble();
        return m;
    }
}
