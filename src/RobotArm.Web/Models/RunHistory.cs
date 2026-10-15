using RobotArm.Core.Tasks;

namespace RobotArm.Web.Models;

/// <summary>
/// Up to four finished runs kept in memory for comparison (T10 item 3). Registered as a scoped service,
/// so the runs survive a visit to the robustness page. Nothing is stored outside the browser tab.
/// </summary>
public sealed class RunHistory
{
    public const int Capacity = 4;
    private readonly List<RunRecord> _runs = [];
    private int _nextId = 1;

    public IReadOnlyList<RunRecord> Runs => _runs;

    public bool Contains(RunResult result) => _runs.Any(r => ReferenceEquals(r.Result, result));

    /// <summary>Adds a finished run; when full, the oldest run is dropped. Returns the dropped run, if any.</summary>
    public RunRecord? Add(RunResult result, Scenario scenario)
    {
        if (Contains(result)) return null;
        RunRecord? dropped = null;
        if (_runs.Count >= Capacity)
        {
            dropped = _runs[0];
            _runs.RemoveAt(0);
        }
        var color = RunPlots.RunColors.First(c => _runs.All(r => r.Color != c));
        _runs.Add(new RunRecord(_nextId++, scenario.Name, color, scenario, result));
        return dropped;
    }

    public void Remove(int id) => _runs.RemoveAll(r => r.Id == id);

    public void Clear() => _runs.Clear();
}
