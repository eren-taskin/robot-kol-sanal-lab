using System.Reflection;

namespace RobotArm.Tests;

/// <summary>PROJECT.md §2 rule 1: Core references nothing.</summary>
public sealed class DependencyRuleTests
{
    [Fact]
    public void CoreAssembly_ReferencesNoOtherRobotArmAssembly()
    {
        var core = Assembly.Load("RobotArm.Core");
        var coreName = core.GetName().Name;

        var offending = core.GetReferencedAssemblies()
            .Select(a => a.Name ?? string.Empty)
            .Where(n => n.StartsWith("RobotArm.", StringComparison.Ordinal) && n != coreName)
            .ToList();

        Assert.Equal("RobotArm.Core", coreName);
        Assert.Empty(offending);
    }
}
