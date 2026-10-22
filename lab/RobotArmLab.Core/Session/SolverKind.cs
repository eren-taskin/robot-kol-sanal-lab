using RobotArmLab.Core.Solvers;

namespace RobotArmLab.Core.Session;

/// <summary>Formda seçilebilen ADD çözücüleri.</summary>
public enum SolverKind
{
    /// <summary>Açık Euler, 1. mertebe, sabit adım.</summary>
    Euler,

    /// <summary>Heun (RK2), 2. mertebe, sabit adım.</summary>
    Heun,

    /// <summary>Klasik RK4, 4. mertebe, sabit adım.</summary>
    Rk4,

    /// <summary>Dormand–Prince RK45, uyarlamalı adım (rtol/atol ile).</summary>
    DormandPrince,
}

/// <summary>Seçilen çözücü türünden çözücü nesnesini kurar ve arayüzde gösterilecek adı verir.</summary>
public static class SolverFactory
{
    /// <summary>Açılır listede görünen adlar, <see cref="SolverKind"/> sırasıyla.</summary>
    public static string[] DisplayNames()
    {
        return new string[] { "Euler", "Heun", "RK4", "Dormand–Prince" };
    }

    /// <summary>Çözücü sabit adımlı mı? Sabit adımlıda kullanıcı h girer, Dormand–Prince'te rtol/atol girer.</summary>
    public static bool IsFixedStep(SolverKind kind)
    {
        return kind != SolverKind.DormandPrince;
    }

    /// <summary>
    /// Yeni bir çözücü nesnesi kurar. rtol ve atol yalnız Dormand–Prince için kullanılır. Her Reset'te yeni
    /// nesne kurulur; böylece Dormand–Prince'in önceki koşudan kalan adım tahmini sonucu etkilemez.
    /// </summary>
    public static IOdeSolver Create(SolverKind kind, double relativeTolerance, double absoluteTolerance)
    {
        switch (kind)
        {
            case SolverKind.Euler:
                return new EulerSolver();
            case SolverKind.Heun:
                return new HeunSolver();
            case SolverKind.DormandPrince:
                DormandPrinceSolver dp = new DormandPrinceSolver();
                dp.RelativeTolerance = relativeTolerance;
                dp.AbsoluteTolerance = absoluteTolerance;
                return dp;
            default:
                return new Rk4Solver();
        }
    }
}
