using RobotArmLab.Core.Solvers;

namespace RobotArmLab.Tests;

public class SolverTests
{
    private static double[] Decay(double t, double[] x) { return new[] { -x[0] }; }

    private static double[] Oscillator(double t, double[] x) { return new[] { x[1], -x[0] }; }

    // Global error at T of a fixed-step solver, against the exact solution.
    private static double GlobalError(IOdeSolver solver, OdeFunction f, double[] x0, double[] exact, double T, double h)
    {
        int n = (int)Math.Round(T / h);
        double[] x = x0;
        for (int k = 0; k < n; k++)
            x = solver.Step(f, k * h, x, h);
        double err = 0.0;
        for (int i = 0; i < x.Length; i++)
            err = Math.Max(err, Math.Abs(x[i] - exact[i]));
        return err;
    }

    public static TheoryData<string, double, double> Orders()
    {
        TheoryData<string, double, double> d = new TheoryData<string, double, double>();
        d.Add("Euler", 0.01, 2.0);
        d.Add("Heun", 0.01, 4.0);
        d.Add("RK4", 0.05, 16.0);
        return d;
    }

    private static IOdeSolver Make(string name)
    {
        if (name == "Euler") return new EulerSolver();
        if (name == "Heun") return new HeunSolver();
        return new Rk4Solver();
    }

    [Theory]
    [MemberData(nameof(Orders))]
    public void HalvingTheStepDividesTheErrorBy2ToThePower(string name, double h, double expectedRatio)
    {
        IOdeSolver s = Make(name);
        double T = 2.0;

        double e1 = GlobalError(s, Decay, new[] { 1.0 }, new[] { Math.Exp(-T) }, T, h);
        double e2 = GlobalError(s, Decay, new[] { 1.0 }, new[] { Math.Exp(-T) }, T, h / 2);
        Assert.InRange(e1 / e2, 0.8 * expectedRatio, 1.2 * expectedRatio);

        double[] exact = { Math.Cos(T), -Math.Sin(T) };
        double o1 = GlobalError(s, Oscillator, new[] { 1.0, 0.0 }, exact, T, h);
        double o2 = GlobalError(s, Oscillator, new[] { 1.0, 0.0 }, exact, T, h / 2);
        Assert.InRange(o1 / o2, 0.8 * expectedRatio, 1.2 * expectedRatio);
    }

    [Fact]
    public void DormandPrinceMeetsItsTolerance()
    {
        foreach (double rtol in new[] { 1e-4, 1e-6, 1e-8 })
        {
            DormandPrinceSolver dp = new DormandPrinceSolver();
            dp.RelativeTolerance = rtol;
            dp.AbsoluteTolerance = rtol * 1e-3;

            double[] x = dp.Step(Decay, 0.0, new[] { 1.0 }, 2.0);
            Assert.True(Math.Abs(x[0] - Math.Exp(-2.0)) <= rtol * Math.Exp(-2.0), $"decay rtol {rtol}: {Math.Abs(x[0] - Math.Exp(-2.0)):E3}");

            double[] y = new[] { 1.0, 0.0 };
            for (int k = 0; k < 20; k++)
                y = dp.Step(Oscillator, 0.1 * k, y, 0.1);   // 20 output intervals of 0.1 s, like display frames
            Assert.True(Math.Abs(y[0] - Math.Cos(2.0)) <= 10 * rtol, $"oscillator rtol {rtol}");
            Assert.True(Math.Abs(y[1] + Math.Sin(2.0)) <= 10 * rtol, $"oscillator rtol {rtol}");

            Assert.True(dp.LastAcceptedStepS > 0.0);
            Assert.True(dp.AcceptedStepCount > 0);
            Assert.True(dp.RejectedStepCount >= 0);
        }
    }

    [Fact]
    public void DormandPrinceTakesSmallerStepsForTighterTolerance()
    {
        DormandPrinceSolver loose = new DormandPrinceSolver(), tight = new DormandPrinceSolver();
        loose.RelativeTolerance = 1e-3; loose.AbsoluteTolerance = 1e-6;
        tight.RelativeTolerance = 1e-9; tight.AbsoluteTolerance = 1e-12;
        loose.Step(Oscillator, 0.0, new[] { 1.0, 0.0 }, 2.0);
        tight.Step(Oscillator, 0.0, new[] { 1.0, 0.0 }, 2.0);
        Assert.True(tight.AcceptedStepCount > 5 * loose.AcceptedStepCount);
    }
}
