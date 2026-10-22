namespace RobotArmLab.Core.Solvers;

/// <summary>
/// Adi diferansiyel denklemin sağ tarafı: ẋ = f(t, x). Kol için x = [q; q̇] (8 değişken) ve
/// f = [q̇; q̈]. Çözücüler bu fonksiyonu adım içinde birkaç kez çağırır.
/// </summary>
public delegate double[] OdeFunction(double t, double[] x);

/// <summary>
/// Sayısal ADD çözücüsü (integratör). Bir adımda x(t)'den x(t + h)'yi hesaplar. Form, kullanıcının
/// seçtiği çözücüyü bu arayüz üzerinden kullanır; böylece Euler, Heun, RK4 ve Dormand–Prince karşılaştırılabilir.
/// </summary>
public interface IOdeSolver
{
    /// <summary>
    /// x(t)'den x(t + h)'yi hesaplar ve yeni durum vektörünü döndürür (x değiştirilmez). h, s.
    /// Uyarlamalı çözücü bu aralığı kendi içinde daha küçük adımlara bölebilir.
    /// </summary>
    double[] Step(OdeFunction f, double t, double[] x, double h);
}
