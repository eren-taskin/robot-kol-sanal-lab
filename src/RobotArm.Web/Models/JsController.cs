using System.Diagnostics;
using Microsoft.JSInterop;
using RobotArm.Core.Control;
using RobotArm.Core.Dynamics;
using RobotArm.Core.Model;

namespace RobotArm.Web.Models;

public sealed record CustomLawExample(string Key, string Title, string Description, string Code);

public sealed record DryRunResult(bool Ok, double ElapsedMs, int Calls, string? Error);

public sealed class CustomControllerFaultException(string message) : Exception(message);

public static class CustomControllerExamples
{
    public const string DefaultPdgCode = """
        function kontrol(t, q, qd, ref, model) {
          // 1. Kontrolcünün inanç modelinden yerçekimi tork vektörü g(q)'yu al (4 eleman, N·m)
          const g = model.gravity(q);
          // 2. Kurulum tablosundaki eklem başı Kp (N·m/rad) ve Kd (N·m·s/rad) kazançlarını oku
          const kp = model.kp;
          const kd = model.kd;
          // 3. Her eklem için u_i = Kp_i*(q_d,i - q_i) + Kd_i*(qd_d,i - qd_i) + g_i yasasını hesapla
          const u = [0, 0, 0, 0];
          for (let i = 0; i < 4; i++) {
            u[i] = kp[i] * (ref.q[i] - q[i]) + kd[i] * (ref.qd[i] - qd[i]) + g[i];
          }
          // 4. 4 eklem tork komutunu (N·m) döndür (±4,1 N·m sınırı otomatik uygulanır)
          return u;
        }
        """;

    public const string CtByHandCode = """
        function kontrol(t, q, qd, ref, model) {
          // 1. İnanç modelinden 4x4 atalet matrisi M(q) ve Coriolis+yerçekimi terimi h(q, qd)'yi al
          const M = model.M(q);
          const h = model.bias(q, qd);
          // 2. Dış çevrim doğal frekans ve sönüm kazançlarını tanımla (Kp = 400 1/s², Kd = 40 1/s)
          const kp = [400, 400, 400, 400];
          const kd = [40, 40, 40, 40];
          // 3. İstenen eklem ivmesi vektörü a_i = qdd_d,i + Kd_i*(qd_d,i - qd_i) + Kp_i*(q_d,i - q_i) oluştur
          const a = [0, 0, 0, 0];
          for (let i = 0; i < 4; i++) {
            a[i] = ref.qdd[i] + kd[i] * (ref.qd[i] - qd[i]) + kp[i] * (ref.q[i] - q[i]);
          }
          // 4. İnanç modelinin eklem sürtünme telafisini hesapla: fv*qd + fc*tanh(qd / omega_s)
          const tauF = [0, 0, 0, 0];
          for (let i = 0; i < 4; i++) {
            tauF[i] = 0.01 * qd[i] + 0.02 * Math.tanh(qd[i] / 0.1);
          }
          // 5. Hesaplanmış tork yasası u = M(q)*a + h(q, qd) + tau_f(qd) çarpımını yap ve döndür
          const u = [0, 0, 0, 0];
          for (let i = 0; i < 4; i++) {
            let ma = 0;
            for (let j = 0; j < 4; j++) {
              ma += M[i][j] * a[j];
            }
            u[i] = ma + h[i] + tauF[i];
          }
          return u;
        }
        """;

    public const string SlidingModeCode = """
        function kontrol(t, q, qd, ref, model) {
          // 1. İnanç modelinden yerçekimi tork vektörü g(q)'yu al (4 eleman, N·m)
          const g = model.gravity(q);
          // 2. Kayan yüzey eğimi lambda (1/s), anahtarlama kazancı ks (N·m), doğrusal kazanç kd ve sınır katmanı phi tanımla
          const lambda = [20.0, 10.0, 20.0, 40.0];
          const ks = [0.10, 0.20, 0.10, 0.02];
          const kd = [0.20, 0.40, 0.20, 0.03];
          const phi = [1.0, 1.0, 1.0, 1.0];
          // 3. Her eklem için kayan yüzey s_i = (qd_d,i - qd_i) + lambda_i*(q_d,i - q_i) değerini hesapla
          const u = [0, 0, 0, 0];
          for (let i = 0; i < 4; i++) {
            const e = ref.q[i] - q[i];
            const edot = ref.qd[i] - qd[i];
            const s = edot + lambda[i] * e;
            // 4. Çatırdamayı (chattering) önlemek için sınır katmanı içinde sürekli doyma fonksiyonu sat(s/phi) kullan
            const sat = s / (Math.abs(s) + phi[i]);
            // 5. Yerçekimi telafisi ve kayan kipli geri besleme torkunu birleştir: u_i = g_i + kd_i*s + ks_i*sat
            u[i] = g[i] + kd[i] * s + ks[i] * sat;
          }
          // 6. 4 eklem tork komutunu (N·m) döndür
          return u;
        }
        """;

    public static IReadOnlyList<CustomLawExample> All { get; } =
    [
        new("pdg", "1. PD + Yerçekimi Telafisi (Varsayılan, PDG eşdeğeri)",
            "İnanç modelinin g(q) yerçekimi telafisi ile eklem başı PD geri beslemesini birleştirir.",
            DefaultPdgCode),
        new("ct", "2. Hesaplanmış Tork (Elle Yazılmış CT)",
            "İnanç modelinin M(q) atalet matrisi ve h(q, q̇) terimini kullanarak u = M(q)a + h(q, q̇) + τf hesaplar.",
            CtByHandCode),
        new("smc", "3. Sınır Katmanlı Kayan Kipli Kontrol (SMC)",
            "s = ė + λe kayan yüzeyi ve φ sınır katmanı üzerinde çatırdamasız doyma fonksiyonu ile dayanıklı kontrol uygular.",
            SlidingModeCode)
    ];
}

/// <summary>
/// T10b: Student-written JavaScript controller implementing Core's <see cref="IController"/>.
/// Calls the student's <c>kontrol(t, q, qd, ref, model)</c> function synchronously via <see cref="IJSInProcessRuntime"/>.
/// <c>model.M(q)</c>, <c>model.gravity(q)</c> and <c>model.bias(q, qd)</c> call back to .NET synchronously and
/// compute from <see cref="Belief"/> (never from the true plant).
/// </summary>
public sealed class JsController : IController, IDisposable
{
    private readonly IJSInProcessRuntime _js;
    private readonly DotNetObjectReference<JsController> _selfRef;
    private readonly double[] _torqueLimit;
    private readonly bool[] _clamped = new bool[4];
    private readonly double[] _zero = new double[4];
    private readonly double[] _qArr = new double[4];
    private readonly double[] _qdArr = new double[4];
    private readonly double[] _qRefArr = new double[4];
    private readonly double[] _vRefArr = new double[4];
    private readonly double[] _aRefArr = new double[4];
    private readonly double[] _mFlat = new double[16];
    private readonly double[] _gBuf = new double[4];
    private readonly double[] _hBuf = new double[4];

    private RecursiveNewtonEuler _dynamics;
    private Vector3d _gravity;

    public JsController(
        ControllerBelief belief,
        IJSInProcessRuntime js,
        string sourceCode,
        ReadOnlySpan<double> kp,
        ReadOnlySpan<double> kd,
        ReadOnlySpan<double> ki)
    {
        ArgumentNullException.ThrowIfNull(belief);
        ArgumentNullException.ThrowIfNull(js);
        Belief = belief;
        _js = js;
        _dynamics = new RecursiveNewtonEuler(belief.Model);
        _gravity = belief.Planet.GravityVector;
        _torqueLimit = new double[4];
        for (var i = 0; i < 4; i++)
        {
            _torqueLimit[i] = belief.Model.Actuators[i].StallTorque;
        }

        _selfRef = DotNetObjectReference.Create(this);
        var prep = _js.Invoke<PrepareResult>(
            "RobotArmCustomController.prepareRuntime",
            sourceCode,
            _selfRef,
            kp.ToArray(),
            kd.ToArray(),
            ki.ToArray());

        if (!prep.Ok)
        {
            throw new CustomControllerFaultException(prep.Error ?? "Kendi kontrolcün derlenemedi.");
        }
    }

    private sealed record PrepareResult(bool Ok, string? Error);

    public ControllerBelief Belief { get; private set; }

    public int DegreesOfFreedom => 4;

    public ReadOnlySpan<bool> Clamped => _clamped;

    public string? SlowCallWarning { get; private set; }

    public double MaxCallMs { get; private set; }

    public void SetPayloadBelief(double payloadKg)
    {
        Belief = new ControllerBelief(Belief.BaseModel, Belief.Planet, payloadKg);
        _dynamics = new RecursiveNewtonEuler(Belief.Model);
        _gravity = Belief.Planet.GravityVector;
    }

    public void Reset()
    {
        Array.Clear(_clamped);
    }

    public void Compute(double t, ReadOnlySpan<double> q, ReadOnlySpan<double> qd, JointReference reference, Span<double> u)
    {
        q.CopyTo(_qArr);
        qd.CopyTo(_qdArr);
        reference.Position.CopyTo(_qRefArr);
        reference.Velocity.CopyTo(_vRefArr);
        reference.Acceleration.CopyTo(_aRefArr);

        var startTick = Stopwatch.GetTimestamp();
        double[]? res;
        try
        {
            res = _js.Invoke<double[]>("RobotArmCustomController.invokeKontrol", t, _qArr, _qdArr, _qRefArr, _vRefArr, _aRefArr);
        }
        catch (Exception)
        {
            throw new CustomControllerFaultException("Kontrolcün sayı olmayan bir değer döndürdü");
        }
        var callMs = Stopwatch.GetElapsedTime(startTick).TotalMilliseconds;

        if (callMs > MaxCallMs)
        {
            MaxCallMs = callMs;
        }
        if (callMs > 5.0 && SlowCallWarning is null)
        {
            SlowCallWarning = $"Uyarı: Kendi kontrolcün t = {LabFormat.Fixed(t, 3)} s adımında 5 ms sınırını aştı ({LabFormat.Fixed(callMs, 2)} ms).";
        }

        if (res is null || res.Length < 5 || res[4] != 1.0)
        {
            throw new CustomControllerFaultException("Kontrolcün sayı olmayan bir değer döndürdü");
        }

        for (var i = 0; i < 4; i++)
        {
            var raw = res[i];
            if (!double.IsFinite(raw))
            {
                throw new CustomControllerFaultException("Kontrolcün sayı olmayan bir değer döndürdü");
            }
            var limit = _torqueLimit[i];
            _clamped[i] = Math.Abs(raw) > limit;
            u[i] = Math.Clamp(raw, -limit, limit);
        }
    }

    [JSInvokable("ComputeM")]
    public double[][] ComputeM(double[] q)
    {
        if (q is null || q.Length != 4)
        {
            throw new ArgumentException("M(q) için 4 elemanlı q dizisi gereklidir.");
        }
        _dynamics.InertiaMatrix(q, _mFlat);
        return
        [
            [_mFlat[0], _mFlat[1], _mFlat[2], _mFlat[3]],
            [_mFlat[4], _mFlat[5], _mFlat[6], _mFlat[7]],
            [_mFlat[8], _mFlat[9], _mFlat[10], _mFlat[11]],
            [_mFlat[12], _mFlat[13], _mFlat[14], _mFlat[15]]
        ];
    }

    [JSInvokable("ComputeGravity")]
    public double[] ComputeGravity(double[] q)
    {
        if (q is null || q.Length != 4)
        {
            throw new ArgumentException("gravity(q) için 4 elemanlı q dizisi gereklidir.");
        }
        _dynamics.BiasTorque(q, _zero, _gravity, _gBuf);
        return [_gBuf[0], _gBuf[1], _gBuf[2], _gBuf[3]];
    }

    [JSInvokable("ComputeBias")]
    public double[] ComputeBias(double[] q, double[] qd)
    {
        if (q is null || q.Length != 4 || qd is null || qd.Length != 4)
        {
            throw new ArgumentException("bias(q, qd) için 4 elemanlı q ve qd dizileri gereklidir.");
        }
        _dynamics.BiasTorque(q, qd, _gravity, _hBuf);
        return [_hBuf[0], _hBuf[1], _hBuf[2], _hBuf[3]];
    }

    public void Dispose()
    {
        _js.InvokeVoid("RobotArmCustomController.disposeRuntime");
        _selfRef.Dispose();
    }
}
