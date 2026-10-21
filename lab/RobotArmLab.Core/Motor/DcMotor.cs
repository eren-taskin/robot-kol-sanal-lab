using System;

namespace RobotArmLab.Core.Motor;

/// <summary>
/// DYNAMIXEL XM430-W350 servo motorunun gerilim → tork modeli, eklem çıkışında (dişliden sonra):
/// τ = N·Kt·(V − N·Ke·q̇) / R. Önce gerilim ±Vmax'a, sonra tork ±τmax'a kırpılır (doyum).
/// </summary>
/// <remarks>
/// Endüktans ihmal edilir: veri sayfası L vermez; bu boydaki küçük DC motorlarda elektriksel zaman sabiti
/// L/R tipik olarak 0,1 ms mertebesindedir, kolun mekanik zaman sabitleri ise milisaniyeler–saniyelerdir. Akım gerilime anında uyar kabul edilir; böylece duruma akım
/// eklenmez ve durum 8 değişkenli kalır (x = [q; q̇]).
///
/// Veri sayfası (ROBOTIS e-Manual, XM430-W350): 12,0 V'ta tutulma torku 4,1 N·m ve tutulma akımı 2,3 A,
/// yüksüz hız 46 rpm, dişli oranı 353,5:1. R, Kt ve Ke veri sayfasında yoktur; şöyle türetilir:
///   R    = V / I_stall           = 12 / 2,3            = 5,217 Ω   (q̇ = 0 iken akımı yalnız direnç sınırlar)
///   N·Kt = τ_stall / I_stall     = 4,1 / 2,3           = 1,783 N·m/A (çıkış tarafı)
///   N·Ke = V / ω_noload          = 12 / (46·2π/60)     = 2,491 V·s/rad (yüksüzde τ = 0 ⇒ V = N·Ke·q̇)
/// İdeal bir motorda Kt = Ke olurdu; buradaki fark dişli kutusunun verimini ve iç sürtünmesini bu iki
/// sabite toplu olarak yükler. Böylece model veri sayfasındaki iki noktayı (tutulma ve yüksüz hız) tam verir.
/// </remarks>
public class DcMotor
{
    /// <summary>Sargı direnci R, Ω.</summary>
    public double ResistanceOhm { get; set; }

    /// <summary>Motor tarafı tork sabiti Kt (dişliden önce), N·m/A.</summary>
    public double TorqueConstantNmPerA { get; set; }

    /// <summary>Motor tarafı zıt EMK sabiti Ke (dişliden önce), V·s/rad.</summary>
    public double BackEmfConstantVsPerRad { get; set; }

    /// <summary>Dişli oranı N (boyutsuz).</summary>
    public double GearRatio { get; set; }

    /// <summary>Uygulanabilecek en büyük gerilim Vmax, V (önerilen işletme gerilimi 12 V).</summary>
    public double MaxVoltageV { get; set; }

    /// <summary>
    /// Veri sayfasından türetilmiş XM430-W350 modeli (sınıf açıklamasındaki türetme). gearRatio, kol
    /// parametrelerindeki N ile aynı olsun diye dışarıdan verilir (varsayılan 353,5).
    /// </summary>
    public static DcMotor Xm430W350(double gearRatio)
    {
        double stallTorqueNm = 4.1, stallCurrentA = 2.3, voltageV = 12.0;
        double noLoadSpeedRadPerS = 46.0 * 2.0 * Math.PI / 60.0;   // 46 rpm = 4,817 rad/s

        DcMotor m = new DcMotor();
        m.GearRatio = gearRatio;
        m.MaxVoltageV = voltageV;
        m.ResistanceOhm = voltageV / stallCurrentA;
        m.TorqueConstantNmPerA = stallTorqueNm / stallCurrentA / gearRatio;
        m.BackEmfConstantVsPerRad = voltageV / noLoadSpeedRadPerS / gearRatio;
        return m;
    }

    /// <summary>
    /// Eklem çıkışındaki motor torku, N·m: τ = N·Kt·(V − N·Ke·q̇)/R, sonra ±torqueLimitNm doyumu.
    /// voltageV: komut gerilimi (±Vmax'a kırpılır), jointSpeedRadPerS: eklem hızı q̇. Eklem dönerken
    /// zıt EMK (N·Ke·q̇) etkin gerilimi azaltır; motor bu yüzden hız arttıkça daha az tork üretir.
    /// </summary>
    public double Torque(double voltageV, double jointSpeedRadPerS, double torqueLimitNm)
    {
        double v = Math.Max(-MaxVoltageV, Math.Min(MaxVoltageV, voltageV));
        double currentA = (v - GearRatio * BackEmfConstantVsPerRad * jointSpeedRadPerS) / ResistanceOhm;
        double tau = GearRatio * TorqueConstantNmPerA * currentA;
        return Math.Max(-torqueLimitNm, Math.Min(torqueLimitNm, tau));
    }

    /// <summary>Eklem dururken (q̇ = 0) verilen gerilimde üretilen tork N·Kt·V/R, N·m (doyumsuz).</summary>
    public double StallTorqueNm(double voltageV)
    {
        return GearRatio * TorqueConstantNmPerA * voltageV / ResistanceOhm;
    }

    /// <summary>Yük yokken (τ = 0) verilen gerilimde ulaşılan eklem hızı V/(N·Ke), rad/s.</summary>
    public double NoLoadSpeedRadPerS(double voltageV)
    {
        return voltageV / (GearRatio * BackEmfConstantVsPerRad);
    }

    /// <summary>
    /// Zıt EMK'nın eklemde yarattığı eşdeğer viskoz sönüm b = (N·Kt)(N·Ke)/R, N·m·s/rad (yaklaşık 0,85).
    /// Gerilim modunda kolu güçlü biçimde frenler ve sistemi sertleştirir; adım boyu seçerken önemlidir.
    /// </summary>
    public double BackEmfDampingNmsPerRad()
    {
        return GearRatio * TorqueConstantNmPerA * GearRatio * BackEmfConstantVsPerRad / ResistanceOhm;
    }
}
