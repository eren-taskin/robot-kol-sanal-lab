namespace RobotArm.Web.Models;

/// <summary>"Ne gözlemlemelisin?" hints shown under the plots for each library scenario (T10 item 5).</summary>
public static class ScenarioHints
{
    private static readonly Dictionary<string, string> Hints = new(StringComparer.Ordinal)
    {
        ["Dünya – boş kol – noktaya git"] =
            "Açı grafiğinde düz çizgiler kesikli referansları yakından izler; hata grafiğinde hatalar hareket bitince sıfıra iner. " +
            "Tork grafiğinde 2., 3. ve 4. eklemin torku hareket bitince sıfıra değil, kolu yerçekimine karşı taşıyan sabit bir değere oturur: bu, PDG'nin ĝ(q) telafisidir. " +
            "Hiçbir tork ±4,1 N·m sınırına yaklaşmaz, bu yüzden gölgeli bölge yoktur.",
        ["Ay – 0,3 kg – al ve bırak"] =
            "Dikey çizgiler yükün alındığı ve bırakıldığı anları gösterir. " +
            "Sağ eksende gerçek ve inanılan yük üst üste biner: kontrolcü yükten haberdardır, bu yüzden yük alınınca uç hatasında yalnızca kısa bir sıçrama görülür. " +
            "Ay'da g küçük olduğu için 0,3 kg yük torkları pek artırmaz; Dünya'da aynı deneyi deneyip tork grafiğini karşılaştırın.",
        ["Mars – çember izleme"] =
            "Çember çizilirken 1. eklem (taban) ileri geri döner, diğer eklemler de buna eşlik eder; referanslar yumuşak eğrilerdir. " +
            "Uç konum hatası grafiğinde hatanın hareket boyunca ne kadar küçük kaldığına ve hareket bitince sıfıra indiğine bakın. " +
            "Görev, hareket süresince ölçülen RMS uç hatası eşiğin altında kalırsa başarılı sayılır.",
        ["Yüksek g – 0,5 kg – tutma (doyum)"] =
            "Tork grafiğinde 2. eklemin torku daha ilk örnekte 4,1 N·m sınırına dayanır ve gölgeli kırpma bölgesi başlar: tutmak için gereken yaklaşık 6,84 N·m'yi motor veremez. " +
            "Kol düşmeye başlar, hata grafiği hızla büyür ve deney kısa sürede eklem sınırı hatasıyla durur. " +
            "Bu bir kontrolcü ayarı sorunu değil, motorun fiziksel sınırıdır: yükü ya da g'yi azaltmadan hiçbir kazanç bunu düzeltemez.",
        ["Yörünge – noktaya git"] =
            "Yerçekimi olmadığı için hareket bitince bütün torklar sıfıra iner. " +
            "Dünya senaryosuyla karşılaştırın (ikisini de karşılaştırmaya ekleyip τ2'yi üst üste koyun): orada 2. eklem sabit bir yerçekimi torku taşır. " +
            "PDG yerçekimini zaten telafi ettiği için iki deneyin hata eğrileri birbirine çok benzer.",
        ["Yanlış model – CT, 0,3 kg habersiz yük"] =
            "Sağ eksende kontrolcünün inandığı yük 0 kg, gerçek yük 0,3 kg'dır. " +
            "Hata grafiğinde özellikle 4. eklem (bilek) hatası hareket bitince sıfıra inmez ve kalıcı bir değerde kalır: CT'nin bilek sertliği M̂44·Kp çok düşüktür. " +
            "Bu koşuyu karşılaştırmaya ekleyin, sonra CT+I senaryosunu çalıştırıp e4 sinyalini üst üste koyun.",
        ["Yanlış model düzeltildi – CT+I, 0,3 kg habersiz yük"] =
            "Kontrolcü yine 0,3 kg yükten habersizdir ve başlangıçta hata yine büyür. " +
            "İntegral terimi kalıcı hatayı birikerek giderir: 4. eklem hatası zamanla sıfıra döner ve uç noktası hedefe yerleşir. " +
            "CT koşusuyla karşılaştırdığınızda fark ilk geçici hatada değil, hareket sonundaki kalıcı hatadadır.",
        ["Örnekleme – Ts sınırına yakın"] =
            "Ts, başlangıç pozundaki örneklemeli döngü kararlılık sınırının %90'ıdır; kararlılık panelinde ρ < 1 ve en büyük kararlı Ts görünür. " +
            "1. eklemdeki küçük bozucu darbe uç noktayı yalnızca birkaç yüzdebir milimetre iter; uç hatası grafiğinde bu tepe kısa sürede söner ve poz tutulur. " +
            "Ts'yi paneldeki en büyük kararlı Ts'nin biraz üstüne çıkarıp tekrar çalıştırın: doğrusal analize göre aynı küçük bozucu bu kez sönmemeli, büyümelidir."
    };

    /// <summary>The hint for a library scenario name, or null for an edited or imported scenario.</summary>
    public static string? For(string scenarioName) => Hints.GetValueOrDefault(scenarioName);
}
