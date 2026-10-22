using System.Windows.Forms;

namespace RobotArmLab.Forms;

/// <summary>NumericUpDown kutularına güvenle değer yazmak için yardımcı.</summary>
public static class NumericBox
{
    /// <summary>
    /// Değeri kutunun en küçük ve en büyük değerine kırpıp yazar. NumericUpDown sınır dışı bir değerde istisna
    /// atar; oturumdaki değer (ör. elle verilmiş 2 kg yük) kutunun sınırını aşsa bile form çökmemelidir.
    /// </summary>
    public static void Set(NumericUpDown box, double value)
    {
        decimal v = (decimal)value;
        if (v < box.Minimum)
            v = box.Minimum;
        if (v > box.Maximum)
            v = box.Maximum;
        box.Value = v;
    }
}
