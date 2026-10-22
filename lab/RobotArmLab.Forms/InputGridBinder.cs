using System;
using System.Windows.Forms;
using RobotArmLab.Core.Session;
using RobotArmLab.Core.Simulation;

namespace RobotArmLab.Forms;

/// <summary>
/// "Giriş" sekmesindeki tabloyu (her eklem için bir satır) kurar, oturumdaki değerleri tabloya yazar ve
/// tablodan geri okur. Arayüzde açılar derece, oturumda radyandır; çevirme yalnız burada yapılır.
/// </summary>
public static class InputGridBinder
{
    private const int ColShape = 1, ColAmplitude = 2, ColStepTime = 3, ColFrequency = 4, ColQ0 = 5, ColQd0 = 6;
    private static readonly string[] ShapeNames = { "Sabit", "Basamak", "Sinüs" };   // InputShape sırasıyla

    /// <summary>Sütunları ve dört eklem satırını oluşturur. Form kurulurken bir kez çağrılır.</summary>
    public static void Setup(DataGridView grid)
    {
        grid.AllowUserToAddRows = false;
        grid.AllowUserToDeleteRows = false;
        grid.RowHeadersVisible = false;
        grid.AutoSizeColumnsMode = DataGridViewAutoSizeColumnsMode.Fill;

        DataGridViewTextBoxColumn joint = new DataGridViewTextBoxColumn();
        joint.HeaderText = "Eklem";
        joint.ReadOnly = true;
        grid.Columns.Add(joint);

        DataGridViewComboBoxColumn shape = new DataGridViewComboBoxColumn();
        shape.HeaderText = "Tür";
        shape.Items.AddRange(ShapeNames);
        grid.Columns.Add(shape);

        AddNumberColumn(grid, "Genlik (N·m)");
        AddNumberColumn(grid, "t0 (s)");
        AddNumberColumn(grid, "Frekans (Hz)");
        AddNumberColumn(grid, "q0 (°)");
        AddNumberColumn(grid, "q̇0 (°/s)");

        for (int i = 0; i < 4; i++)
            grid.Rows.Add("Eklem " + (i + 1), ShapeNames[0], 0.0, 0.0, 0.0, 0.0, 0.0);

        // Hücreye sayı olmayan bir şey yazılırsa varsayılan hata penceresi yerine satıra uyarı yazılır.
        grid.DataError += delegate (object sender, DataGridViewDataErrorEventArgs e)
        {
            e.ThrowException = false;
            grid.Rows[e.RowIndex].ErrorText = "Geçerli bir sayı yazın (ör. 0,5).";
        };
        grid.CellEndEdit += delegate (object sender, DataGridViewCellEventArgs e)
        {
            grid.Rows[e.RowIndex].ErrorText = "";
        };
    }

    /// <summary>Genlik sütununun birimini giriş moduna göre değiştirir: tork modunda N·m, gerilim modunda V.</summary>
    public static void SetMode(DataGridView grid, InputMode mode)
    {
        grid.Columns[ColAmplitude].HeaderText = mode == InputMode.Voltage ? "Genlik (V)" : "Genlik (N·m)";
    }

    /// <summary>Oturumdaki girişleri ve başlangıç durumunu tabloya yazar (radyan → derece).</summary>
    public static void Load(DataGridView grid, LabSession session)
    {
        for (int i = 0; i < 4; i++)
        {
            DataGridViewRow row = grid.Rows[i];
            JointInput u = session.Inputs[i];
            row.Cells[ColShape].Value = ShapeNames[(int)u.Shape];
            row.Cells[ColAmplitude].Value = u.Amplitude;
            row.Cells[ColStepTime].Value = u.StepTimeS;
            row.Cells[ColFrequency].Value = u.FrequencyHz;
            row.Cells[ColQ0].Value = RadToDeg(session.InitialQRad[i]);
            row.Cells[ColQd0].Value = RadToDeg(session.InitialQdRadPerS[i]);
        }
        SetMode(grid, session.Mode);
    }

    /// <summary>
    /// Tablodaki değerleri oturuma yazar (derece → radyan). Değerlerin anlamlı olup olmadığını burası değil,
    /// oturumun Reset/Validate metodu denetler.
    /// </summary>
    public static void Read(DataGridView grid, LabSession session)
    {
        grid.EndEdit();   // düzenlenmekte olan hücrenin değeri de alınsın
        for (int i = 0; i < 4; i++)
        {
            DataGridViewRow row = grid.Rows[i];
            JointInput u = new JointInput();
            u.Shape = (InputShape)Array.IndexOf(ShapeNames, (string)row.Cells[ColShape].Value);
            u.Amplitude = CellNumber(row, ColAmplitude);
            u.StepTimeS = CellNumber(row, ColStepTime);
            u.FrequencyHz = CellNumber(row, ColFrequency);
            session.Inputs[i] = u;
            session.InitialQRad[i] = DegToRad(CellNumber(row, ColQ0));
            session.InitialQdRadPerS[i] = DegToRad(CellNumber(row, ColQd0));
        }
    }

    private static void AddNumberColumn(DataGridView grid, string header)
    {
        DataGridViewTextBoxColumn column = new DataGridViewTextBoxColumn();
        column.HeaderText = header;
        column.ValueType = typeof(double);   // tablo yazılanı bilgisayarın dil ayarıyla (Türkçe'de virgül) sayıya çevirir
        grid.Columns.Add(column);
    }

    // Boş hücre 0 sayılır; ValueType double olduğundan dolu hücre zaten sayıdır.
    private static double CellNumber(DataGridViewRow row, int column)
    {
        object value = row.Cells[column].Value;
        if (value == null || value == DBNull.Value)
            return 0.0;
        return Convert.ToDouble(value);
    }

    private static double DegToRad(double deg) { return deg * Math.PI / 180.0; }

    private static double RadToDeg(double rad) { return rad * 180.0 / Math.PI; }
}
