namespace RobotArmLab.Forms
{
    partial class MainForm
    {
        /// <summary>Required designer variable.</summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>Clean up any resources being used.</summary>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            components = new System.ComponentModel.Container();
            toolStrip = new System.Windows.Forms.ToolStrip();
            btnStart = new System.Windows.Forms.ToolStripButton();
            btnPause = new System.Windows.Forms.ToolStripButton();
            btnReset = new System.Windows.Forms.ToolStripButton();
            sep1 = new System.Windows.Forms.ToolStripSeparator();
            lblSpeed = new System.Windows.Forms.ToolStripLabel();
            cmbSpeed = new System.Windows.Forms.ToolStripComboBox();
            sep2 = new System.Windows.Forms.ToolStripSeparator();
            btnSaveCsv = new System.Windows.Forms.ToolStripButton();
            sep3 = new System.Windows.Forms.ToolStripSeparator();
            lblSimTime = new System.Windows.Forms.ToolStripLabel();
            lblStatus = new System.Windows.Forms.ToolStripLabel();
            splitMain = new System.Windows.Forms.SplitContainer();
            tabSettings = new System.Windows.Forms.TabControl();
            pageParameters = new System.Windows.Forms.TabPage();
            propertyGrid = new System.Windows.Forms.PropertyGrid();
            btnDefaults = new System.Windows.Forms.Button();
            pageEnvironment = new System.Windows.Forms.TabPage();
            environmentPanel = new EnvironmentPanel();
            pageSolver = new System.Windows.Forms.TabPage();
            solverPanel = new SolverPanel();
            pageInput = new System.Windows.Forms.TabPage();
            inputPanel = new InputPanel();
            pageControl = new System.Windows.Forms.TabPage();
            controlPanel = new ControlPanel();
            pageTask = new System.Windows.Forms.TabPage();
            taskPanel = new TaskPanel();
            splitRight = new System.Windows.Forms.SplitContainer();
            splitViews = new System.Windows.Forms.SplitContainer();
            view3DHost = new System.Windows.Forms.Panel();
            armView2D = new ArmView2D();
            plotHost = new System.Windows.Forms.Panel();
            timer = new System.Windows.Forms.Timer(components);
            saveCsvDialog = new System.Windows.Forms.SaveFileDialog();
            toolStrip.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)splitMain).BeginInit();
            splitMain.Panel1.SuspendLayout();
            splitMain.Panel2.SuspendLayout();
            splitMain.SuspendLayout();
            tabSettings.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)splitRight).BeginInit();
            splitRight.Panel1.SuspendLayout();
            splitRight.Panel2.SuspendLayout();
            splitRight.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)splitViews).BeginInit();
            splitViews.Panel1.SuspendLayout();
            splitViews.Panel2.SuspendLayout();
            splitViews.SuspendLayout();
            SuspendLayout();
            //
            // toolStrip: Başlat, Duraklat, Sıfırla, Hız, CSV kaydet, simülasyon zamanı, durma nedeni
            //
            toolStrip.Items.AddRange(new System.Windows.Forms.ToolStripItem[] { btnStart, btnPause, btnReset, sep1, lblSpeed, cmbSpeed, sep2, btnSaveCsv, sep3, lblSimTime, lblStatus });
            toolStrip.Name = "toolStrip";
            btnStart.Text = "Başlat";
            btnStart.Click += btnStart_Click;
            btnPause.Text = "Duraklat";
            btnPause.Click += btnPause_Click;
            btnReset.Text = "Sıfırla";
            btnReset.Click += btnReset_Click;
            lblSpeed.Text = "Hız:";
            cmbSpeed.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbSpeed.Items.AddRange(new object[] { "0,25×", "0,5×", "1×", "2×", "4×" });
            cmbSpeed.Size = new System.Drawing.Size(70, 25);
            cmbSpeed.SelectedIndexChanged += cmbSpeed_SelectedIndexChanged;
            btnSaveCsv.Text = "CSV kaydet";
            btnSaveCsv.Click += btnSaveCsv_Click;
            lblSimTime.Text = "t = 0,000 s";
            lblStatus.ForeColor = System.Drawing.Color.DarkRed;
            lblStatus.Text = "";
            //
            // splitMain: solda ayar sekmeleri, sağda görünümler ve grafikler
            //
            splitMain.Dock = System.Windows.Forms.DockStyle.Fill;
            splitMain.Name = "splitMain";
            splitMain.Panel1.Controls.Add(tabSettings);
            splitMain.Panel2.Controls.Add(splitRight);
            splitMain.Size = new System.Drawing.Size(1280, 775);
            splitMain.SplitterDistance = 380;
            // tabSettings: Kol parametreleri, Ortam ve yük, Çözücü, Giriş, Kontrol (kapalı çevrim), Görev
            tabSettings.Controls.Add(pageParameters);
            tabSettings.Controls.Add(pageEnvironment);
            tabSettings.Controls.Add(pageSolver);
            tabSettings.Controls.Add(pageInput);
            tabSettings.Controls.Add(pageControl);
            tabSettings.Controls.Add(pageTask);
            tabSettings.Dock = System.Windows.Forms.DockStyle.Fill;
            tabSettings.Name = "tabSettings";
            pageParameters.Controls.Add(propertyGrid);
            pageParameters.Controls.Add(btnDefaults);
            pageParameters.Name = "pageParameters";
            pageParameters.Text = "Kol parametreleri";
            propertyGrid.Dock = System.Windows.Forms.DockStyle.Fill;
            propertyGrid.Name = "propertyGrid";
            btnDefaults.Dock = System.Windows.Forms.DockStyle.Bottom;
            btnDefaults.Name = "btnDefaults";
            btnDefaults.Size = new System.Drawing.Size(372, 30);
            btnDefaults.Text = "Varsayılana dön";
            btnDefaults.Click += btnDefaults_Click;
            pageEnvironment.Controls.Add(environmentPanel);
            pageEnvironment.Name = "pageEnvironment";
            pageEnvironment.Text = "Ortam ve yük";
            environmentPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            environmentPanel.Name = "environmentPanel";
            pageSolver.Controls.Add(solverPanel);
            pageSolver.Name = "pageSolver";
            pageSolver.Text = "Çözücü";
            solverPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            solverPanel.Name = "solverPanel";
            pageInput.Controls.Add(inputPanel);
            pageInput.Name = "pageInput";
            pageInput.Text = "Giriş";
            inputPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            inputPanel.Name = "inputPanel";
            pageControl.Controls.Add(controlPanel);
            pageControl.Name = "pageControl";
            pageControl.Text = "Kontrol";
            controlPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            controlPanel.Name = "controlPanel";
            pageTask.Controls.Add(taskPanel);
            pageTask.Name = "pageTask";
            pageTask.Text = "Görev";
            taskPanel.Dock = System.Windows.Forms.DockStyle.Fill;
            taskPanel.Name = "taskPanel";
            //
            // splitRight: üstte 3B ve 2B görünümler, altta grafikler
            //
            splitRight.Dock = System.Windows.Forms.DockStyle.Fill;
            splitRight.Name = "splitRight";
            splitRight.Orientation = System.Windows.Forms.Orientation.Horizontal;
            splitRight.Panel1.Controls.Add(splitViews);
            splitRight.Panel2.Controls.Add(plotHost);
            splitRight.Size = new System.Drawing.Size(896, 775);
            splitRight.SplitterDistance = 420;
            //
            // splitViews: solda 3B görünüm (çalışma anında eklenir), sağda 2B yan görünüm
            //
            splitViews.Dock = System.Windows.Forms.DockStyle.Fill;
            splitViews.Name = "splitViews";
            splitViews.Panel1.Controls.Add(view3DHost);
            splitViews.Panel2.Controls.Add(armView2D);
            splitViews.Size = new System.Drawing.Size(896, 420);
            splitViews.SplitterDistance = 480;
            view3DHost.Dock = System.Windows.Forms.DockStyle.Fill;
            view3DHost.Name = "view3DHost";
            armView2D.Dock = System.Windows.Forms.DockStyle.Fill;
            armView2D.Name = "armView2D";
            plotHost.Dock = System.Windows.Forms.DockStyle.Fill;
            plotHost.Name = "plotHost";
            // timer: ~30 ms'de bir simülasyonu ilerletir ve görünümleri yeniler
            timer.Interval = 30;
            timer.Tick += timer_Tick;
            saveCsvDialog.DefaultExt = "csv";
            saveCsvDialog.FileName = "simulasyon.csv";
            saveCsvDialog.Filter = "CSV dosyası (*.csv)|*.csv";
            //
            // MainForm
            //
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            ClientSize = new System.Drawing.Size(1280, 800);
            Controls.Add(splitMain);
            Controls.Add(toolStrip);
            Name = "MainForm";
            Text = "Robot Kol Sanal Laboratuvarı";
            FormClosing += MainForm_FormClosing;
            toolStrip.ResumeLayout(false);
            toolStrip.PerformLayout();
            splitMain.Panel1.ResumeLayout(false);
            splitMain.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)splitMain).EndInit();
            splitMain.ResumeLayout(false);
            tabSettings.ResumeLayout(false);
            splitRight.Panel1.ResumeLayout(false);
            splitRight.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)splitRight).EndInit();
            splitRight.ResumeLayout(false);
            splitViews.Panel1.ResumeLayout(false);
            splitViews.Panel2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)splitViews).EndInit();
            splitViews.ResumeLayout(false);
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.ToolStrip toolStrip;
        private System.Windows.Forms.ToolStripButton btnStart;
        private System.Windows.Forms.ToolStripButton btnPause;
        private System.Windows.Forms.ToolStripButton btnReset;
        private System.Windows.Forms.ToolStripSeparator sep1;
        private System.Windows.Forms.ToolStripLabel lblSpeed;
        private System.Windows.Forms.ToolStripComboBox cmbSpeed;
        private System.Windows.Forms.ToolStripSeparator sep2;
        private System.Windows.Forms.ToolStripButton btnSaveCsv;
        private System.Windows.Forms.ToolStripSeparator sep3;
        private System.Windows.Forms.ToolStripLabel lblSimTime;
        private System.Windows.Forms.ToolStripLabel lblStatus;
        private System.Windows.Forms.SplitContainer splitMain;
        private System.Windows.Forms.TabControl tabSettings;
        private System.Windows.Forms.TabPage pageParameters;
        private System.Windows.Forms.PropertyGrid propertyGrid;
        private System.Windows.Forms.Button btnDefaults;
        private System.Windows.Forms.TabPage pageEnvironment;
        private EnvironmentPanel environmentPanel;
        private System.Windows.Forms.TabPage pageSolver;
        private SolverPanel solverPanel;
        private System.Windows.Forms.TabPage pageInput;
        private InputPanel inputPanel;
        private System.Windows.Forms.TabPage pageControl;
        private ControlPanel controlPanel;
        private System.Windows.Forms.TabPage pageTask;
        private TaskPanel taskPanel;
        private System.Windows.Forms.SplitContainer splitRight;
        private System.Windows.Forms.SplitContainer splitViews;
        private System.Windows.Forms.Panel view3DHost;
        private ArmView2D armView2D;
        private System.Windows.Forms.Panel plotHost;
        private System.Windows.Forms.Timer timer;
        private System.Windows.Forms.SaveFileDialog saveCsvDialog;
    }
}
