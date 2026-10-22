namespace RobotArmLab.Forms
{
    partial class ControlPanel
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

        #region Component Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent()
        {
            rbOpen = new System.Windows.Forms.RadioButton();
            rbClosed = new System.Windows.Forms.RadioButton();
            lblController = new System.Windows.Forms.Label();
            cmbController = new System.Windows.Forms.ComboBox();
            gridGains = new System.Windows.Forms.PropertyGrid();
            lblEstimatedPayload = new System.Windows.Forms.Label();
            numEstimatedPayload = new System.Windows.Forms.NumericUpDown();
            lblSamplePeriod = new System.Windows.Forms.Label();
            numSamplePeriodMs = new System.Windows.Forms.NumericUpDown();
            lblSensor = new System.Windows.Forms.Label();
            cmbSensor = new System.Windows.Forms.ComboBox();
            lblCounts = new System.Windows.Forms.Label();
            numCounts = new System.Windows.Forms.NumericUpDown();
            lblNoise = new System.Windows.Forms.Label();
            numNoiseMrad = new System.Windows.Forms.NumericUpDown();
            lblSeed = new System.Windows.Forms.Label();
            numSeed = new System.Windows.Forms.NumericUpDown();
            lblActuator = new System.Windows.Forms.Label();
            cmbActuator = new System.Windows.Forms.ComboBox();
            lblMotorNote = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)numEstimatedPayload).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numSamplePeriodMs).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numCounts).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numNoiseMrad).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numSeed).BeginInit();
            SuspendLayout();
            rbOpen.AutoSize = true;
            rbOpen.Checked = true;
            rbOpen.Location = new System.Drawing.Point(8, 8);
            rbOpen.Text = "Açık çevrim";
            rbOpen.Name = "rbOpen";
            rbOpen.TabIndex = 0;
            rbClosed.AutoSize = true;
            rbClosed.Location = new System.Drawing.Point(130, 8);
            rbClosed.Text = "Kapalı çevrim";
            rbClosed.Name = "rbClosed";
            rbClosed.TabIndex = 1;
            lblController.AutoSize = true;
            lblController.Location = new System.Drawing.Point(8, 40);
            lblController.Text = "Kontrolcü:";
            lblController.Name = "lblController";
            cmbController.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbController.Location = new System.Drawing.Point(120, 36);
            cmbController.Size = new System.Drawing.Size(220, 23);
            cmbController.Name = "cmbController";
            cmbController.TabIndex = 3;
            cmbController.SelectedIndexChanged += cmbController_SelectedIndexChanged;
            gridGains.HelpVisible = true;
            gridGains.Location = new System.Drawing.Point(8, 68);
            gridGains.Size = new System.Drawing.Size(336, 200);
            gridGains.ToolbarVisible = false;
            gridGains.Name = "gridGains";
            gridGains.TabIndex = 4;
            lblEstimatedPayload.AutoSize = true;
            lblEstimatedPayload.Location = new System.Drawing.Point(8, 280);
            lblEstimatedPayload.Text = "Tahmini yük (kg):";
            lblEstimatedPayload.Name = "lblEstimatedPayload";
            numEstimatedPayload.DecimalPlaces = 3;
            numEstimatedPayload.Increment = new decimal(new int[] { 5, 0, 0, 131072 });
            numEstimatedPayload.Location = new System.Drawing.Point(160, 276);
            numEstimatedPayload.Maximum = new decimal(new int[] { 5, 0, 0, 65536 });
            numEstimatedPayload.Minimum = new decimal(new int[] { 0, 0, 0, 0 });
            numEstimatedPayload.Size = new System.Drawing.Size(100, 23);
            numEstimatedPayload.Value = new decimal(new int[] { 0, 0, 0, 0 });
            numEstimatedPayload.Name = "numEstimatedPayload";
            numEstimatedPayload.TabIndex = 6;
            lblSamplePeriod.AutoSize = true;
            lblSamplePeriod.Location = new System.Drawing.Point(8, 310);
            lblSamplePeriod.Text = "Örnekleme Ts (ms):";
            lblSamplePeriod.Name = "lblSamplePeriod";
            numSamplePeriodMs.DecimalPlaces = 3;
            numSamplePeriodMs.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            numSamplePeriodMs.Location = new System.Drawing.Point(160, 306);
            numSamplePeriodMs.Maximum = new decimal(new int[] { 50, 0, 0, 0 });
            numSamplePeriodMs.Minimum = new decimal(new int[] { 1, 0, 0, 65536 });
            numSamplePeriodMs.Size = new System.Drawing.Size(100, 23);
            numSamplePeriodMs.Value = new decimal(new int[] { 2, 0, 0, 0 });
            numSamplePeriodMs.Name = "numSamplePeriodMs";
            numSamplePeriodMs.TabIndex = 8;
            lblSensor.AutoSize = true;
            lblSensor.Location = new System.Drawing.Point(8, 340);
            lblSensor.Text = "Algılayıcı:";
            lblSensor.Name = "lblSensor";
            cmbSensor.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbSensor.Location = new System.Drawing.Point(160, 336);
            cmbSensor.Size = new System.Drawing.Size(180, 23);
            cmbSensor.Name = "cmbSensor";
            cmbSensor.TabIndex = 10;
            cmbSensor.Items.AddRange(new object[] { "Kusursuz", "Kodlayıcı (nicemleme)" });
            lblCounts.AutoSize = true;
            lblCounts.Location = new System.Drawing.Point(8, 370);
            lblCounts.Text = "Sayım / tur:";
            lblCounts.Name = "lblCounts";
            numCounts.DecimalPlaces = 0;
            numCounts.Increment = new decimal(new int[] { 1024, 0, 0, 0 });
            numCounts.Location = new System.Drawing.Point(160, 366);
            numCounts.Maximum = new decimal(new int[] { 1000000, 0, 0, 0 });
            numCounts.Minimum = new decimal(new int[] { 16, 0, 0, 0 });
            numCounts.Size = new System.Drawing.Size(100, 23);
            numCounts.Value = new decimal(new int[] { 4096, 0, 0, 0 });
            numCounts.Name = "numCounts";
            numCounts.TabIndex = 12;
            lblNoise.AutoSize = true;
            lblNoise.Location = new System.Drawing.Point(8, 400);
            lblNoise.Text = "Gürültü σ (mrad):";
            lblNoise.Name = "lblNoise";
            numNoiseMrad.DecimalPlaces = 3;
            numNoiseMrad.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            numNoiseMrad.Location = new System.Drawing.Point(160, 396);
            numNoiseMrad.Maximum = new decimal(new int[] { 100, 0, 0, 0 });
            numNoiseMrad.Minimum = new decimal(new int[] { 0, 0, 0, 0 });
            numNoiseMrad.Size = new System.Drawing.Size(100, 23);
            numNoiseMrad.Value = new decimal(new int[] { 0, 0, 0, 0 });
            numNoiseMrad.Name = "numNoiseMrad";
            numNoiseMrad.TabIndex = 14;
            lblSeed.AutoSize = true;
            lblSeed.Location = new System.Drawing.Point(8, 430);
            lblSeed.Text = "Gürültü tohumu:";
            lblSeed.Name = "lblSeed";
            numSeed.DecimalPlaces = 0;
            numSeed.Increment = new decimal(new int[] { 1, 0, 0, 0 });
            numSeed.Location = new System.Drawing.Point(160, 426);
            numSeed.Maximum = new decimal(new int[] { 1000000, 0, 0, 0 });
            numSeed.Minimum = new decimal(new int[] { 0, 0, 0, 0 });
            numSeed.Size = new System.Drawing.Size(100, 23);
            numSeed.Value = new decimal(new int[] { 1, 0, 0, 0 });
            numSeed.Name = "numSeed";
            numSeed.TabIndex = 16;
            lblActuator.AutoSize = true;
            lblActuator.Location = new System.Drawing.Point(8, 460);
            lblActuator.Text = "Eyleyici:";
            lblActuator.Name = "lblActuator";
            cmbActuator.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbActuator.Location = new System.Drawing.Point(160, 456);
            cmbActuator.Size = new System.Drawing.Size(180, 23);
            cmbActuator.Name = "cmbActuator";
            cmbActuator.TabIndex = 18;
            cmbActuator.Items.AddRange(new object[] { "İdeal tork", "Motor (gerilim, zıt EMK, 12 V)" });
            cmbActuator.SelectedIndexChanged += cmbActuator_SelectedIndexChanged;
            lblMotorNote.ForeColor = System.Drawing.Color.DarkRed;
            lblMotorNote.Location = new System.Drawing.Point(8, 488);
            lblMotorNote.Size = new System.Drawing.Size(336, 48);
            lblMotorNote.Text = "Motor eyleyicisinde çözücü adımı h ≤ 0,2 ms olmalıdır: zıt EMK hafif bileği çok sertleştirir (λ ≈ 5600 1/s), daha büyük adımda RK4 kararlılık sınırına takılır.";
            lblMotorNote.Visible = false;
            lblMotorNote.Name = "lblMotorNote";
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            AutoScroll = true;
            Controls.Add(rbOpen);
            Controls.Add(rbClosed);
            Controls.Add(lblController);
            Controls.Add(cmbController);
            Controls.Add(gridGains);
            Controls.Add(lblEstimatedPayload);
            Controls.Add(numEstimatedPayload);
            Controls.Add(lblSamplePeriod);
            Controls.Add(numSamplePeriodMs);
            Controls.Add(lblSensor);
            Controls.Add(cmbSensor);
            Controls.Add(lblCounts);
            Controls.Add(numCounts);
            Controls.Add(lblNoise);
            Controls.Add(numNoiseMrad);
            Controls.Add(lblSeed);
            Controls.Add(numSeed);
            Controls.Add(lblActuator);
            Controls.Add(cmbActuator);
            Controls.Add(lblMotorNote);
            Name = "ControlPanel";
            Size = new System.Drawing.Size(350, 540);
            ((System.ComponentModel.ISupportInitialize)numEstimatedPayload).EndInit();
            ((System.ComponentModel.ISupportInitialize)numSamplePeriodMs).EndInit();
            ((System.ComponentModel.ISupportInitialize)numCounts).EndInit();
            ((System.ComponentModel.ISupportInitialize)numNoiseMrad).EndInit();
            ((System.ComponentModel.ISupportInitialize)numSeed).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.RadioButton rbOpen;
        private System.Windows.Forms.RadioButton rbClosed;
        private System.Windows.Forms.Label lblController;
        private System.Windows.Forms.ComboBox cmbController;
        private System.Windows.Forms.PropertyGrid gridGains;
        private System.Windows.Forms.Label lblEstimatedPayload;
        private System.Windows.Forms.NumericUpDown numEstimatedPayload;
        private System.Windows.Forms.Label lblSamplePeriod;
        private System.Windows.Forms.NumericUpDown numSamplePeriodMs;
        private System.Windows.Forms.Label lblSensor;
        private System.Windows.Forms.ComboBox cmbSensor;
        private System.Windows.Forms.Label lblCounts;
        private System.Windows.Forms.NumericUpDown numCounts;
        private System.Windows.Forms.Label lblNoise;
        private System.Windows.Forms.NumericUpDown numNoiseMrad;
        private System.Windows.Forms.Label lblSeed;
        private System.Windows.Forms.NumericUpDown numSeed;
        private System.Windows.Forms.Label lblActuator;
        private System.Windows.Forms.ComboBox cmbActuator;
        private System.Windows.Forms.Label lblMotorNote;
    }
}
