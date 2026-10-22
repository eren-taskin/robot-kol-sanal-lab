namespace RobotArmLab.Forms
{
    partial class CirclePickPanel
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
            lblRadius = new System.Windows.Forms.Label();
            numRadiusMm = new System.Windows.Forms.NumericUpDown();
            lblPeriod = new System.Windows.Forms.Label();
            numPeriodS = new System.Windows.Forms.NumericUpDown();
            lblPlane = new System.Windows.Forms.Label();
            cmbPlane = new System.Windows.Forms.ComboBox();
            lblApproach = new System.Windows.Forms.Label();
            numApproachS = new System.Windows.Forms.NumericUpDown();
            lblRamp = new System.Windows.Forms.Label();
            numRampS = new System.Windows.Forms.NumericUpDown();
            chkPick = new System.Windows.Forms.CheckBox();
            numPickS = new System.Windows.Forms.NumericUpDown();
            lblPickNote = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)numRadiusMm).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numPeriodS).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numApproachS).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numRampS).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numPickS).BeginInit();
            SuspendLayout();
            lblRadius.AutoSize = true;
            lblRadius.Location = new System.Drawing.Point(8, 6);
            lblRadius.Text = "Çember yarıçapı (mm):";
            lblRadius.Name = "lblRadius";
            numRadiusMm.DecimalPlaces = 1;
            numRadiusMm.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numRadiusMm.Location = new System.Drawing.Point(160, 2);
            numRadiusMm.Maximum = new decimal(new int[] { 300, 0, 0, 0 });
            numRadiusMm.Minimum = new decimal(new int[] { 1, 0, 0, 0 });
            numRadiusMm.Size = new System.Drawing.Size(100, 23);
            numRadiusMm.Value = new decimal(new int[] { 30, 0, 0, 0 });
            numRadiusMm.Name = "numRadiusMm";
            numRadiusMm.TabIndex = 1;
            lblPeriod.AutoSize = true;
            lblPeriod.Location = new System.Drawing.Point(8, 36);
            lblPeriod.Text = "Tur süresi (s):";
            lblPeriod.Name = "lblPeriod";
            numPeriodS.DecimalPlaces = 2;
            numPeriodS.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            numPeriodS.Location = new System.Drawing.Point(160, 32);
            numPeriodS.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numPeriodS.Minimum = new decimal(new int[] { 1, 0, 0, 65536 });
            numPeriodS.Size = new System.Drawing.Size(100, 23);
            numPeriodS.Value = new decimal(new int[] { 4, 0, 0, 0 });
            numPeriodS.Name = "numPeriodS";
            numPeriodS.TabIndex = 3;
            lblPlane.AutoSize = true;
            lblPlane.Location = new System.Drawing.Point(8, 66);
            lblPlane.Text = "Düzlem:";
            lblPlane.Name = "lblPlane";
            cmbPlane.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbPlane.Location = new System.Drawing.Point(160, 62);
            cmbPlane.Size = new System.Drawing.Size(180, 23);
            cmbPlane.Name = "cmbPlane";
            cmbPlane.TabIndex = 5;
            cmbPlane.Items.AddRange(new object[] { "Yatay (z sabit)", "Düşey x–z (y sabit)" });
            lblApproach.AutoSize = true;
            lblApproach.Location = new System.Drawing.Point(8, 96);
            lblApproach.Text = "Yaklaşma süresi (s):";
            lblApproach.Name = "lblApproach";
            numApproachS.DecimalPlaces = 2;
            numApproachS.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            numApproachS.Location = new System.Drawing.Point(160, 92);
            numApproachS.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numApproachS.Minimum = new decimal(new int[] { 5, 0, 0, 131072 });
            numApproachS.Size = new System.Drawing.Size(100, 23);
            numApproachS.Value = new decimal(new int[] { 2, 0, 0, 0 });
            numApproachS.Name = "numApproachS";
            numApproachS.TabIndex = 7;
            lblRamp.AutoSize = true;
            lblRamp.Location = new System.Drawing.Point(8, 126);
            lblRamp.Text = "Yumuşak başlama (s):";
            lblRamp.Name = "lblRamp";
            numRampS.DecimalPlaces = 2;
            numRampS.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            numRampS.Location = new System.Drawing.Point(160, 122);
            numRampS.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numRampS.Minimum = new decimal(new int[] { 0, 0, 0, 0 });
            numRampS.Size = new System.Drawing.Size(100, 23);
            numRampS.Value = new decimal(new int[] { 2, 0, 0, 0 });
            numRampS.Name = "numRampS";
            numRampS.TabIndex = 9;
            chkPick.AutoSize = true;
            chkPick.Location = new System.Drawing.Point(8, 158);
            chkPick.Text = "Al-bırak: yükü kavrama anı (s):";
            chkPick.Name = "chkPick";
            chkPick.TabIndex = 10;
            numPickS.DecimalPlaces = 2;
            numPickS.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numPickS.Location = new System.Drawing.Point(250, 156);
            numPickS.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numPickS.Minimum = new decimal(new int[] { 0, 0, 0, 0 });
            numPickS.Size = new System.Drawing.Size(90, 23);
            numPickS.Value = new decimal(new int[] { 1, 0, 0, 0 });
            numPickS.Name = "numPickS";
            numPickS.TabIndex = 11;
            lblPickNote.Location = new System.Drawing.Point(8, 186);
            lblPickNote.Size = new System.Drawing.Size(336, 48);
            lblPickNote.Text = "Kol yüksüz başlar ve bu anda \"Ortam ve yük\" sekmesindeki yükü kavrar. Kontrolcünün tahmini yükü (Kontrol sekmesi) değişmez.";
            lblPickNote.Name = "lblPickNote";
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            AutoScroll = true;
            Controls.Add(lblRadius);
            Controls.Add(numRadiusMm);
            Controls.Add(lblPeriod);
            Controls.Add(numPeriodS);
            Controls.Add(lblPlane);
            Controls.Add(cmbPlane);
            Controls.Add(lblApproach);
            Controls.Add(numApproachS);
            Controls.Add(lblRamp);
            Controls.Add(numRampS);
            Controls.Add(chkPick);
            Controls.Add(numPickS);
            Controls.Add(lblPickNote);
            Name = "CirclePickPanel";
            Size = new System.Drawing.Size(350, 240);
            ((System.ComponentModel.ISupportInitialize)numRadiusMm).EndInit();
            ((System.ComponentModel.ISupportInitialize)numPeriodS).EndInit();
            ((System.ComponentModel.ISupportInitialize)numApproachS).EndInit();
            ((System.ComponentModel.ISupportInitialize)numRampS).EndInit();
            ((System.ComponentModel.ISupportInitialize)numPickS).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.Label lblRadius;
        private System.Windows.Forms.NumericUpDown numRadiusMm;
        private System.Windows.Forms.Label lblPeriod;
        private System.Windows.Forms.NumericUpDown numPeriodS;
        private System.Windows.Forms.Label lblPlane;
        private System.Windows.Forms.ComboBox cmbPlane;
        private System.Windows.Forms.Label lblApproach;
        private System.Windows.Forms.NumericUpDown numApproachS;
        private System.Windows.Forms.Label lblRamp;
        private System.Windows.Forms.NumericUpDown numRampS;
        private System.Windows.Forms.CheckBox chkPick;
        private System.Windows.Forms.NumericUpDown numPickS;
        private System.Windows.Forms.Label lblPickNote;
    }
}
