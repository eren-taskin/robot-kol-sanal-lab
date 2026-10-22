namespace RobotArmLab.Forms
{
    partial class TaskPanel
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
            lblTask = new System.Windows.Forms.Label();
            cmbTask = new System.Windows.Forms.ComboBox();
            lblAngles = new System.Windows.Forms.Label();
            numQ1 = new System.Windows.Forms.NumericUpDown();
            numQ2 = new System.Windows.Forms.NumericUpDown();
            numQ3 = new System.Windows.Forms.NumericUpDown();
            numQ4 = new System.Windows.Forms.NumericUpDown();
            lblXyz = new System.Windows.Forms.Label();
            numX = new System.Windows.Forms.NumericUpDown();
            numY = new System.Windows.Forms.NumericUpDown();
            numZ = new System.Windows.Forms.NumericUpDown();
            lblPitch = new System.Windows.Forms.Label();
            numPitchDeg = new System.Windows.Forms.NumericUpDown();
            chkElbowUp = new System.Windows.Forms.CheckBox();
            lblStart = new System.Windows.Forms.Label();
            numStartS = new System.Windows.Forms.NumericUpDown();
            lblDuration = new System.Windows.Forms.Label();
            numDurationS = new System.Windows.Forms.NumericUpDown();
            circlePickPanel = new CirclePickPanel();
            ((System.ComponentModel.ISupportInitialize)numQ1).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numQ2).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numQ3).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numQ4).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numX).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numY).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numZ).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numPitchDeg).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numStartS).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numDurationS).BeginInit();
            SuspendLayout();
            lblTask.AutoSize = true;
            lblTask.Location = new System.Drawing.Point(8, 12);
            lblTask.Text = "Görev:";
            lblTask.Name = "lblTask";
            cmbTask.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbTask.Location = new System.Drawing.Point(120, 8);
            cmbTask.Size = new System.Drawing.Size(220, 23);
            cmbTask.Name = "cmbTask";
            cmbTask.TabIndex = 1;
            cmbTask.SelectedIndexChanged += cmbTask_SelectedIndexChanged;
            lblAngles.AutoSize = true;
            lblAngles.Location = new System.Drawing.Point(8, 44);
            lblAngles.Text = "Hedef açılar q1..q4 (°):";
            lblAngles.Name = "lblAngles";
            numQ1.DecimalPlaces = 1;
            numQ1.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numQ1.Location = new System.Drawing.Point(8, 64);
            numQ1.Maximum = new decimal(new int[] { 180, 0, 0, 0 });
            numQ1.Minimum = new decimal(new int[] { 180, 0, 0, -2147483648 });
            numQ1.Size = new System.Drawing.Size(76, 23);
            numQ1.Value = new decimal(new int[] { 286, 0, 0, 65536 });
            numQ1.Name = "numQ1";
            numQ1.TabIndex = 3;
            numQ2.DecimalPlaces = 1;
            numQ2.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numQ2.Location = new System.Drawing.Point(92, 64);
            numQ2.Maximum = new decimal(new int[] { 180, 0, 0, 0 });
            numQ2.Minimum = new decimal(new int[] { 180, 0, 0, -2147483648 });
            numQ2.Size = new System.Drawing.Size(76, 23);
            numQ2.Value = new decimal(new int[] { 115, 0, 0, -2147418112 });
            numQ2.Name = "numQ2";
            numQ2.TabIndex = 4;
            numQ3.DecimalPlaces = 1;
            numQ3.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numQ3.Location = new System.Drawing.Point(176, 64);
            numQ3.Maximum = new decimal(new int[] { 180, 0, 0, 0 });
            numQ3.Minimum = new decimal(new int[] { 180, 0, 0, -2147483648 });
            numQ3.Size = new System.Drawing.Size(76, 23);
            numQ3.Value = new decimal(new int[] { 115, 0, 0, 65536 });
            numQ3.Name = "numQ3";
            numQ3.TabIndex = 5;
            numQ4.DecimalPlaces = 1;
            numQ4.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numQ4.Location = new System.Drawing.Point(260, 64);
            numQ4.Maximum = new decimal(new int[] { 180, 0, 0, 0 });
            numQ4.Minimum = new decimal(new int[] { 180, 0, 0, -2147483648 });
            numQ4.Size = new System.Drawing.Size(76, 23);
            numQ4.Value = new decimal(new int[] { 458, 0, 0, 65536 });
            numQ4.Name = "numQ4";
            numQ4.TabIndex = 6;
            lblXyz.AutoSize = true;
            lblXyz.Location = new System.Drawing.Point(8, 96);
            lblXyz.Text = "Hedef x, y, z (mm):";
            lblXyz.Name = "lblXyz";
            numX.DecimalPlaces = 1;
            numX.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numX.Location = new System.Drawing.Point(8, 116);
            numX.Maximum = new decimal(new int[] { 500, 0, 0, 0 });
            numX.Minimum = new decimal(new int[] { 500, 0, 0, -2147483648 });
            numX.Size = new System.Drawing.Size(100, 23);
            numX.Value = new decimal(new int[] { 200, 0, 0, 0 });
            numX.Name = "numX";
            numX.TabIndex = 8;
            numY.DecimalPlaces = 1;
            numY.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numY.Location = new System.Drawing.Point(120, 116);
            numY.Maximum = new decimal(new int[] { 500, 0, 0, 0 });
            numY.Minimum = new decimal(new int[] { 500, 0, 0, -2147483648 });
            numY.Size = new System.Drawing.Size(100, 23);
            numY.Value = new decimal(new int[] { 50, 0, 0, 0 });
            numY.Name = "numY";
            numY.TabIndex = 9;
            numZ.DecimalPlaces = 1;
            numZ.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numZ.Location = new System.Drawing.Point(232, 116);
            numZ.Maximum = new decimal(new int[] { 500, 0, 0, 0 });
            numZ.Minimum = new decimal(new int[] { 500, 0, 0, -2147483648 });
            numZ.Size = new System.Drawing.Size(100, 23);
            numZ.Value = new decimal(new int[] { 150, 0, 0, 0 });
            numZ.Name = "numZ";
            numZ.TabIndex = 10;
            lblPitch.AutoSize = true;
            lblPitch.Location = new System.Drawing.Point(8, 150);
            lblPitch.Text = "Takım eğimi ψ (°):";
            lblPitch.Name = "lblPitch";
            numPitchDeg.DecimalPlaces = 1;
            numPitchDeg.Increment = new decimal(new int[] { 5, 0, 0, 0 });
            numPitchDeg.Location = new System.Drawing.Point(160, 146);
            numPitchDeg.Maximum = new decimal(new int[] { 180, 0, 0, 0 });
            numPitchDeg.Minimum = new decimal(new int[] { 180, 0, 0, -2147483648 });
            numPitchDeg.Size = new System.Drawing.Size(80, 23);
            numPitchDeg.Value = new decimal(new int[] { 0, 0, 0, 0 });
            numPitchDeg.Name = "numPitchDeg";
            numPitchDeg.TabIndex = 12;
            chkElbowUp.AutoSize = true;
            chkElbowUp.Location = new System.Drawing.Point(250, 148);
            chkElbowUp.Text = "Dirsek yukarı";
            chkElbowUp.Name = "chkElbowUp";
            chkElbowUp.TabIndex = 13;
            lblStart.AutoSize = true;
            lblStart.Location = new System.Drawing.Point(8, 180);
            lblStart.Text = "Başlama anı t0 (s):";
            lblStart.Name = "lblStart";
            numStartS.DecimalPlaces = 2;
            numStartS.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numStartS.Location = new System.Drawing.Point(160, 176);
            numStartS.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numStartS.Minimum = new decimal(new int[] { 0, 0, 0, 0 });
            numStartS.Size = new System.Drawing.Size(100, 23);
            numStartS.Value = new decimal(new int[] { 0, 0, 0, 0 });
            numStartS.Name = "numStartS";
            numStartS.TabIndex = 15;
            lblDuration.AutoSize = true;
            lblDuration.Location = new System.Drawing.Point(8, 210);
            lblDuration.Text = "Hareket süresi T (s):";
            lblDuration.Name = "lblDuration";
            numDurationS.DecimalPlaces = 2;
            numDurationS.Increment = new decimal(new int[] { 5, 0, 0, 65536 });
            numDurationS.Location = new System.Drawing.Point(160, 206);
            numDurationS.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numDurationS.Minimum = new decimal(new int[] { 5, 0, 0, 131072 });
            numDurationS.Size = new System.Drawing.Size(100, 23);
            numDurationS.Value = new decimal(new int[] { 2, 0, 0, 0 });
            numDurationS.Name = "numDurationS";
            numDurationS.TabIndex = 17;
            circlePickPanel.Location = new System.Drawing.Point(0, 234);
            circlePickPanel.Size = new System.Drawing.Size(350, 250);
            circlePickPanel.Name = "circlePickPanel";
            circlePickPanel.TabIndex = 18;
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            AutoScroll = true;
            Controls.Add(lblTask);
            Controls.Add(cmbTask);
            Controls.Add(lblAngles);
            Controls.Add(numQ1);
            Controls.Add(numQ2);
            Controls.Add(numQ3);
            Controls.Add(numQ4);
            Controls.Add(lblXyz);
            Controls.Add(numX);
            Controls.Add(numY);
            Controls.Add(numZ);
            Controls.Add(lblPitch);
            Controls.Add(numPitchDeg);
            Controls.Add(chkElbowUp);
            Controls.Add(lblStart);
            Controls.Add(numStartS);
            Controls.Add(lblDuration);
            Controls.Add(numDurationS);
            Controls.Add(circlePickPanel);
            Name = "TaskPanel";
            Size = new System.Drawing.Size(350, 490);
            ((System.ComponentModel.ISupportInitialize)numQ1).EndInit();
            ((System.ComponentModel.ISupportInitialize)numQ2).EndInit();
            ((System.ComponentModel.ISupportInitialize)numQ3).EndInit();
            ((System.ComponentModel.ISupportInitialize)numQ4).EndInit();
            ((System.ComponentModel.ISupportInitialize)numX).EndInit();
            ((System.ComponentModel.ISupportInitialize)numY).EndInit();
            ((System.ComponentModel.ISupportInitialize)numZ).EndInit();
            ((System.ComponentModel.ISupportInitialize)numPitchDeg).EndInit();
            ((System.ComponentModel.ISupportInitialize)numStartS).EndInit();
            ((System.ComponentModel.ISupportInitialize)numDurationS).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.Label lblTask;
        private System.Windows.Forms.ComboBox cmbTask;
        private System.Windows.Forms.Label lblAngles;
        private System.Windows.Forms.NumericUpDown numQ1;
        private System.Windows.Forms.NumericUpDown numQ2;
        private System.Windows.Forms.NumericUpDown numQ3;
        private System.Windows.Forms.NumericUpDown numQ4;
        private System.Windows.Forms.Label lblXyz;
        private System.Windows.Forms.NumericUpDown numX;
        private System.Windows.Forms.NumericUpDown numY;
        private System.Windows.Forms.NumericUpDown numZ;
        private System.Windows.Forms.Label lblPitch;
        private System.Windows.Forms.NumericUpDown numPitchDeg;
        private System.Windows.Forms.CheckBox chkElbowUp;
        private System.Windows.Forms.Label lblStart;
        private System.Windows.Forms.NumericUpDown numStartS;
        private System.Windows.Forms.Label lblDuration;
        private System.Windows.Forms.NumericUpDown numDurationS;
        private CirclePickPanel circlePickPanel;
    }
}
