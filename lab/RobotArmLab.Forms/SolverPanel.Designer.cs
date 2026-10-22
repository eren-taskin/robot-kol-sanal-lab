namespace RobotArmLab.Forms
{
    partial class SolverPanel
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
            lblSolver = new System.Windows.Forms.Label();
            cmbSolver = new System.Windows.Forms.ComboBox();
            lblStep = new System.Windows.Forms.Label();
            numStepMs = new System.Windows.Forms.NumericUpDown();
            lblRtol = new System.Windows.Forms.Label();
            txtRtol = new System.Windows.Forms.TextBox();
            lblAtol = new System.Windows.Forms.Label();
            txtAtol = new System.Windows.Forms.TextBox();
            lblDuration = new System.Windows.Forms.Label();
            numDuration = new System.Windows.Forms.NumericUpDown();
            ((System.ComponentModel.ISupportInitialize)numStepMs).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numDuration).BeginInit();
            SuspendLayout();
            //
            // lblSolver
            //
            lblSolver.AutoSize = true;
            lblSolver.Location = new System.Drawing.Point(8, 12);
            lblSolver.Name = "lblSolver";
            lblSolver.Text = "Çözücü:";
            //
            // cmbSolver
            //
            cmbSolver.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbSolver.Location = new System.Drawing.Point(120, 8);
            cmbSolver.Name = "cmbSolver";
            cmbSolver.Size = new System.Drawing.Size(180, 23);
            cmbSolver.TabIndex = 0;
            cmbSolver.SelectedIndexChanged += cmbSolver_SelectedIndexChanged;
            //
            // lblStep
            //
            lblStep.AutoSize = true;
            lblStep.Location = new System.Drawing.Point(8, 46);
            lblStep.Name = "lblStep";
            lblStep.Text = "Adım h (ms):";
            //
            // numStepMs
            //
            numStepMs.DecimalPlaces = 3;
            numStepMs.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numStepMs.Location = new System.Drawing.Point(120, 42);
            numStepMs.Maximum = new decimal(new int[] { 50, 0, 0, 0 });
            numStepMs.Minimum = new decimal(new int[] { 1, 0, 0, 196608 });
            numStepMs.Name = "numStepMs";
            numStepMs.Size = new System.Drawing.Size(100, 23);
            numStepMs.TabIndex = 1;
            numStepMs.Value = new decimal(new int[] { 5, 0, 0, 65536 });
            //
            // lblRtol
            //
            lblRtol.AutoSize = true;
            lblRtol.Location = new System.Drawing.Point(8, 80);
            lblRtol.Name = "lblRtol";
            lblRtol.Text = "rtol:";
            //
            // txtRtol
            //
            txtRtol.Location = new System.Drawing.Point(120, 76);
            txtRtol.Name = "txtRtol";
            txtRtol.Size = new System.Drawing.Size(100, 23);
            txtRtol.TabIndex = 2;
            //
            // lblAtol
            //
            lblAtol.AutoSize = true;
            lblAtol.Location = new System.Drawing.Point(8, 114);
            lblAtol.Name = "lblAtol";
            lblAtol.Text = "atol:";
            //
            // txtAtol
            //
            txtAtol.Location = new System.Drawing.Point(120, 110);
            txtAtol.Name = "txtAtol";
            txtAtol.Size = new System.Drawing.Size(100, 23);
            txtAtol.TabIndex = 3;
            //
            // lblDuration
            //
            lblDuration.AutoSize = true;
            lblDuration.Location = new System.Drawing.Point(8, 148);
            lblDuration.Name = "lblDuration";
            lblDuration.Text = "Süre (s):";
            //
            // numDuration
            //
            numDuration.DecimalPlaces = 1;
            numDuration.Location = new System.Drawing.Point(120, 144);
            numDuration.Maximum = new decimal(new int[] { 600, 0, 0, 0 });
            numDuration.Minimum = new decimal(new int[] { 1, 0, 0, 65536 });
            numDuration.Name = "numDuration";
            numDuration.Size = new System.Drawing.Size(100, 23);
            numDuration.TabIndex = 4;
            numDuration.Value = new decimal(new int[] { 10, 0, 0, 0 });
            //
            // SolverPanel
            //
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            Controls.Add(lblSolver);
            Controls.Add(cmbSolver);
            Controls.Add(lblStep);
            Controls.Add(numStepMs);
            Controls.Add(lblRtol);
            Controls.Add(txtRtol);
            Controls.Add(lblAtol);
            Controls.Add(txtAtol);
            Controls.Add(lblDuration);
            Controls.Add(numDuration);
            Name = "SolverPanel";
            Size = new System.Drawing.Size(350, 190);
            ((System.ComponentModel.ISupportInitialize)numStepMs).EndInit();
            ((System.ComponentModel.ISupportInitialize)numDuration).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.Label lblSolver;
        private System.Windows.Forms.ComboBox cmbSolver;
        private System.Windows.Forms.Label lblStep;
        private System.Windows.Forms.NumericUpDown numStepMs;
        private System.Windows.Forms.Label lblRtol;
        private System.Windows.Forms.TextBox txtRtol;
        private System.Windows.Forms.Label lblAtol;
        private System.Windows.Forms.TextBox txtAtol;
        private System.Windows.Forms.Label lblDuration;
        private System.Windows.Forms.NumericUpDown numDuration;
    }
}
