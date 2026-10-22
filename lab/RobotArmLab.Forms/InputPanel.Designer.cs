namespace RobotArmLab.Forms
{
    partial class InputPanel
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
            rbTorque = new System.Windows.Forms.RadioButton();
            rbVoltage = new System.Windows.Forms.RadioButton();
            gridInputs = new System.Windows.Forms.DataGridView();
            lblHint = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)gridInputs).BeginInit();
            SuspendLayout();
            //
            // rbTorque
            //
            rbTorque.AutoSize = true;
            rbTorque.Checked = true;
            rbTorque.Location = new System.Drawing.Point(8, 8);
            rbTorque.Name = "rbTorque";
            rbTorque.TabIndex = 0;
            rbTorque.TabStop = true;
            rbTorque.Text = "Tork (N·m)";
            rbTorque.CheckedChanged += rbMode_CheckedChanged;
            //
            // rbVoltage
            //
            rbVoltage.AutoSize = true;
            rbVoltage.Location = new System.Drawing.Point(120, 8);
            rbVoltage.Name = "rbVoltage";
            rbVoltage.TabIndex = 1;
            rbVoltage.Text = "Gerilim (V)";
            rbVoltage.CheckedChanged += rbMode_CheckedChanged;
            //
            // gridInputs
            //
            gridInputs.Anchor = System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right;
            gridInputs.ColumnHeadersHeightSizeMode = System.Windows.Forms.DataGridViewColumnHeadersHeightSizeMode.AutoSize;
            gridInputs.Location = new System.Drawing.Point(8, 36);
            gridInputs.Name = "gridInputs";
            gridInputs.Size = new System.Drawing.Size(334, 140);
            gridInputs.TabIndex = 2;
            //
            // lblHint
            //
            lblHint.Anchor = System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Left | System.Windows.Forms.AnchorStyles.Right;
            lblHint.Location = new System.Drawing.Point(8, 180);
            lblHint.Name = "lblHint";
            lblHint.Size = new System.Drawing.Size(334, 46);
            lblHint.Text = "Sabit: genlik hep uygulanır. Basamak: t0'dan sonra. Sinüs: genlik · sin(2π f t). Gerilim ±12 V'a, tork ±τmax'a kırpılır.";
            //
            // InputPanel
            //
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            Controls.Add(rbTorque);
            Controls.Add(rbVoltage);
            Controls.Add(gridInputs);
            Controls.Add(lblHint);
            Name = "InputPanel";
            Size = new System.Drawing.Size(350, 230);
            ((System.ComponentModel.ISupportInitialize)gridInputs).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.RadioButton rbTorque;
        private System.Windows.Forms.RadioButton rbVoltage;
        private System.Windows.Forms.DataGridView gridInputs;
        private System.Windows.Forms.Label lblHint;
    }
}
