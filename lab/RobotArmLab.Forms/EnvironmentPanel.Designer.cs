namespace RobotArmLab.Forms
{
    partial class EnvironmentPanel
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
            lblEnvironment = new System.Windows.Forms.Label();
            cmbEnvironment = new System.Windows.Forms.ComboBox();
            lblGravity = new System.Windows.Forms.Label();
            numGravity = new System.Windows.Forms.NumericUpDown();
            lblPayload = new System.Windows.Forms.Label();
            numPayload = new System.Windows.Forms.NumericUpDown();
            lblNote = new System.Windows.Forms.Label();
            ((System.ComponentModel.ISupportInitialize)numGravity).BeginInit();
            ((System.ComponentModel.ISupportInitialize)numPayload).BeginInit();
            SuspendLayout();
            //
            // lblEnvironment
            //
            lblEnvironment.AutoSize = true;
            lblEnvironment.Location = new System.Drawing.Point(8, 12);
            lblEnvironment.Name = "lblEnvironment";
            lblEnvironment.Text = "Ortam:";
            //
            // cmbEnvironment
            //
            cmbEnvironment.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            cmbEnvironment.Location = new System.Drawing.Point(120, 8);
            cmbEnvironment.Name = "cmbEnvironment";
            cmbEnvironment.Size = new System.Drawing.Size(220, 23);
            cmbEnvironment.TabIndex = 0;
            cmbEnvironment.SelectedIndexChanged += cmbEnvironment_SelectedIndexChanged;
            //
            // lblGravity
            //
            lblGravity.AutoSize = true;
            lblGravity.Location = new System.Drawing.Point(8, 46);
            lblGravity.Name = "lblGravity";
            lblGravity.Text = "g (m/s²):";
            //
            // numGravity
            //
            numGravity.DecimalPlaces = 2;
            numGravity.Increment = new decimal(new int[] { 1, 0, 0, 65536 });
            numGravity.Location = new System.Drawing.Point(120, 42);
            numGravity.Maximum = new decimal(new int[] { 100, 0, 0, 0 });
            numGravity.Name = "numGravity";
            numGravity.Size = new System.Drawing.Size(100, 23);
            numGravity.TabIndex = 1;
            //
            // lblPayload
            //
            lblPayload.AutoSize = true;
            lblPayload.Location = new System.Drawing.Point(8, 80);
            lblPayload.Name = "lblPayload";
            lblPayload.Text = "Yük kütlesi (kg):";
            //
            // numPayload
            //
            numPayload.DecimalPlaces = 3;
            numPayload.Increment = new decimal(new int[] { 5, 0, 0, 131072 });
            numPayload.Location = new System.Drawing.Point(120, 76);
            numPayload.Maximum = new decimal(new int[] { 5, 0, 0, 65536 });
            numPayload.Name = "numPayload";
            numPayload.Size = new System.Drawing.Size(100, 23);
            numPayload.TabIndex = 2;
            //
            // lblNote
            //
            lblNote.Location = new System.Drawing.Point(8, 114);
            lblNote.Name = "lblNote";
            lblNote.Size = new System.Drawing.Size(332, 60);
            lblNote.Text = "Değişiklikler \"Sıfırla\" (veya bitmiş bir koşudan sonra \"Başlat\") ile uygulanır. Yük takım noktasına bağlı nokta kütledir, en çok 0,5 kg.";
            //
            // EnvironmentPanel
            //
            AutoScaleDimensions = new System.Drawing.SizeF(7F, 15F);
            AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            Controls.Add(lblEnvironment);
            Controls.Add(cmbEnvironment);
            Controls.Add(lblGravity);
            Controls.Add(numGravity);
            Controls.Add(lblPayload);
            Controls.Add(numPayload);
            Controls.Add(lblNote);
            Name = "EnvironmentPanel";
            Size = new System.Drawing.Size(350, 190);
            ((System.ComponentModel.ISupportInitialize)numGravity).EndInit();
            ((System.ComponentModel.ISupportInitialize)numPayload).EndInit();
            ResumeLayout(false);
            PerformLayout();
        }

        #endregion

        private System.Windows.Forms.Label lblEnvironment;
        private System.Windows.Forms.ComboBox cmbEnvironment;
        private System.Windows.Forms.Label lblGravity;
        private System.Windows.Forms.NumericUpDown numGravity;
        private System.Windows.Forms.Label lblPayload;
        private System.Windows.Forms.NumericUpDown numPayload;
        private System.Windows.Forms.Label lblNote;
    }
}
