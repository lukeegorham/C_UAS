namespace ConvertRawData
{
    partial class ConvertRawData
    {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
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
            this.InputFileNameBox = new System.Windows.Forms.TextBox();
            this.label1 = new System.Windows.Forms.Label();
            this.StartProcessing = new System.Windows.Forms.Button();
            this.label2 = new System.Windows.Forms.Label();
            this.MessageBox = new System.Windows.Forms.Label();
            this.label3 = new System.Windows.Forms.Label();
            this.OutputFileNameBox = new System.Windows.Forms.TextBox();
            this.progressBar1 = new System.Windows.Forms.ProgressBar();
            this.Clear_button = new System.Windows.Forms.Button();
            this.SuspendLayout();
            // 
            // InputFileNameBox
            // 
            this.InputFileNameBox.Location = new System.Drawing.Point(11, 46);
            this.InputFileNameBox.Margin = new System.Windows.Forms.Padding(2);
            this.InputFileNameBox.Name = "InputFileNameBox";
            this.InputFileNameBox.Size = new System.Drawing.Size(348, 20);
            this.InputFileNameBox.TabIndex = 0;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(8, 23);
            this.label1.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(100, 13);
            this.label1.TabIndex = 1;
            this.label1.Text = "Enter input filename";
            // 
            // StartProcessing
            // 
            this.StartProcessing.Location = new System.Drawing.Point(119, 149);
            this.StartProcessing.Margin = new System.Windows.Forms.Padding(2);
            this.StartProcessing.Name = "StartProcessing";
            this.StartProcessing.Size = new System.Drawing.Size(102, 21);
            this.StartProcessing.TabIndex = 2;
            this.StartProcessing.Text = "Start Processing";
            this.StartProcessing.UseVisualStyleBackColor = true;
            this.StartProcessing.Click += new System.EventHandler(this.StartProcessing_Click);
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(17, 177);
            this.label2.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(55, 13);
            this.label2.TabIndex = 3;
            this.label2.Text = "Messages";
            // 
            // MessageBox
            // 
            this.MessageBox.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            this.MessageBox.Location = new System.Drawing.Point(17, 197);
            this.MessageBox.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.MessageBox.Name = "MessageBox";
            this.MessageBox.Size = new System.Drawing.Size(334, 61);
            this.MessageBox.TabIndex = 4;
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(8, 83);
            this.label3.Margin = new System.Windows.Forms.Padding(2, 0, 2, 0);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(107, 13);
            this.label3.TabIndex = 5;
            this.label3.Text = "Enter output filename";
            // 
            // OutputFileNameBox
            // 
            this.OutputFileNameBox.Location = new System.Drawing.Point(8, 109);
            this.OutputFileNameBox.Margin = new System.Windows.Forms.Padding(2);
            this.OutputFileNameBox.Name = "OutputFileNameBox";
            this.OutputFileNameBox.Size = new System.Drawing.Size(348, 20);
            this.OutputFileNameBox.TabIndex = 6;
            this.OutputFileNameBox.Text = "output.csv";
            // 
            // progressBar1
            // 
            this.progressBar1.Location = new System.Drawing.Point(31, 283);
            this.progressBar1.Margin = new System.Windows.Forms.Padding(2);
            this.progressBar1.Name = "progressBar1";
            this.progressBar1.Size = new System.Drawing.Size(301, 15);
            this.progressBar1.TabIndex = 7;
            // 
            // Clear_button
            // 
            this.Clear_button.Location = new System.Drawing.Point(288, 146);
            this.Clear_button.Name = "Clear_button";
            this.Clear_button.Size = new System.Drawing.Size(75, 23);
            this.Clear_button.TabIndex = 8;
            this.Clear_button.Text = "Clear All";
            this.Clear_button.UseVisualStyleBackColor = true;
            this.Clear_button.Click += new System.EventHandler(this.Clear_button_Click);
            // 
            // ConvertRawData
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(381, 338);
            this.Controls.Add(this.Clear_button);
            this.Controls.Add(this.progressBar1);
            this.Controls.Add(this.OutputFileNameBox);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.MessageBox);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.StartProcessing);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.InputFileNameBox);
            this.Margin = new System.Windows.Forms.Padding(2);
            this.Name = "ConvertRawData";
            this.Text = "Form1";
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.TextBox InputFileNameBox;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.Button StartProcessing;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.Label MessageBox;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.TextBox OutputFileNameBox;
        private System.Windows.Forms.ProgressBar progressBar1;
        private System.Windows.Forms.Button Clear_button;
    }
}

