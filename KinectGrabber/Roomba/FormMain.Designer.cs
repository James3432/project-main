namespace RobotController
{
    partial class FormMain
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
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FormMain));
            this.btn_stop = new System.Windows.Forms.Button();
            this.grp_control = new System.Windows.Forms.GroupBox();
            this.img_ctrl = new System.Windows.Forms.PictureBox();
            this.lbl_control = new System.Windows.Forms.Label();
            this.grp_stat = new System.Windows.Forms.GroupBox();
            this.txt_log = new System.Windows.Forms.ListBox();
            this.btn_disconn = new System.Windows.Forms.Button();
            this.btn_connect = new System.Windows.Forms.Button();
            this.btn_tcpcon = new System.Windows.Forms.Button();
            this.btn_tcpdis = new System.Windows.Forms.Button();
            this.btn_tcptest = new System.Windows.Forms.Button();
            this.groupBox1 = new System.Windows.Forms.GroupBox();
            this.btn_stopscan = new System.Windows.Forms.Button();
            this.btn_scan = new System.Windows.Forms.Button();
            this.img_kin = new System.Windows.Forms.PictureBox();
            this.grp_control.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.img_ctrl)).BeginInit();
            this.grp_stat.SuspendLayout();
            this.groupBox1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.img_kin)).BeginInit();
            this.SuspendLayout();
            // 
            // btn_stop
            // 
            this.btn_stop.BackColor = System.Drawing.SystemColors.Control;
            this.btn_stop.Enabled = false;
            this.btn_stop.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Bold, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_stop.ForeColor = System.Drawing.Color.Red;
            this.btn_stop.Location = new System.Drawing.Point(69, 157);
            this.btn_stop.Name = "btn_stop";
            this.btn_stop.Size = new System.Drawing.Size(90, 23);
            this.btn_stop.TabIndex = 6;
            this.btn_stop.TabStop = false;
            this.btn_stop.Text = "STOP";
            this.btn_stop.UseVisualStyleBackColor = true;
            this.btn_stop.Click += new System.EventHandler(this.btn_stop_Click);
            this.btn_stop.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_stop.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // grp_control
            // 
            this.grp_control.Controls.Add(this.img_ctrl);
            this.grp_control.Controls.Add(this.lbl_control);
            this.grp_control.Controls.Add(this.btn_stop);
            this.grp_control.Location = new System.Drawing.Point(12, 12);
            this.grp_control.Name = "grp_control";
            this.grp_control.Size = new System.Drawing.Size(236, 195);
            this.grp_control.TabIndex = 0;
            this.grp_control.TabStop = false;
            this.grp_control.Text = "Control";
            // 
            // img_ctrl
            // 
            this.img_ctrl.Image = global::RobotController.Properties.Resources.stop;
            this.img_ctrl.InitialImage = global::RobotController.Properties.Resources.stop;
            this.img_ctrl.Location = new System.Drawing.Point(69, 51);
            this.img_ctrl.Name = "img_ctrl";
            this.img_ctrl.Size = new System.Drawing.Size(90, 90);
            this.img_ctrl.TabIndex = 8;
            this.img_ctrl.TabStop = false;
            // 
            // lbl_control
            // 
            this.lbl_control.AutoSize = true;
            this.lbl_control.Location = new System.Drawing.Point(16, 24);
            this.lbl_control.Name = "lbl_control";
            this.lbl_control.Size = new System.Drawing.Size(206, 13);
            this.lbl_control.TabIndex = 7;
            this.lbl_control.Text = "Use the arrow keys to control the Roomba";
            // 
            // grp_stat
            // 
            this.grp_stat.Controls.Add(this.txt_log);
            this.grp_stat.Controls.Add(this.btn_disconn);
            this.grp_stat.Controls.Add(this.btn_connect);
            this.grp_stat.Location = new System.Drawing.Point(12, 213);
            this.grp_stat.Name = "grp_stat";
            this.grp_stat.Size = new System.Drawing.Size(236, 160);
            this.grp_stat.TabIndex = 1;
            this.grp_stat.TabStop = false;
            this.grp_stat.Text = "Status";
            // 
            // txt_log
            // 
            this.txt_log.FormattingEnabled = true;
            this.txt_log.Location = new System.Drawing.Point(6, 19);
            this.txt_log.Name = "txt_log";
            this.txt_log.ScrollAlwaysVisible = true;
            this.txt_log.SelectionMode = System.Windows.Forms.SelectionMode.None;
            this.txt_log.Size = new System.Drawing.Size(224, 95);
            this.txt_log.TabIndex = 3;
            this.txt_log.TabStop = false;
            // 
            // btn_disconn
            // 
            this.btn_disconn.BackColor = System.Drawing.SystemColors.Control;
            this.btn_disconn.ForeColor = System.Drawing.SystemColors.ControlText;
            this.btn_disconn.Location = new System.Drawing.Point(123, 122);
            this.btn_disconn.Name = "btn_disconn";
            this.btn_disconn.Size = new System.Drawing.Size(107, 32);
            this.btn_disconn.TabIndex = 8;
            this.btn_disconn.TabStop = false;
            this.btn_disconn.Text = "Disconnect";
            this.btn_disconn.UseVisualStyleBackColor = true;
            this.btn_disconn.Click += new System.EventHandler(this.btn_disconn_Click);
            this.btn_disconn.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_disconn.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // btn_connect
            // 
            this.btn_connect.BackColor = System.Drawing.SystemColors.Control;
            this.btn_connect.ForeColor = System.Drawing.SystemColors.ControlText;
            this.btn_connect.Location = new System.Drawing.Point(6, 122);
            this.btn_connect.Name = "btn_connect";
            this.btn_connect.Size = new System.Drawing.Size(111, 32);
            this.btn_connect.TabIndex = 7;
            this.btn_connect.TabStop = false;
            this.btn_connect.Text = "Connect";
            this.btn_connect.UseVisualStyleBackColor = true;
            this.btn_connect.Click += new System.EventHandler(this.btn_connect_Click);
            this.btn_connect.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_connect.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // btn_tcpcon
            // 
            this.btn_tcpcon.Location = new System.Drawing.Point(18, 24);
            this.btn_tcpcon.Name = "btn_tcpcon";
            this.btn_tcpcon.Size = new System.Drawing.Size(112, 44);
            this.btn_tcpcon.TabIndex = 2;
            this.btn_tcpcon.Text = "Connect";
            this.btn_tcpcon.UseVisualStyleBackColor = true;
            this.btn_tcpcon.Click += new System.EventHandler(this.btn_tcpcon_Click);
            this.btn_tcpcon.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_tcpcon.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // btn_tcpdis
            // 
            this.btn_tcpdis.Enabled = false;
            this.btn_tcpdis.Location = new System.Drawing.Point(18, 138);
            this.btn_tcpdis.Name = "btn_tcpdis";
            this.btn_tcpdis.Size = new System.Drawing.Size(112, 44);
            this.btn_tcpdis.TabIndex = 3;
            this.btn_tcpdis.Text = "Disconnect";
            this.btn_tcpdis.UseVisualStyleBackColor = true;
            this.btn_tcpdis.Click += new System.EventHandler(this.btn_tcpdis_Click);
            this.btn_tcpdis.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_tcpdis.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // btn_tcptest
            // 
            this.btn_tcptest.Enabled = false;
            this.btn_tcptest.Location = new System.Drawing.Point(18, 79);
            this.btn_tcptest.Name = "btn_tcptest";
            this.btn_tcptest.Size = new System.Drawing.Size(112, 44);
            this.btn_tcptest.TabIndex = 5;
            this.btn_tcptest.Text = "Test";
            this.btn_tcptest.UseVisualStyleBackColor = true;
            this.btn_tcptest.Click += new System.EventHandler(this.btn_tcptest_Click);
            this.btn_tcptest.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_tcptest.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // groupBox1
            // 
            this.groupBox1.Controls.Add(this.btn_stopscan);
            this.groupBox1.Controls.Add(this.btn_scan);
            this.groupBox1.Controls.Add(this.img_kin);
            this.groupBox1.Controls.Add(this.btn_tcpcon);
            this.groupBox1.Controls.Add(this.btn_tcpdis);
            this.groupBox1.Controls.Add(this.btn_tcptest);
            this.groupBox1.Location = new System.Drawing.Point(254, 12);
            this.groupBox1.Name = "groupBox1";
            this.groupBox1.Size = new System.Drawing.Size(147, 361);
            this.groupBox1.TabIndex = 8;
            this.groupBox1.TabStop = false;
            this.groupBox1.Text = "Kinect Grabber";
            // 
            // btn_stopscan
            // 
            this.btn_stopscan.Enabled = false;
            this.btn_stopscan.Font = new System.Drawing.Font("Microsoft Sans Serif", 8.25F, System.Drawing.FontStyle.Regular, System.Drawing.GraphicsUnit.Point, ((byte)(0)));
            this.btn_stopscan.ForeColor = System.Drawing.SystemColors.ControlText;
            this.btn_stopscan.Location = new System.Drawing.Point(40, 323);
            this.btn_stopscan.Name = "btn_stopscan";
            this.btn_stopscan.Size = new System.Drawing.Size(64, 23);
            this.btn_stopscan.TabIndex = 10;
            this.btn_stopscan.Text = "STOP";
            this.btn_stopscan.UseVisualStyleBackColor = true;
            this.btn_stopscan.Click += new System.EventHandler(this.btn_stopscan_Click);
            this.btn_stopscan.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_stopscan.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // btn_scan
            // 
            this.btn_scan.BackgroundImage = ((System.Drawing.Image)(resources.GetObject("btn_scan.BackgroundImage")));
            this.btn_scan.Enabled = false;
            this.btn_scan.Location = new System.Drawing.Point(40, 281);
            this.btn_scan.Name = "btn_scan";
            this.btn_scan.Size = new System.Drawing.Size(64, 34);
            this.btn_scan.TabIndex = 9;
            this.btn_scan.UseVisualStyleBackColor = true;
            this.btn_scan.Click += new System.EventHandler(this.btn_scan_Click);
            this.btn_scan.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.btn_scan.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            // 
            // img_kin
            // 
            this.img_kin.Image = ((System.Drawing.Image)(resources.GetObject("img_kin.Image")));
            this.img_kin.Location = new System.Drawing.Point(40, 201);
            this.img_kin.Name = "img_kin";
            this.img_kin.Size = new System.Drawing.Size(64, 64);
            this.img_kin.TabIndex = 8;
            this.img_kin.TabStop = false;
            // 
            // FormMain
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 13F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(414, 383);
            this.Controls.Add(this.groupBox1);
            this.Controls.Add(this.grp_stat);
            this.Controls.Add(this.grp_control);
            this.FormBorderStyle = System.Windows.Forms.FormBorderStyle.FixedSingle;
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.KeyPreview = true;
            this.MaximizeBox = false;
            this.Name = "FormMain";
            this.StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen;
            this.Text = "Roomba Controller";
            this.FormClosing += new System.Windows.Forms.FormClosingEventHandler(this.FormMain_FormClosing);
            this.KeyDown += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyDown);
            this.KeyUp += new System.Windows.Forms.KeyEventHandler(this.FormMain_KeyUp);
            this.PreviewKeyDown += new System.Windows.Forms.PreviewKeyDownEventHandler(this.FormMain_PreviewKeyDown);
            this.grp_control.ResumeLayout(false);
            this.grp_control.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.img_ctrl)).EndInit();
            this.grp_stat.ResumeLayout(false);
            this.groupBox1.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.img_kin)).EndInit();
            this.ResumeLayout(false);

        }

        #endregion

        private System.Windows.Forms.Button btn_stop;
        private System.Windows.Forms.GroupBox grp_control;
        private System.Windows.Forms.GroupBox grp_stat;
        private System.Windows.Forms.Button btn_disconn;
        private System.Windows.Forms.Button btn_connect;
        public System.Windows.Forms.ListBox txt_log;
        private System.Windows.Forms.Button btn_tcpcon;
        private System.Windows.Forms.Button btn_tcpdis;
        private System.Windows.Forms.Button btn_tcptest;
        private System.Windows.Forms.GroupBox groupBox1;
        private System.Windows.Forms.PictureBox img_kin;
        private System.Windows.Forms.Button btn_scan;
        private System.Windows.Forms.Button btn_stopscan;
        private System.Windows.Forms.Label lbl_control;
        private System.Windows.Forms.PictureBox img_ctrl;
    }
}

