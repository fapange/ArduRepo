#define SLV_ADDED

namespace ArdupilotMega.GCSViews
{
    partial class FlightData
    {
        private System.ComponentModel.IContainer components = null;

        private void InitializeComponent()
        {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(FlightData));
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle1 = new System.Windows.Forms.DataGridViewCellStyle();
            System.Windows.Forms.DataGridViewCellStyle dataGridViewCellStyle2 = new System.Windows.Forms.DataGridViewCellStyle();
            this.contextMenuStrip1 = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.goHereToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.pointCameraHereToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.MainH = new System.Windows.Forms.SplitContainer();
            this.SubMainLeft = new System.Windows.Forms.SplitContainer();
            this.hud1 = new hud.HUD();
            this.contextMenuStrip2 = new System.Windows.Forms.ContextMenuStrip(this.components);
            this.recordHudToAVIToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.stopRecordToolStripMenuItem = new System.Windows.Forms.ToolStripMenuItem();
            this.bindingSource1 = new System.Windows.Forms.BindingSource(this.components);
            this.Tabs = new System.Windows.Forms.TabControl();
            this.tabActions = new System.Windows.Forms.TabPage();
            this.BUT_script = new ArdupilotMega.MyButton();
            this.myButton2 = new ArdupilotMega.MyButton();
            this.BUT_joystick = new ArdupilotMega.MyButton();
            this.BUT_quickmanual = new ArdupilotMega.MyButton();
            this.BUT_quickrtl = new ArdupilotMega.MyButton();
            this.BUT_quickauto = new ArdupilotMega.MyButton();
            this.CMB_setwp = new System.Windows.Forms.ComboBox();
            this.BUT_setwp = new ArdupilotMega.MyButton();
            this.CMB_modes = new System.Windows.Forms.ComboBox();
            this.BUT_setmode = new ArdupilotMega.MyButton();
            this.BUT_clear_track = new ArdupilotMega.MyButton();
            this.CMB_action = new System.Windows.Forms.ComboBox();
            this.BUT_Homealt = new ArdupilotMega.MyButton();
            this.BUT_RAWSensor = new ArdupilotMega.MyButton();
            this.myButton1 = new ArdupilotMega.MyButton();
            this.BUTrestartmission = new ArdupilotMega.MyButton();
            this.BUTactiondo = new ArdupilotMega.MyButton();
            this.tabGauges = new System.Windows.Forms.TabPage();
            this.Gheading = new AGaugeApp.AGauge();
            this.GaccZ = new AGaugeApp.AGauge();
            this.GaccY = new AGaugeApp.AGauge();
            this.GaccX = new AGaugeApp.AGauge();
            this.Gthrottle = new AGaugeApp.AGauge();
            this.Grud = new AGaugeApp.AGauge();
            this.Gaftcurrent = new AGaugeApp.AGauge();
            this.Gfwdcurrent = new AGaugeApp.AGauge();
            this.Grpm = new AGaugeApp.AGauge();
            this.Gspeed = new AGaugeApp.AGauge();
            this.Galt = new AGaugeApp.AGauge();
            this.Gvspeed = new AGaugeApp.AGauge();
            this.tabStatus = new System.Windows.Forms.TabPage();
            this.tabTLogs = new System.Windows.Forms.TabPage();
            this.lbl_logpercent = new ArdupilotMega.MyLabel();
            this.NUM_playbackspeed = new System.Windows.Forms.NumericUpDown();
            this.BUT_log2kml = new ArdupilotMega.MyButton();
            this.tracklog = new System.Windows.Forms.TrackBar();
            this.BUT_playlog = new ArdupilotMega.MyButton();
            this.BUT_loadtelem = new ArdupilotMega.MyButton();
            this.tabCDnR = new System.Windows.Forms.TabPage();
            this.CDnR_Send = new System.Windows.Forms.Button();
            this.CDnR_Time = new System.Windows.Forms.NumericUpDown();
            this.CDnR_Altitude = new System.Windows.Forms.NumericUpDown();
            this.CDnR_Airspeed = new System.Windows.Forms.NumericUpDown();
            this.CDnR_Heading = new System.Windows.Forms.NumericUpDown();
            this.CDnR_CheckTime = new System.Windows.Forms.CheckBox();
            this.CDnR_CheckAltitude = new System.Windows.Forms.CheckBox();
            this.CDnR_CheckAirspeed = new System.Windows.Forms.CheckBox();
            this.CDnR_CheckHeading = new System.Windows.Forms.CheckBox();
            this.tabPage1 = new System.Windows.Forms.TabPage();
            this.simSendMessage = new System.Windows.Forms.Button();
            this.label3 = new System.Windows.Forms.Label();
            this.label2 = new System.Windows.Forms.Label();
            this.trafficTimeToWpt = new System.Windows.Forms.NumericUpDown();
            this.trafficSimTime = new System.Windows.Forms.NumericUpDown();
            this.tableMap = new System.Windows.Forms.TableLayoutPanel();
            this.splitContainer1 = new System.Windows.Forms.SplitContainer();
            this.zg1 = new ZedGraph.ZedGraphControl();
            this.lbl_winddir = new ArdupilotMega.MyLabel();
            this.lbl_windvel = new ArdupilotMega.MyLabel();
            this.gMapControl1 = new ArdupilotMega.myGMAP();
            this.panel1 = new System.Windows.Forms.Panel();
            this.TXT_lat = new ArdupilotMega.MyLabel();
            this.Zoomlevel = new System.Windows.Forms.NumericUpDown();
            this.label1 = new ArdupilotMega.MyLabel();
            this.TXT_long = new ArdupilotMega.MyLabel();
            this.TXT_alt = new ArdupilotMega.MyLabel();
            this.CHK_autopan = new System.Windows.Forms.CheckBox();
            this.CB_tuning = new System.Windows.Forms.CheckBox();
            this.dataGridViewImageColumn1 = new System.Windows.Forms.DataGridViewImageColumn();
            this.dataGridViewImageColumn2 = new System.Windows.Forms.DataGridViewImageColumn();
            this.ZedGraphTimer = new System.Windows.Forms.Timer(this.components);
            this.toolTip1 = new System.Windows.Forms.ToolTip(this.components);
            this.label6 = new ArdupilotMega.MyLabel();
            this.contextMenuStrip1.SuspendLayout();
            this.MainH.Panel1.SuspendLayout();
            this.MainH.Panel2.SuspendLayout();
            this.MainH.SuspendLayout();
            this.SubMainLeft.Panel1.SuspendLayout();
            this.SubMainLeft.Panel2.SuspendLayout();
            this.SubMainLeft.SuspendLayout();
            this.contextMenuStrip2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.bindingSource1)).BeginInit();
            this.Tabs.SuspendLayout();
            this.tabActions.SuspendLayout();
            this.tabGauges.SuspendLayout();
            this.tabTLogs.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUM_playbackspeed)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.tracklog)).BeginInit();
            this.tabCDnR.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Time)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Altitude)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Airspeed)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Heading)).BeginInit();
            this.tabPage1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trafficTimeToWpt)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.trafficSimTime)).BeginInit();
            this.tableMap.SuspendLayout();
            this.splitContainer1.Panel1.SuspendLayout();
            this.splitContainer1.Panel2.SuspendLayout();
            this.splitContainer1.SuspendLayout();
            this.panel1.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.Zoomlevel)).BeginInit();
            this.SuspendLayout();
            // 
            // contextMenuStrip1
            // 
            this.contextMenuStrip1.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.goHereToolStripMenuItem,
            this.pointCameraHereToolStripMenuItem});
            this.contextMenuStrip1.Name = "contextMenuStrip1";
            resources.ApplyResources(this.contextMenuStrip1, "contextMenuStrip1");
            // 
            // goHereToolStripMenuItem
            // 
            this.goHereToolStripMenuItem.Name = "goHereToolStripMenuItem";
            resources.ApplyResources(this.goHereToolStripMenuItem, "goHereToolStripMenuItem");
            this.goHereToolStripMenuItem.Click += new System.EventHandler(this.goHereToolStripMenuItem_Click);
            // 
            // pointCameraHereToolStripMenuItem
            // 
            this.pointCameraHereToolStripMenuItem.Name = "pointCameraHereToolStripMenuItem";
            resources.ApplyResources(this.pointCameraHereToolStripMenuItem, "pointCameraHereToolStripMenuItem");
            this.pointCameraHereToolStripMenuItem.Click += new System.EventHandler(this.pointCameraHereToolStripMenuItem_Click);
            // 
            // MainH
            // 
            this.MainH.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            resources.ApplyResources(this.MainH, "MainH");
            this.MainH.Name = "MainH";
            // 
            // MainH.Panel1
            // 
            this.MainH.Panel1.Controls.Add(this.SubMainLeft);
            // 
            // MainH.Panel2
            // 
            this.MainH.Panel2.Controls.Add(this.tableMap);
            this.MainH.SplitterMoved += new System.Windows.Forms.SplitterEventHandler(this.MainH_SplitterMoved);
            // 
            // SubMainLeft
            // 
            this.SubMainLeft.BorderStyle = System.Windows.Forms.BorderStyle.FixedSingle;
            resources.ApplyResources(this.SubMainLeft, "SubMainLeft");
            this.SubMainLeft.Name = "SubMainLeft";
            // 
            // SubMainLeft.Panel1
            // 
            this.SubMainLeft.Panel1.Controls.Add(this.hud1);
            this.SubMainLeft.Panel1.Resize += new System.EventHandler(this.SubMainHT_Panel1_Resize);
            // 
            // SubMainLeft.Panel2
            // 
            this.SubMainLeft.Panel2.BackColor = System.Drawing.Color.RosyBrown;
            this.SubMainLeft.Panel2.Controls.Add(this.Tabs);
            this.SubMainLeft.Panel2.Paint += new System.Windows.Forms.PaintEventHandler(this.SubMainLeft_Panel2_Paint);
            this.SubMainLeft.SplitterMoved += new System.Windows.Forms.SplitterEventHandler(this.SubMainLeft_SplitterMoved);
            // 
            // hud1
            // 
            this.hud1.airspeed = 0F;
            this.hud1.alt = 0F;
            this.hud1.BackColor = System.Drawing.Color.Transparent;
            this.hud1.batterylevel = 0F;
            this.hud1.batteryremaining = 0F;
            this.hud1.ContextMenuStrip = this.contextMenuStrip2;
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("airspeed", this.bindingSource1, "airspeed", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("alt", this.bindingSource1, "alt", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("batterylevel", this.bindingSource1, "battery_voltage", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("hudLoiter", this.bindingSource1, "stateLoiter", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("hudMode", this.bindingSource1, "stateMode", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("numWpts", this.bindingSource1, "num_Wpts", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("hudVehicle", this.bindingSource1, "stateVehicle", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("batteryremaining", this.bindingSource1, "battery_remaining", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("datetime", this.bindingSource1, "datetime", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("disttowp", this.bindingSource1, "wp_dist", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("hudMUX", this.bindingSource1, "MUX", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("hudRC_state", this.bindingSource1, "RC_state", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("hudOdometer", this.bindingSource1, "odometer", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("wpradius", this.bindingSource1, "wp_radius", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("gpsfix", this.bindingSource1, "gpsstatus", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("gpshdop", this.bindingSource1, "gpshdop", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("groundcourse", this.bindingSource1, "groundcourse", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("groundspeed", this.bindingSource1, "groundspeed", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("heading", this.bindingSource1, "yaw", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("linkqualitygcs", this.bindingSource1, "linkqualitygcs", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("mode", this.bindingSource1, "mode", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("navpitch", this.bindingSource1, "nav_pitch", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("navroll", this.bindingSource1, "nav_roll", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("pitch", this.bindingSource1, "pitch", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("roll", this.bindingSource1, "roll", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("status", this.bindingSource1, "armed", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("targetalt", this.bindingSource1, "targetalt", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("targetheading", this.bindingSource1, "nav_bearing", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("targetspeed", this.bindingSource1, "targetairspeed", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("turnrate", this.bindingSource1, "turnrate", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("verticalspeed", this.bindingSource1, "verticalspeed", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("wpno", this.bindingSource1, "wpno", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("xtrack_error", this.bindingSource1, "xtrack_error", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("selectedAircraft", this.bindingSource1, "selected_Aircraft", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("time_left", this.bindingSource1, "time_left", true));
            this.hud1.DataBindings.Add(new System.Windows.Forms.Binding("sensorcalStatus", this.bindingSource1, "sensor_cal_status", true));
            this.hud1.datetime = new System.DateTime(((long)(0)));
            this.hud1.disttowp = 0F;
            resources.ApplyResources(this.hud1, "hud1");
            this.hud1.gpsfix = 0F;
            this.hud1.gpshdop = 0F;
            this.hud1.groundcourse = 0F;
            this.hud1.groundspeed = 0F;
            this.hud1.heading = 0F;
            this.hud1.hudcolor = System.Drawing.Color.White;
            this.hud1.hudLoiter = ((byte)(0));
            this.hud1.hudMode = ((byte)(0));
            this.hud1.hudMUX = 0F;
            this.hud1.hudOdometer = 0F;
            this.hud1.hudRC_state = ((byte)(0));
            this.hud1.hudVehicle = ((byte)(0));
            this.hud1.linkqualitygcs = 0F;
            this.hud1.mode = "Manual";
            this.hud1.Name = "hud1";
            this.hud1.navpitch = 0F;
            this.hud1.navroll = 0F;
            this.hud1.numWpts = ((byte)(0));
            this.hud1.opengl = true;
            this.hud1.pitch = 0F;
            this.hud1.roll = 0F;
            this.hud1.selectedAircraft = 0;
            this.hud1.sensorcalStatus = 0F;
            this.hud1.status = 0;
            this.hud1.streamjpg = null;
            this.hud1.targetalt = 0F;
            this.hud1.targetheading = 0F;
            this.hud1.targetspeed = 0F;
            this.hud1.time_left = 0;
            this.hud1.turnrate = 0F;
            this.hud1.verticalspeed = 0F;
            this.hud1.VSync = false;
            this.hud1.wpno = 0;
            this.hud1.wpradius = 0F;
            this.hud1.xtrack_error = 0F;
            this.hud1.DoubleClick += new System.EventHandler(this.hud1_DoubleClick);
            // 
            // contextMenuStrip2
            // 
            this.contextMenuStrip2.Items.AddRange(new System.Windows.Forms.ToolStripItem[] {
            this.recordHudToAVIToolStripMenuItem,
            this.stopRecordToolStripMenuItem});
            this.contextMenuStrip2.Name = "contextMenuStrip2";
            resources.ApplyResources(this.contextMenuStrip2, "contextMenuStrip2");
            // 
            // recordHudToAVIToolStripMenuItem
            // 
            this.recordHudToAVIToolStripMenuItem.Name = "recordHudToAVIToolStripMenuItem";
            resources.ApplyResources(this.recordHudToAVIToolStripMenuItem, "recordHudToAVIToolStripMenuItem");
            this.recordHudToAVIToolStripMenuItem.Click += new System.EventHandler(this.recordHudToAVIToolStripMenuItem_Click);
            // 
            // stopRecordToolStripMenuItem
            // 
            this.stopRecordToolStripMenuItem.Name = "stopRecordToolStripMenuItem";
            resources.ApplyResources(this.stopRecordToolStripMenuItem, "stopRecordToolStripMenuItem");
            this.stopRecordToolStripMenuItem.Click += new System.EventHandler(this.stopRecordToolStripMenuItem_Click);
            // 
            // bindingSource1
            // 
            this.bindingSource1.DataSource = typeof(ArdupilotMega.CurrentState);
            // 
            // Tabs
            // 
            this.Tabs.Controls.Add(this.tabActions);
            this.Tabs.Controls.Add(this.tabGauges);
            this.Tabs.Controls.Add(this.tabStatus);
            this.Tabs.Controls.Add(this.tabTLogs);
            this.Tabs.Controls.Add(this.tabCDnR);
            this.Tabs.Controls.Add(this.tabPage1);
            resources.ApplyResources(this.Tabs, "Tabs");
            this.Tabs.Name = "Tabs";
            this.Tabs.SelectedIndex = 0;
            this.Tabs.Tag = "";
            this.Tabs.DrawItem += new System.Windows.Forms.DrawItemEventHandler(this.tabControl1_DrawItem);
            this.Tabs.SelectedIndexChanged += new System.EventHandler(this.tabControl1_SelectedIndexChanged);
            // 
            // tabActions
            // 
            this.tabActions.Controls.Add(this.BUT_script);
            this.tabActions.Controls.Add(this.myButton2);
            this.tabActions.Controls.Add(this.BUT_joystick);
            this.tabActions.Controls.Add(this.BUT_quickmanual);
            this.tabActions.Controls.Add(this.BUT_quickrtl);
            this.tabActions.Controls.Add(this.BUT_quickauto);
            this.tabActions.Controls.Add(this.CMB_setwp);
            this.tabActions.Controls.Add(this.BUT_setwp);
            this.tabActions.Controls.Add(this.CMB_modes);
            this.tabActions.Controls.Add(this.BUT_setmode);
            this.tabActions.Controls.Add(this.BUT_clear_track);
            this.tabActions.Controls.Add(this.CMB_action);
            this.tabActions.Controls.Add(this.BUT_Homealt);
            this.tabActions.Controls.Add(this.BUT_RAWSensor);
            this.tabActions.Controls.Add(this.myButton1);
            this.tabActions.Controls.Add(this.BUTrestartmission);
            this.tabActions.Controls.Add(this.BUTactiondo);
            resources.ApplyResources(this.tabActions, "tabActions");
            this.tabActions.Name = "tabActions";
            this.tabActions.UseVisualStyleBackColor = true;
            // 
            // BUT_script
            // 
            resources.ApplyResources(this.BUT_script, "BUT_script");
            this.BUT_script.Name = "BUT_script";
            this.BUT_script.UseVisualStyleBackColor = true;
            this.BUT_script.Click += new System.EventHandler(this.BUT_script_Click);
            // 
            // myButton2
            // 
            this.myButton2.BackColor = System.Drawing.Color.Orange;
            resources.ApplyResources(this.myButton2, "myButton2");
            this.myButton2.Name = "myButton2";
            this.toolTip1.SetToolTip(this.myButton2, resources.GetString("myButton2.ToolTip"));
            this.myButton2.UseVisualStyleBackColor = false;
            this.myButton2.Click += new System.EventHandler(this.BUTFailSafe_Emergency_Click);
            // 
            // BUT_joystick
            // 
            resources.ApplyResources(this.BUT_joystick, "BUT_joystick");
            this.BUT_joystick.Name = "BUT_joystick";
            this.toolTip1.SetToolTip(this.BUT_joystick, resources.GetString("BUT_joystick.ToolTip"));
            this.BUT_joystick.UseVisualStyleBackColor = true;
            this.BUT_joystick.Click += new System.EventHandler(this.BUT_joystick_Click);
            // 
            // BUT_quickmanual
            // 
            resources.ApplyResources(this.BUT_quickmanual, "BUT_quickmanual");
            this.BUT_quickmanual.Name = "BUT_quickmanual";
            this.toolTip1.SetToolTip(this.BUT_quickmanual, resources.GetString("BUT_quickmanual.ToolTip"));
            this.BUT_quickmanual.UseVisualStyleBackColor = true;
            this.BUT_quickmanual.Click += new System.EventHandler(this.BUT_quickmanual_Click);
            // 
            // BUT_quickrtl
            // 
            resources.ApplyResources(this.BUT_quickrtl, "BUT_quickrtl");
            this.BUT_quickrtl.Name = "BUT_quickrtl";
            this.toolTip1.SetToolTip(this.BUT_quickrtl, resources.GetString("BUT_quickrtl.ToolTip"));
            this.BUT_quickrtl.UseVisualStyleBackColor = true;
            this.BUT_quickrtl.Click += new System.EventHandler(this.BUT_quickrtl_Click);
            // 
            // BUT_quickauto
            // 
            resources.ApplyResources(this.BUT_quickauto, "BUT_quickauto");
            this.BUT_quickauto.Name = "BUT_quickauto";
            this.toolTip1.SetToolTip(this.BUT_quickauto, resources.GetString("BUT_quickauto.ToolTip"));
            this.BUT_quickauto.UseVisualStyleBackColor = true;
            this.BUT_quickauto.Click += new System.EventHandler(this.BUT_quickauto_Click);
            // 
            // CMB_setwp
            // 
            this.CMB_setwp.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_setwp.FormattingEnabled = true;
            this.CMB_setwp.Items.AddRange(new object[] {
            resources.GetString("CMB_setwp.Items")});
            resources.ApplyResources(this.CMB_setwp, "CMB_setwp");
            this.CMB_setwp.Name = "CMB_setwp";
            this.CMB_setwp.Click += new System.EventHandler(this.CMB_setwp_Click);
            // 
            // BUT_setwp
            // 
            resources.ApplyResources(this.BUT_setwp, "BUT_setwp");
            this.BUT_setwp.Name = "BUT_setwp";
            this.toolTip1.SetToolTip(this.BUT_setwp, resources.GetString("BUT_setwp.ToolTip"));
            this.BUT_setwp.UseVisualStyleBackColor = true;
            this.BUT_setwp.Click += new System.EventHandler(this.BUT_setwp_Click);
            // 
            // CMB_modes
            // 
            this.CMB_modes.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_modes.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_modes, "CMB_modes");
            this.CMB_modes.Name = "CMB_modes";
            this.CMB_modes.Click += new System.EventHandler(this.CMB_modes_Click);
            // 
            // BUT_setmode
            // 
            resources.ApplyResources(this.BUT_setmode, "BUT_setmode");
            this.BUT_setmode.Name = "BUT_setmode";
            this.toolTip1.SetToolTip(this.BUT_setmode, resources.GetString("BUT_setmode.ToolTip"));
            this.BUT_setmode.UseVisualStyleBackColor = true;
            this.BUT_setmode.Click += new System.EventHandler(this.BUT_setmode_Click);
            // 
            // BUT_clear_track
            // 
            resources.ApplyResources(this.BUT_clear_track, "BUT_clear_track");
            this.BUT_clear_track.Name = "BUT_clear_track";
            this.toolTip1.SetToolTip(this.BUT_clear_track, resources.GetString("BUT_clear_track.ToolTip"));
            this.BUT_clear_track.UseVisualStyleBackColor = true;
            this.BUT_clear_track.Click += new System.EventHandler(this.BUT_clear_track_Click);
            // 
            // CMB_action
            // 
            this.CMB_action.DropDownStyle = System.Windows.Forms.ComboBoxStyle.DropDownList;
            this.CMB_action.DropDownWidth = 110;
            this.CMB_action.FormattingEnabled = true;
            resources.ApplyResources(this.CMB_action, "CMB_action");
            this.CMB_action.Name = "CMB_action";
            // 
            // BUT_Homealt
            // 
            resources.ApplyResources(this.BUT_Homealt, "BUT_Homealt");
            this.BUT_Homealt.Name = "BUT_Homealt";
            this.toolTip1.SetToolTip(this.BUT_Homealt, resources.GetString("BUT_Homealt.ToolTip"));
            this.BUT_Homealt.UseVisualStyleBackColor = true;
            // 
            // BUT_RAWSensor
            // 
            resources.ApplyResources(this.BUT_RAWSensor, "BUT_RAWSensor");
            this.BUT_RAWSensor.Name = "BUT_RAWSensor";
            this.toolTip1.SetToolTip(this.BUT_RAWSensor, resources.GetString("BUT_RAWSensor.ToolTip"));
            this.BUT_RAWSensor.UseVisualStyleBackColor = true;
            this.BUT_RAWSensor.Click += new System.EventHandler(this.BUT_RAWSensor_Click);
            // 
            // myButton1
            // 
            resources.ApplyResources(this.myButton1, "myButton1");
            this.myButton1.Name = "myButton1";
            this.toolTip1.SetToolTip(this.myButton1, resources.GetString("myButton1.ToolTip"));
            this.myButton1.UseVisualStyleBackColor = true;
            this.myButton1.Click += new System.EventHandler(this.BUTzero_acc_hold_Click);
            // 
            // BUTrestartmission
            // 
            resources.ApplyResources(this.BUTrestartmission, "BUTrestartmission");
            this.BUTrestartmission.Name = "BUTrestartmission";
            this.toolTip1.SetToolTip(this.BUTrestartmission, resources.GetString("BUTrestartmission.ToolTip"));
            this.BUTrestartmission.UseVisualStyleBackColor = true;
            this.BUTrestartmission.Click += new System.EventHandler(this.BUTrestartmission_Click);
            // 
            // BUTactiondo
            // 
            resources.ApplyResources(this.BUTactiondo, "BUTactiondo");
            this.BUTactiondo.Name = "BUTactiondo";
            this.toolTip1.SetToolTip(this.BUTactiondo, resources.GetString("BUTactiondo.ToolTip"));
            this.BUTactiondo.UseVisualStyleBackColor = true;
            this.BUTactiondo.Click += new System.EventHandler(this.BUTactiondo_Click);
            // 
            // tabGauges
            // 
            this.tabGauges.Controls.Add(this.Gheading);
            this.tabGauges.Controls.Add(this.GaccZ);
            this.tabGauges.Controls.Add(this.GaccY);
            this.tabGauges.Controls.Add(this.GaccX);
            this.tabGauges.Controls.Add(this.Gthrottle);
            this.tabGauges.Controls.Add(this.Grud);
            this.tabGauges.Controls.Add(this.Gaftcurrent);
            this.tabGauges.Controls.Add(this.Gfwdcurrent);
            this.tabGauges.Controls.Add(this.Grpm);
            this.tabGauges.Controls.Add(this.Gspeed);
            this.tabGauges.Controls.Add(this.Galt);
            this.tabGauges.Controls.Add(this.Gvspeed);
            resources.ApplyResources(this.tabGauges, "tabGauges");
            this.tabGauges.Name = "tabGauges";
            this.tabGauges.UseVisualStyleBackColor = true;
            this.tabGauges.Resize += new System.EventHandler(this.tabPage1_Resize);
            // 
            // Gheading
            // 
            this.Gheading.BackColor = System.Drawing.Color.Transparent;
            this.Gheading.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Gheading, "Gheading");
            this.Gheading.BaseArcColor = System.Drawing.Color.Transparent;
            this.Gheading.BaseArcRadius = 60;
            this.Gheading.BaseArcStart = 270;
            this.Gheading.BaseArcSweep = 360;
            this.Gheading.BaseArcWidth = 2;
            this.Gheading.basesize = new System.Drawing.Size(150, 150);
            this.Gheading.Cap_Idx = ((byte)(0));
            this.Gheading.CapColor = System.Drawing.Color.White;
            this.Gheading.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Gheading.CapPosition = new System.Drawing.Point(46, 80);
            this.Gheading.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(46, 80),
        new System.Drawing.Point(40, 67),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Gheading.CapsText = new string[] {
        "Heading",
        "",
        "",
        "",
        ""};
            this.Gheading.CapText = "Heading";
            this.Gheading.Center = new System.Drawing.Point(75, 75);
            this.Gheading.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "wind_dir", true));
            this.Gheading.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "target_bearing", true));
            this.Gheading.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "nav_bearing", true));
            this.Gheading.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "groundcourse", true));
            this.Gheading.DataBindings.Add(new System.Windows.Forms.Binding("Value4", this.bindingSource1, "yaw", true));
            this.Gheading.MaxValue = 359F;
            this.Gheading.MinValue = 0F;
            this.Gheading.Name = "Gheading";
            this.Gheading.Need_Idx = ((byte)(4));
            this.Gheading.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Red;
            this.Gheading.NeedleColor2 = System.Drawing.Color.White;
            this.Gheading.NeedleEnabled = true;
            this.Gheading.NeedleRadius = 50;
            this.Gheading.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Yellow,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Blue,
        AGaugeApp.AGauge.NeedleColorEnum.Red,
        AGaugeApp.AGauge.NeedleColorEnum.Red};
            this.Gheading.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.Gold,
        System.Drawing.Color.White,
        System.Drawing.Color.Green,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White};
            this.Gheading.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        true};
            this.Gheading.NeedlesRadius = new int[] {
        56,
        50,
        56,
        50,
        50};
            this.Gheading.NeedlesType = new int[] {
        2,
        0,
        2,
        0,
        0};
            this.Gheading.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Gheading.NeedleType = 0;
            this.Gheading.NeedleWidth = 2;
            this.Gheading.Range_Idx = ((byte)(0));
            this.Gheading.RangeColor = System.Drawing.Color.LightGreen;
            this.Gheading.RangeEnabled = false;
            this.Gheading.RangeEndValue = 360F;
            this.Gheading.RangeInnerRadius = 1;
            this.Gheading.RangeOuterRadius = 60;
            this.Gheading.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Gheading.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Gheading.RangesEndValue = new float[] {
        360F,
        200F,
        150F,
        0F,
        0F};
            this.Gheading.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Gheading.RangesOuterRadius = new int[] {
        60,
        60,
        60,
        80,
        80};
            this.Gheading.RangesStartValue = new float[] {
        0F,
        150F,
        75F,
        0F,
        0F};
            this.Gheading.RangeStartValue = 0F;
            this.Gheading.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Gheading.ScaleLinesInterInnerRadius = 52;
            this.Gheading.ScaleLinesInterOuterRadius = 60;
            this.Gheading.ScaleLinesInterWidth = 1;
            this.Gheading.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Gheading.ScaleLinesMajorInnerRadius = 50;
            this.Gheading.ScaleLinesMajorOuterRadius = 60;
            this.Gheading.ScaleLinesMajorStepValue = 45F;
            this.Gheading.ScaleLinesMajorWidth = 2;
            this.Gheading.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Gheading.ScaleLinesMinorInnerRadius = 55;
            this.Gheading.ScaleLinesMinorNumOf = 9;
            this.Gheading.ScaleLinesMinorOuterRadius = 60;
            this.Gheading.ScaleLinesMinorWidth = 1;
            this.Gheading.ScaleNumbersColor = System.Drawing.Color.White;
            this.Gheading.ScaleNumbersFormat = null;
            this.Gheading.ScaleNumbersRadius = 42;
            this.Gheading.ScaleNumbersRotation = 45;
            this.Gheading.ScaleNumbersStartScaleLine = 1;
            this.Gheading.ScaleNumbersStepScaleLines = 1;
            this.Gheading.Value = 0F;
            this.Gheading.Value0 = 0F;
            this.Gheading.Value1 = 0F;
            this.Gheading.Value2 = 0F;
            this.Gheading.Value3 = 0F;
            this.Gheading.Value4 = 0F;
            // 
            // GaccZ
            // 
            this.GaccZ.BackColor = System.Drawing.Color.Transparent;
            this.GaccZ.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.GaccZ, "GaccZ");
            this.GaccZ.BaseArcColor = System.Drawing.Color.Transparent;
            this.GaccZ.BaseArcRadius = 70;
            this.GaccZ.BaseArcStart = 135;
            this.GaccZ.BaseArcSweep = 270;
            this.GaccZ.BaseArcWidth = 2;
            this.GaccZ.basesize = new System.Drawing.Size(150, 150);
            this.GaccZ.Cap_Idx = ((byte)(0));
            this.GaccZ.CapColor = System.Drawing.Color.White;
            this.GaccZ.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.GaccZ.CapPosition = new System.Drawing.Point(50, 80);
            this.GaccZ.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.GaccZ.CapsText = new string[] {
        "Z acc",
        "",
        "",
        "",
        ""};
            this.GaccZ.CapText = "Z acc";
            this.GaccZ.Center = new System.Drawing.Point(75, 75);
            this.GaccZ.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "az_g", true));
            this.GaccZ.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "az_g", true));
            this.GaccZ.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "azmin_g", true));
            this.GaccZ.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "azmax_g", true));
            this.GaccZ.MaxValue = 5F;
            this.GaccZ.MinValue = -5F;
            this.GaccZ.Name = "GaccZ";
            this.GaccZ.Need_Idx = ((byte)(4));
            this.GaccZ.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.GaccZ.NeedleColor2 = System.Drawing.Color.Brown;
            this.GaccZ.NeedleEnabled = false;
            this.GaccZ.NeedleRadius = 50;
            this.GaccZ.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.GaccZ.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Brown};
            this.GaccZ.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        false};
            this.GaccZ.NeedlesRadius = new int[] {
        50,
        50,
        56,
        56,
        50};
            this.GaccZ.NeedlesType = new int[] {
        0,
        0,
        2,
        2,
        0};
            this.GaccZ.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.GaccZ.NeedleType = 0;
            this.GaccZ.NeedleWidth = 2;
            this.GaccZ.Range_Idx = ((byte)(2));
            this.GaccZ.RangeColor = System.Drawing.Color.Orange;
            this.GaccZ.RangeEnabled = false;
            this.GaccZ.RangeEndValue = 50F;
            this.GaccZ.RangeInnerRadius = 1;
            this.GaccZ.RangeOuterRadius = 70;
            this.GaccZ.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.GaccZ.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.GaccZ.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.GaccZ.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.GaccZ.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.GaccZ.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.GaccZ.RangeStartValue = 35F;
            this.GaccZ.ScaleLinesInterColor = System.Drawing.Color.White;
            this.GaccZ.ScaleLinesInterInnerRadius = 52;
            this.GaccZ.ScaleLinesInterOuterRadius = 60;
            this.GaccZ.ScaleLinesInterWidth = 1;
            this.GaccZ.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.GaccZ.ScaleLinesMajorInnerRadius = 50;
            this.GaccZ.ScaleLinesMajorOuterRadius = 60;
            this.GaccZ.ScaleLinesMajorStepValue = 1F;
            this.GaccZ.ScaleLinesMajorWidth = 2;
            this.GaccZ.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.GaccZ.ScaleLinesMinorInnerRadius = 55;
            this.GaccZ.ScaleLinesMinorNumOf = 9;
            this.GaccZ.ScaleLinesMinorOuterRadius = 60;
            this.GaccZ.ScaleLinesMinorWidth = 1;
            this.GaccZ.ScaleNumbersColor = System.Drawing.Color.White;
            this.GaccZ.ScaleNumbersFormat = null;
            this.GaccZ.ScaleNumbersRadius = 42;
            this.GaccZ.ScaleNumbersRotation = 0;
            this.GaccZ.ScaleNumbersStartScaleLine = 1;
            this.GaccZ.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.GaccZ, resources.GetString("GaccZ.ToolTip"));
            this.GaccZ.Value = -1F;
            this.GaccZ.Value0 = -1F;
            this.GaccZ.Value1 = -1F;
            this.GaccZ.Value2 = -1F;
            this.GaccZ.Value3 = -1F;
            this.GaccZ.Value4 = -1F;
            // 
            // GaccY
            // 
            this.GaccY.BackColor = System.Drawing.Color.Transparent;
            this.GaccY.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.GaccY, "GaccY");
            this.GaccY.BaseArcColor = System.Drawing.Color.Transparent;
            this.GaccY.BaseArcRadius = 70;
            this.GaccY.BaseArcStart = 135;
            this.GaccY.BaseArcSweep = 270;
            this.GaccY.BaseArcWidth = 2;
            this.GaccY.basesize = new System.Drawing.Size(150, 150);
            this.GaccY.Cap_Idx = ((byte)(0));
            this.GaccY.CapColor = System.Drawing.Color.White;
            this.GaccY.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.GaccY.CapPosition = new System.Drawing.Point(50, 80);
            this.GaccY.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.GaccY.CapsText = new string[] {
        "Y acc",
        "",
        "",
        "",
        ""};
            this.GaccY.CapText = "Y acc";
            this.GaccY.Center = new System.Drawing.Point(75, 75);
            this.GaccY.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "ay_g", true));
            this.GaccY.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "ay_g", true));
            this.GaccY.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "aymin_g", true));
            this.GaccY.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "aymax_g", true));
            this.GaccY.MaxValue = 5F;
            this.GaccY.MinValue = -5F;
            this.GaccY.Name = "GaccY";
            this.GaccY.Need_Idx = ((byte)(4));
            this.GaccY.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.GaccY.NeedleColor2 = System.Drawing.Color.Brown;
            this.GaccY.NeedleEnabled = false;
            this.GaccY.NeedleRadius = 50;
            this.GaccY.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.GaccY.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Brown};
            this.GaccY.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        false};
            this.GaccY.NeedlesRadius = new int[] {
        50,
        50,
        56,
        56,
        50};
            this.GaccY.NeedlesType = new int[] {
        0,
        0,
        2,
        2,
        0};
            this.GaccY.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.GaccY.NeedleType = 0;
            this.GaccY.NeedleWidth = 2;
            this.GaccY.Range_Idx = ((byte)(2));
            this.GaccY.RangeColor = System.Drawing.Color.Orange;
            this.GaccY.RangeEnabled = false;
            this.GaccY.RangeEndValue = 50F;
            this.GaccY.RangeInnerRadius = 1;
            this.GaccY.RangeOuterRadius = 70;
            this.GaccY.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.GaccY.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.GaccY.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.GaccY.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.GaccY.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.GaccY.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.GaccY.RangeStartValue = 35F;
            this.GaccY.ScaleLinesInterColor = System.Drawing.Color.White;
            this.GaccY.ScaleLinesInterInnerRadius = 52;
            this.GaccY.ScaleLinesInterOuterRadius = 60;
            this.GaccY.ScaleLinesInterWidth = 1;
            this.GaccY.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.GaccY.ScaleLinesMajorInnerRadius = 50;
            this.GaccY.ScaleLinesMajorOuterRadius = 60;
            this.GaccY.ScaleLinesMajorStepValue = 1F;
            this.GaccY.ScaleLinesMajorWidth = 2;
            this.GaccY.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.GaccY.ScaleLinesMinorInnerRadius = 55;
            this.GaccY.ScaleLinesMinorNumOf = 9;
            this.GaccY.ScaleLinesMinorOuterRadius = 60;
            this.GaccY.ScaleLinesMinorWidth = 1;
            this.GaccY.ScaleNumbersColor = System.Drawing.Color.White;
            this.GaccY.ScaleNumbersFormat = null;
            this.GaccY.ScaleNumbersRadius = 42;
            this.GaccY.ScaleNumbersRotation = 0;
            this.GaccY.ScaleNumbersStartScaleLine = 1;
            this.GaccY.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.GaccY, resources.GetString("GaccY.ToolTip"));
            this.GaccY.Value = 0F;
            this.GaccY.Value0 = 0F;
            this.GaccY.Value1 = 0F;
            this.GaccY.Value2 = 0F;
            this.GaccY.Value3 = 0F;
            this.GaccY.Value4 = 0F;
            // 
            // GaccX
            // 
            this.GaccX.BackColor = System.Drawing.Color.Transparent;
            this.GaccX.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.GaccX, "GaccX");
            this.GaccX.BaseArcColor = System.Drawing.Color.Transparent;
            this.GaccX.BaseArcRadius = 70;
            this.GaccX.BaseArcStart = 135;
            this.GaccX.BaseArcSweep = 270;
            this.GaccX.BaseArcWidth = 2;
            this.GaccX.basesize = new System.Drawing.Size(150, 150);
            this.GaccX.Cap_Idx = ((byte)(0));
            this.GaccX.CapColor = System.Drawing.Color.White;
            this.GaccX.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.GaccX.CapPosition = new System.Drawing.Point(50, 80);
            this.GaccX.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.GaccX.CapsText = new string[] {
        "X acc",
        "",
        "",
        "",
        ""};
            this.GaccX.CapText = "X acc";
            this.GaccX.Center = new System.Drawing.Point(75, 75);
            this.GaccX.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "ax_g", true));
            this.GaccX.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "ax_g", true));
            this.GaccX.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "axmin_g", true));
            this.GaccX.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "axmax_g", true));
            this.GaccX.MaxValue = 5F;
            this.GaccX.MinValue = -5F;
            this.GaccX.Name = "GaccX";
            this.GaccX.Need_Idx = ((byte)(4));
            this.GaccX.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.GaccX.NeedleColor2 = System.Drawing.Color.Brown;
            this.GaccX.NeedleEnabled = false;
            this.GaccX.NeedleRadius = 50;
            this.GaccX.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.GaccX.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Brown};
            this.GaccX.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        false};
            this.GaccX.NeedlesRadius = new int[] {
        50,
        50,
        56,
        56,
        50};
            this.GaccX.NeedlesType = new int[] {
        0,
        0,
        2,
        2,
        0};
            this.GaccX.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.GaccX.NeedleType = 0;
            this.GaccX.NeedleWidth = 2;
            this.GaccX.Range_Idx = ((byte)(2));
            this.GaccX.RangeColor = System.Drawing.Color.Orange;
            this.GaccX.RangeEnabled = false;
            this.GaccX.RangeEndValue = 50F;
            this.GaccX.RangeInnerRadius = 1;
            this.GaccX.RangeOuterRadius = 70;
            this.GaccX.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.GaccX.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.GaccX.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.GaccX.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.GaccX.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.GaccX.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.GaccX.RangeStartValue = 35F;
            this.GaccX.ScaleLinesInterColor = System.Drawing.Color.White;
            this.GaccX.ScaleLinesInterInnerRadius = 52;
            this.GaccX.ScaleLinesInterOuterRadius = 60;
            this.GaccX.ScaleLinesInterWidth = 1;
            this.GaccX.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.GaccX.ScaleLinesMajorInnerRadius = 50;
            this.GaccX.ScaleLinesMajorOuterRadius = 60;
            this.GaccX.ScaleLinesMajorStepValue = 1F;
            this.GaccX.ScaleLinesMajorWidth = 2;
            this.GaccX.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.GaccX.ScaleLinesMinorInnerRadius = 55;
            this.GaccX.ScaleLinesMinorNumOf = 9;
            this.GaccX.ScaleLinesMinorOuterRadius = 60;
            this.GaccX.ScaleLinesMinorWidth = 1;
            this.GaccX.ScaleNumbersColor = System.Drawing.Color.White;
            this.GaccX.ScaleNumbersFormat = null;
            this.GaccX.ScaleNumbersRadius = 42;
            this.GaccX.ScaleNumbersRotation = 0;
            this.GaccX.ScaleNumbersStartScaleLine = 1;
            this.GaccX.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.GaccX, resources.GetString("GaccX.ToolTip"));
            this.GaccX.Value = 0F;
            this.GaccX.Value0 = 0F;
            this.GaccX.Value1 = 0F;
            this.GaccX.Value2 = 0F;
            this.GaccX.Value3 = 0F;
            this.GaccX.Value4 = 0F;
            // 
            // Gthrottle
            // 
            this.Gthrottle.BackColor = System.Drawing.Color.Transparent;
            this.Gthrottle.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Gthrottle, "Gthrottle");
            this.Gthrottle.BaseArcColor = System.Drawing.Color.Transparent;
            this.Gthrottle.BaseArcRadius = 70;
            this.Gthrottle.BaseArcStart = 135;
            this.Gthrottle.BaseArcSweep = 270;
            this.Gthrottle.BaseArcWidth = 2;
            this.Gthrottle.basesize = new System.Drawing.Size(150, 150);
            this.Gthrottle.Cap_Idx = ((byte)(0));
            this.Gthrottle.CapColor = System.Drawing.Color.White;
            this.Gthrottle.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Gthrottle.CapPosition = new System.Drawing.Point(50, 80);
            this.Gthrottle.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Gthrottle.CapsText = new string[] {
        "Throttle",
        "",
        "",
        "",
        ""};
            this.Gthrottle.CapText = "Throttle";
            this.Gthrottle.Center = new System.Drawing.Point(75, 75);
            this.Gthrottle.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "ach3percent", true));
            this.Gthrottle.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "mycruisethrottle", true));
            this.Gthrottle.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "minThrottle", true));
            this.Gthrottle.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "maxThrottle", true));
            this.Gthrottle.MaxValue = 100F;
            this.Gthrottle.MinValue = 0F;
            this.Gthrottle.Name = "Gthrottle";
            this.Gthrottle.Need_Idx = ((byte)(4));
            this.Gthrottle.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Gthrottle.NeedleColor2 = System.Drawing.Color.Brown;
            this.Gthrottle.NeedleEnabled = false;
            this.Gthrottle.NeedleRadius = 50;
            this.Gthrottle.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Gthrottle.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Gold,
        System.Drawing.Color.Brown};
            this.Gthrottle.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        false};
            this.Gthrottle.NeedlesRadius = new int[] {
        50,
        56,
        56,
        56,
        50};
            this.Gthrottle.NeedlesType = new int[] {
        0,
        2,
        2,
        2,
        0};
            this.Gthrottle.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Gthrottle.NeedleType = 0;
            this.Gthrottle.NeedleWidth = 2;
            this.Gthrottle.Range_Idx = ((byte)(2));
            this.Gthrottle.RangeColor = System.Drawing.Color.Orange;
            this.Gthrottle.RangeEnabled = false;
            this.Gthrottle.RangeEndValue = 50F;
            this.Gthrottle.RangeInnerRadius = 1;
            this.Gthrottle.RangeOuterRadius = 70;
            this.Gthrottle.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Gthrottle.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Gthrottle.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.Gthrottle.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Gthrottle.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.Gthrottle.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.Gthrottle.RangeStartValue = 35F;
            this.Gthrottle.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Gthrottle.ScaleLinesInterInnerRadius = 52;
            this.Gthrottle.ScaleLinesInterOuterRadius = 60;
            this.Gthrottle.ScaleLinesInterWidth = 1;
            this.Gthrottle.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Gthrottle.ScaleLinesMajorInnerRadius = 50;
            this.Gthrottle.ScaleLinesMajorOuterRadius = 60;
            this.Gthrottle.ScaleLinesMajorStepValue = 10F;
            this.Gthrottle.ScaleLinesMajorWidth = 2;
            this.Gthrottle.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Gthrottle.ScaleLinesMinorInnerRadius = 55;
            this.Gthrottle.ScaleLinesMinorNumOf = 9;
            this.Gthrottle.ScaleLinesMinorOuterRadius = 60;
            this.Gthrottle.ScaleLinesMinorWidth = 1;
            this.Gthrottle.ScaleNumbersColor = System.Drawing.Color.White;
            this.Gthrottle.ScaleNumbersFormat = null;
            this.Gthrottle.ScaleNumbersRadius = 42;
            this.Gthrottle.ScaleNumbersRotation = 0;
            this.Gthrottle.ScaleNumbersStartScaleLine = 1;
            this.Gthrottle.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.Gthrottle, resources.GetString("Gthrottle.ToolTip"));
            this.Gthrottle.Value = 0F;
            this.Gthrottle.Value0 = 0F;
            this.Gthrottle.Value1 = 0F;
            this.Gthrottle.Value2 = 0F;
            this.Gthrottle.Value3 = 0F;
            this.Gthrottle.Value4 = 0F;
            // 
            // Grud
            // 
            this.Grud.BackColor = System.Drawing.Color.Transparent;
            this.Grud.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Grud, "Grud");
            this.Grud.BaseArcColor = System.Drawing.Color.Transparent;
            this.Grud.BaseArcRadius = 70;
            this.Grud.BaseArcStart = 135;
            this.Grud.BaseArcSweep = 270;
            this.Grud.BaseArcWidth = 2;
            this.Grud.basesize = new System.Drawing.Size(150, 150);
            this.Grud.Cap_Idx = ((byte)(0));
            this.Grud.CapColor = System.Drawing.Color.White;
            this.Grud.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Grud.CapPosition = new System.Drawing.Point(50, 80);
            this.Grud.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Grud.CapsText = new string[] {
        "Rudder",
        "",
        "",
        "",
        ""};
            this.Grud.CapText = "Rudder";
            this.Grud.Center = new System.Drawing.Point(75, 75);
            this.Grud.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "rudder_servo", true));
            this.Grud.MaxValue = 100F;
            this.Grud.MinValue = -100F;
            this.Grud.Name = "Grud";
            this.Grud.Need_Idx = ((byte)(4));
            this.Grud.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Grud.NeedleColor2 = System.Drawing.Color.Brown;
            this.Grud.NeedleEnabled = false;
            this.Grud.NeedleRadius = 50;
            this.Grud.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Red,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Grud.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Brown};
            this.Grud.NeedlesEnabled = new bool[] {
        true,
        false,
        false,
        false,
        false};
            this.Grud.NeedlesRadius = new int[] {
        50,
        50,
        50,
        50,
        50};
            this.Grud.NeedlesType = new int[] {
        0,
        0,
        0,
        0,
        0};
            this.Grud.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Grud.NeedleType = 0;
            this.Grud.NeedleWidth = 2;
            this.Grud.Range_Idx = ((byte)(2));
            this.Grud.RangeColor = System.Drawing.Color.Orange;
            this.Grud.RangeEnabled = false;
            this.Grud.RangeEndValue = 50F;
            this.Grud.RangeInnerRadius = 1;
            this.Grud.RangeOuterRadius = 70;
            this.Grud.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Grud.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Grud.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.Grud.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Grud.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.Grud.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.Grud.RangeStartValue = 35F;
            this.Grud.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Grud.ScaleLinesInterInnerRadius = 52;
            this.Grud.ScaleLinesInterOuterRadius = 60;
            this.Grud.ScaleLinesInterWidth = 1;
            this.Grud.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Grud.ScaleLinesMajorInnerRadius = 50;
            this.Grud.ScaleLinesMajorOuterRadius = 60;
            this.Grud.ScaleLinesMajorStepValue = 25F;
            this.Grud.ScaleLinesMajorWidth = 2;
            this.Grud.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Grud.ScaleLinesMinorInnerRadius = 55;
            this.Grud.ScaleLinesMinorNumOf = 9;
            this.Grud.ScaleLinesMinorOuterRadius = 60;
            this.Grud.ScaleLinesMinorWidth = 1;
            this.Grud.ScaleNumbersColor = System.Drawing.Color.White;
            this.Grud.ScaleNumbersFormat = null;
            this.Grud.ScaleNumbersRadius = 42;
            this.Grud.ScaleNumbersRotation = 0;
            this.Grud.ScaleNumbersStartScaleLine = 1;
            this.Grud.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.Grud, resources.GetString("Grud.ToolTip"));
            this.Grud.Value = 0F;
            this.Grud.Value0 = 0F;
            this.Grud.Value1 = 0F;
            this.Grud.Value2 = 0F;
            this.Grud.Value3 = 0F;
            this.Grud.Value4 = 0F;
            // 
            // Gaftcurrent
            // 
            this.Gaftcurrent.BackColor = System.Drawing.Color.Transparent;
            this.Gaftcurrent.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Gaftcurrent, "Gaftcurrent");
            this.Gaftcurrent.BaseArcColor = System.Drawing.Color.Transparent;
            this.Gaftcurrent.BaseArcRadius = 70;
            this.Gaftcurrent.BaseArcStart = 135;
            this.Gaftcurrent.BaseArcSweep = 270;
            this.Gaftcurrent.BaseArcWidth = 2;
            this.Gaftcurrent.basesize = new System.Drawing.Size(150, 150);
            this.Gaftcurrent.Cap_Idx = ((byte)(0));
            this.Gaftcurrent.CapColor = System.Drawing.Color.White;
            this.Gaftcurrent.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Gaftcurrent.CapPosition = new System.Drawing.Point(50, 80);
            this.Gaftcurrent.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Gaftcurrent.CapsText = new string[] {
        "Elevator",
        "",
        "",
        "",
        ""};
            this.Gaftcurrent.CapText = "Elevator";
            this.Gaftcurrent.Center = new System.Drawing.Point(75, 75);
            this.Gaftcurrent.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "elevator_servo", true));
            this.Gaftcurrent.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "pitch", true));
            this.Gaftcurrent.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "nav_pitch", true));
            this.Gaftcurrent.MaxValue = 100F;
            this.Gaftcurrent.MinValue = -100F;
            this.Gaftcurrent.Name = "Gaftcurrent";
            this.Gaftcurrent.Need_Idx = ((byte)(4));
            this.Gaftcurrent.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Gaftcurrent.NeedleColor2 = System.Drawing.Color.Brown;
            this.Gaftcurrent.NeedleEnabled = false;
            this.Gaftcurrent.NeedleRadius = 50;
            this.Gaftcurrent.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Red,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Gaftcurrent.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Red,
        System.Drawing.Color.Red,
        System.Drawing.Color.Brown};
            this.Gaftcurrent.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        false,
        false};
            this.Gaftcurrent.NeedlesRadius = new int[] {
        50,
        50,
        56,
        56,
        50};
            this.Gaftcurrent.NeedlesType = new int[] {
        0,
        0,
        2,
        2,
        0};
            this.Gaftcurrent.NeedlesWidth = new int[] {
        2,
        1,
        2,
        2,
        2};
            this.Gaftcurrent.NeedleType = 0;
            this.Gaftcurrent.NeedleWidth = 2;
            this.Gaftcurrent.Range_Idx = ((byte)(2));
            this.Gaftcurrent.RangeColor = System.Drawing.Color.Orange;
            this.Gaftcurrent.RangeEnabled = false;
            this.Gaftcurrent.RangeEndValue = 50F;
            this.Gaftcurrent.RangeInnerRadius = 1;
            this.Gaftcurrent.RangeOuterRadius = 70;
            this.Gaftcurrent.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Gaftcurrent.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Gaftcurrent.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.Gaftcurrent.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Gaftcurrent.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.Gaftcurrent.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.Gaftcurrent.RangeStartValue = 35F;
            this.Gaftcurrent.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Gaftcurrent.ScaleLinesInterInnerRadius = 52;
            this.Gaftcurrent.ScaleLinesInterOuterRadius = 60;
            this.Gaftcurrent.ScaleLinesInterWidth = 1;
            this.Gaftcurrent.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Gaftcurrent.ScaleLinesMajorInnerRadius = 50;
            this.Gaftcurrent.ScaleLinesMajorOuterRadius = 60;
            this.Gaftcurrent.ScaleLinesMajorStepValue = 25F;
            this.Gaftcurrent.ScaleLinesMajorWidth = 2;
            this.Gaftcurrent.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Gaftcurrent.ScaleLinesMinorInnerRadius = 55;
            this.Gaftcurrent.ScaleLinesMinorNumOf = 9;
            this.Gaftcurrent.ScaleLinesMinorOuterRadius = 60;
            this.Gaftcurrent.ScaleLinesMinorWidth = 1;
            this.Gaftcurrent.ScaleNumbersColor = System.Drawing.Color.White;
            this.Gaftcurrent.ScaleNumbersFormat = null;
            this.Gaftcurrent.ScaleNumbersRadius = 42;
            this.Gaftcurrent.ScaleNumbersRotation = 0;
            this.Gaftcurrent.ScaleNumbersStartScaleLine = 1;
            this.Gaftcurrent.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.Gaftcurrent, resources.GetString("Gaftcurrent.ToolTip"));
            this.Gaftcurrent.Value = 0F;
            this.Gaftcurrent.Value0 = 0F;
            this.Gaftcurrent.Value1 = 0F;
            this.Gaftcurrent.Value2 = 0F;
            this.Gaftcurrent.Value3 = 0F;
            this.Gaftcurrent.Value4 = 0F;
            // 
            // Gfwdcurrent
            // 
            this.Gfwdcurrent.BackColor = System.Drawing.Color.Transparent;
            this.Gfwdcurrent.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Gfwdcurrent, "Gfwdcurrent");
            this.Gfwdcurrent.BaseArcColor = System.Drawing.Color.Transparent;
            this.Gfwdcurrent.BaseArcRadius = 70;
            this.Gfwdcurrent.BaseArcStart = 135;
            this.Gfwdcurrent.BaseArcSweep = 270;
            this.Gfwdcurrent.BaseArcWidth = 2;
            this.Gfwdcurrent.basesize = new System.Drawing.Size(150, 150);
            this.Gfwdcurrent.Cap_Idx = ((byte)(0));
            this.Gfwdcurrent.CapColor = System.Drawing.Color.White;
            this.Gfwdcurrent.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Gfwdcurrent.CapPosition = new System.Drawing.Point(52, 80);
            this.Gfwdcurrent.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(52, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Gfwdcurrent.CapsText = new string[] {
        "Aileron",
        "",
        "",
        "",
        ""};
            this.Gfwdcurrent.CapText = "Aileron";
            this.Gfwdcurrent.Center = new System.Drawing.Point(75, 75);
            this.Gfwdcurrent.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "aileron_servo", true));
            this.Gfwdcurrent.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "roll", true));
            this.Gfwdcurrent.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "nav_roll", true));
            this.Gfwdcurrent.MaxValue = 100F;
            this.Gfwdcurrent.MinValue = -100F;
            this.Gfwdcurrent.Name = "Gfwdcurrent";
            this.Gfwdcurrent.Need_Idx = ((byte)(4));
            this.Gfwdcurrent.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Gfwdcurrent.NeedleColor2 = System.Drawing.Color.Brown;
            this.Gfwdcurrent.NeedleEnabled = false;
            this.Gfwdcurrent.NeedleRadius = 50;
            this.Gfwdcurrent.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Red,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Gfwdcurrent.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Red,
        System.Drawing.Color.Red,
        System.Drawing.Color.Brown};
            this.Gfwdcurrent.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        false,
        false};
            this.Gfwdcurrent.NeedlesRadius = new int[] {
        50,
        50,
        56,
        56,
        50};
            this.Gfwdcurrent.NeedlesType = new int[] {
        0,
        0,
        2,
        2,
        0};
            this.Gfwdcurrent.NeedlesWidth = new int[] {
        2,
        1,
        2,
        2,
        2};
            this.Gfwdcurrent.NeedleType = 0;
            this.Gfwdcurrent.NeedleWidth = 2;
            this.Gfwdcurrent.Range_Idx = ((byte)(2));
            this.Gfwdcurrent.RangeColor = System.Drawing.Color.Orange;
            this.Gfwdcurrent.RangeEnabled = false;
            this.Gfwdcurrent.RangeEndValue = 50F;
            this.Gfwdcurrent.RangeInnerRadius = 1;
            this.Gfwdcurrent.RangeOuterRadius = 70;
            this.Gfwdcurrent.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Gfwdcurrent.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Gfwdcurrent.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.Gfwdcurrent.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Gfwdcurrent.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.Gfwdcurrent.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.Gfwdcurrent.RangeStartValue = 35F;
            this.Gfwdcurrent.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Gfwdcurrent.ScaleLinesInterInnerRadius = 52;
            this.Gfwdcurrent.ScaleLinesInterOuterRadius = 60;
            this.Gfwdcurrent.ScaleLinesInterWidth = 1;
            this.Gfwdcurrent.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Gfwdcurrent.ScaleLinesMajorInnerRadius = 50;
            this.Gfwdcurrent.ScaleLinesMajorOuterRadius = 60;
            this.Gfwdcurrent.ScaleLinesMajorStepValue = 25F;
            this.Gfwdcurrent.ScaleLinesMajorWidth = 2;
            this.Gfwdcurrent.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Gfwdcurrent.ScaleLinesMinorInnerRadius = 55;
            this.Gfwdcurrent.ScaleLinesMinorNumOf = 9;
            this.Gfwdcurrent.ScaleLinesMinorOuterRadius = 60;
            this.Gfwdcurrent.ScaleLinesMinorWidth = 1;
            this.Gfwdcurrent.ScaleNumbersColor = System.Drawing.Color.White;
            this.Gfwdcurrent.ScaleNumbersFormat = null;
            this.Gfwdcurrent.ScaleNumbersRadius = 42;
            this.Gfwdcurrent.ScaleNumbersRotation = 0;
            this.Gfwdcurrent.ScaleNumbersStartScaleLine = 1;
            this.Gfwdcurrent.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.Gfwdcurrent, resources.GetString("Gfwdcurrent.ToolTip"));
            this.Gfwdcurrent.Value = 0F;
            this.Gfwdcurrent.Value0 = 0F;
            this.Gfwdcurrent.Value1 = 0F;
            this.Gfwdcurrent.Value2 = 0F;
            this.Gfwdcurrent.Value3 = 0F;
            this.Gfwdcurrent.Value4 = 0F;
            // 
            // Grpm
            // 
            this.Grpm.BackColor = System.Drawing.Color.Transparent;
            this.Grpm.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Grpm, "Grpm");
            this.Grpm.BaseArcColor = System.Drawing.Color.Transparent;
            this.Grpm.BaseArcRadius = 70;
            this.Grpm.BaseArcStart = 135;
            this.Grpm.BaseArcSweep = 270;
            this.Grpm.BaseArcWidth = 2;
            this.Grpm.basesize = new System.Drawing.Size(150, 150);
            this.Grpm.Cap_Idx = ((byte)(0));
            this.Grpm.CapColor = System.Drawing.Color.White;
            this.Grpm.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Grpm.CapPosition = new System.Drawing.Point(50, 80);
            this.Grpm.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Grpm.CapsText = new string[] {
        "RPM",
        "x1000",
        "",
        "",
        ""};
            this.Grpm.CapText = "RPM";
            this.Grpm.Center = new System.Drawing.Point(75, 75);
            this.Grpm.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "aRPM", true));
            this.Grpm.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "aRPMpwm", true));
            this.Grpm.MaxValue = 6F;
            this.Grpm.MinValue = 0F;
            this.Grpm.Name = "Grpm";
            this.Grpm.Need_Idx = ((byte)(4));
            this.Grpm.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Grpm.NeedleColor2 = System.Drawing.Color.Brown;
            this.Grpm.NeedleEnabled = false;
            this.Grpm.NeedleRadius = 50;
            this.Grpm.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Grpm.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.Green,
        System.Drawing.Color.White,
        System.Drawing.Color.Green,
        System.Drawing.Color.Brown};
            this.Grpm.NeedlesEnabled = new bool[] {
        true,
        true,
        false,
        false,
        false};
            this.Grpm.NeedlesRadius = new int[] {
        50,
        50,
        50,
        56,
        50};
            this.Grpm.NeedlesType = new int[] {
        0,
        2,
        0,
        2,
        0};
            this.Grpm.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Grpm.NeedleType = 0;
            this.Grpm.NeedleWidth = 2;
            this.Grpm.Range_Idx = ((byte)(2));
            this.Grpm.RangeColor = System.Drawing.Color.Orange;
            this.Grpm.RangeEnabled = false;
            this.Grpm.RangeEndValue = 50F;
            this.Grpm.RangeInnerRadius = 1;
            this.Grpm.RangeOuterRadius = 70;
            this.Grpm.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Grpm.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Grpm.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.Grpm.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Grpm.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.Grpm.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.Grpm.RangeStartValue = 35F;
            this.Grpm.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Grpm.ScaleLinesInterInnerRadius = 52;
            this.Grpm.ScaleLinesInterOuterRadius = 60;
            this.Grpm.ScaleLinesInterWidth = 1;
            this.Grpm.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Grpm.ScaleLinesMajorInnerRadius = 50;
            this.Grpm.ScaleLinesMajorOuterRadius = 60;
            this.Grpm.ScaleLinesMajorStepValue = 1F;
            this.Grpm.ScaleLinesMajorWidth = 2;
            this.Grpm.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Grpm.ScaleLinesMinorInnerRadius = 55;
            this.Grpm.ScaleLinesMinorNumOf = 9;
            this.Grpm.ScaleLinesMinorOuterRadius = 60;
            this.Grpm.ScaleLinesMinorWidth = 1;
            this.Grpm.ScaleNumbersColor = System.Drawing.Color.White;
            this.Grpm.ScaleNumbersFormat = null;
            this.Grpm.ScaleNumbersRadius = 42;
            this.Grpm.ScaleNumbersRotation = 0;
            this.Grpm.ScaleNumbersStartScaleLine = 1;
            this.Grpm.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.Grpm, resources.GetString("Grpm.ToolTip"));
            this.Grpm.Value = 0F;
            this.Grpm.Value0 = 0F;
            this.Grpm.Value1 = 0F;
            this.Grpm.Value2 = 0F;
            this.Grpm.Value3 = 0F;
            this.Grpm.Value4 = 0F;
            this.Grpm.DoubleClick += new System.EventHandler(this.Gspeed_DoubleClick);
            // 
            // Gspeed
            // 
            this.Gspeed.BackColor = System.Drawing.Color.Transparent;
            this.Gspeed.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Gspeed, "Gspeed");
            this.Gspeed.BaseArcColor = System.Drawing.Color.Transparent;
            this.Gspeed.BaseArcRadius = 70;
            this.Gspeed.BaseArcStart = 135;
            this.Gspeed.BaseArcSweep = 270;
            this.Gspeed.BaseArcWidth = 2;
            this.Gspeed.basesize = new System.Drawing.Size(150, 150);
            this.Gspeed.Cap_Idx = ((byte)(0));
            this.Gspeed.CapColor = System.Drawing.Color.White;
            this.Gspeed.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Gspeed.CapPosition = new System.Drawing.Point(50, 80);
            this.Gspeed.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(50, 80),
        new System.Drawing.Point(50, 110),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Gspeed.CapsText = new string[] {
        "Speed",
        "",
        "",
        "",
        ""};
            this.Gspeed.CapText = "Speed";
            this.Gspeed.Center = new System.Drawing.Point(75, 75);
            this.Gspeed.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "airspeed", true));
            this.Gspeed.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "groundspeed", true));
            this.Gspeed.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "wind_vel", true));
            this.Gspeed.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "targetairspeed", true));
            this.Gspeed.MaxValue = 60F;
            this.Gspeed.MinValue = 0F;
            this.Gspeed.Name = "Gspeed";
            this.Gspeed.Need_Idx = ((byte)(4));
            this.Gspeed.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Gspeed.NeedleColor2 = System.Drawing.Color.Brown;
            this.Gspeed.NeedleEnabled = false;
            this.Gspeed.NeedleRadius = 50;
            this.Gspeed.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Red,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Gspeed.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.Green,
        System.Drawing.Color.Brown};
            this.Gspeed.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        false};
            this.Gspeed.NeedlesRadius = new int[] {
        50,
        50,
        50,
        56,
        50};
            this.Gspeed.NeedlesType = new int[] {
        0,
        0,
        0,
        2,
        0};
            this.Gspeed.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Gspeed.NeedleType = 0;
            this.Gspeed.NeedleWidth = 2;
            this.Gspeed.Range_Idx = ((byte)(2));
            this.Gspeed.RangeColor = System.Drawing.Color.Orange;
            this.Gspeed.RangeEnabled = false;
            this.Gspeed.RangeEndValue = 50F;
            this.Gspeed.RangeInnerRadius = 1;
            this.Gspeed.RangeOuterRadius = 70;
            this.Gspeed.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Gspeed.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Gspeed.RangesEndValue = new float[] {
        35F,
        60F,
        50F,
        0F,
        0F};
            this.Gspeed.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Gspeed.RangesOuterRadius = new int[] {
        70,
        70,
        70,
        80,
        80};
            this.Gspeed.RangesStartValue = new float[] {
        0F,
        50F,
        35F,
        0F,
        0F};
            this.Gspeed.RangeStartValue = 35F;
            this.Gspeed.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Gspeed.ScaleLinesInterInnerRadius = 52;
            this.Gspeed.ScaleLinesInterOuterRadius = 60;
            this.Gspeed.ScaleLinesInterWidth = 1;
            this.Gspeed.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Gspeed.ScaleLinesMajorInnerRadius = 50;
            this.Gspeed.ScaleLinesMajorOuterRadius = 60;
            this.Gspeed.ScaleLinesMajorStepValue = 10F;
            this.Gspeed.ScaleLinesMajorWidth = 2;
            this.Gspeed.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Gspeed.ScaleLinesMinorInnerRadius = 55;
            this.Gspeed.ScaleLinesMinorNumOf = 9;
            this.Gspeed.ScaleLinesMinorOuterRadius = 60;
            this.Gspeed.ScaleLinesMinorWidth = 1;
            this.Gspeed.ScaleNumbersColor = System.Drawing.Color.White;
            this.Gspeed.ScaleNumbersFormat = null;
            this.Gspeed.ScaleNumbersRadius = 42;
            this.Gspeed.ScaleNumbersRotation = 0;
            this.Gspeed.ScaleNumbersStartScaleLine = 1;
            this.Gspeed.ScaleNumbersStepScaleLines = 1;
            this.toolTip1.SetToolTip(this.Gspeed, resources.GetString("Gspeed.ToolTip"));
            this.Gspeed.Value = 0F;
            this.Gspeed.Value0 = 0F;
            this.Gspeed.Value1 = 0F;
            this.Gspeed.Value2 = 0F;
            this.Gspeed.Value3 = 0F;
            this.Gspeed.Value4 = 0F;
            this.Gspeed.DoubleClick += new System.EventHandler(this.Gspeed_DoubleClick);
            // 
            // Galt
            // 
            this.Galt.BackColor = System.Drawing.Color.Transparent;
            this.Galt.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Galt, "Galt");
            this.Galt.BaseArcColor = System.Drawing.Color.Transparent;
            this.Galt.BaseArcRadius = 60;
            this.Galt.BaseArcStart = 270;
            this.Galt.BaseArcSweep = 360;
            this.Galt.BaseArcWidth = 2;
            this.Galt.basesize = new System.Drawing.Size(150, 150);
            this.Galt.Cap_Idx = ((byte)(0));
            this.Galt.CapColor = System.Drawing.Color.White;
            this.Galt.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Galt.CapPosition = new System.Drawing.Point(67, 80);
            this.Galt.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(67, 80),
        new System.Drawing.Point(30, 55),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Galt.CapsText = new string[] {
        "Alt",
        "",
        "",
        "",
        ""};
            this.Galt.CapText = "Alt";
            this.Galt.Center = new System.Drawing.Point(75, 75);
            this.Galt.DataBindings.Add(new System.Windows.Forms.Binding("Value3", this.bindingSource1, "altd100", true));
            this.Galt.DataBindings.Add(new System.Windows.Forms.Binding("Value2", this.bindingSource1, "baro_altd100", true));
            this.Galt.DataBindings.Add(new System.Windows.Forms.Binding("Value1", this.bindingSource1, "gps_altd100", true));
            this.Galt.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "targetaltd100", true));
            this.Galt.MaxValue = 9.99F;
            this.Galt.MinValue = 0F;
            this.Galt.Name = "Galt";
            this.Galt.Need_Idx = ((byte)(4));
            this.Galt.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Galt.NeedleColor2 = System.Drawing.Color.White;
            this.Galt.NeedleEnabled = false;
            this.Galt.NeedleRadius = 50;
            this.Galt.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Blue,
        AGaugeApp.AGauge.NeedleColorEnum.Green,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Galt.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.Green,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White};
            this.Galt.NeedlesEnabled = new bool[] {
        true,
        true,
        true,
        true,
        false};
            this.Galt.NeedlesRadius = new int[] {
        56,
        50,
        50,
        50,
        50};
            this.Galt.NeedlesType = new int[] {
        2,
        0,
        0,
        0,
        0};
            this.Galt.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Galt.NeedleType = 0;
            this.Galt.NeedleWidth = 2;
            this.Galt.Range_Idx = ((byte)(0));
            this.Galt.RangeColor = System.Drawing.Color.LightGreen;
            this.Galt.RangeEnabled = false;
            this.Galt.RangeEndValue = 360F;
            this.Galt.RangeInnerRadius = 1;
            this.Galt.RangeOuterRadius = 60;
            this.Galt.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Galt.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Galt.RangesEndValue = new float[] {
        360F,
        200F,
        150F,
        0F,
        0F};
            this.Galt.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Galt.RangesOuterRadius = new int[] {
        60,
        60,
        60,
        80,
        80};
            this.Galt.RangesStartValue = new float[] {
        0F,
        150F,
        75F,
        0F,
        0F};
            this.Galt.RangeStartValue = 0F;
            this.Galt.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Galt.ScaleLinesInterInnerRadius = 52;
            this.Galt.ScaleLinesInterOuterRadius = 60;
            this.Galt.ScaleLinesInterWidth = 1;
            this.Galt.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Galt.ScaleLinesMajorInnerRadius = 50;
            this.Galt.ScaleLinesMajorOuterRadius = 60;
            this.Galt.ScaleLinesMajorStepValue = 1F;
            this.Galt.ScaleLinesMajorWidth = 2;
            this.Galt.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Galt.ScaleLinesMinorInnerRadius = 55;
            this.Galt.ScaleLinesMinorNumOf = 9;
            this.Galt.ScaleLinesMinorOuterRadius = 60;
            this.Galt.ScaleLinesMinorWidth = 1;
            this.Galt.ScaleNumbersColor = System.Drawing.Color.White;
            this.Galt.ScaleNumbersFormat = "";
            this.Galt.ScaleNumbersRadius = 42;
            this.Galt.ScaleNumbersRotation = 0;
            this.Galt.ScaleNumbersStartScaleLine = 1;
            this.Galt.ScaleNumbersStepScaleLines = 1;
            this.Galt.Value = 0F;
            this.Galt.Value0 = 0F;
            this.Galt.Value1 = 0F;
            this.Galt.Value2 = 0F;
            this.Galt.Value3 = 0F;
            this.Galt.Value4 = 0F;
            // 
            // Gvspeed
            // 
            this.Gvspeed.BackColor = System.Drawing.Color.Transparent;
            this.Gvspeed.BackgroundImage = global::ArdupilotMega.Properties.Resources.Gaugebg;
            resources.ApplyResources(this.Gvspeed, "Gvspeed");
            this.Gvspeed.BaseArcColor = System.Drawing.Color.Transparent;
            this.Gvspeed.BaseArcRadius = 60;
            this.Gvspeed.BaseArcStart = 20;
            this.Gvspeed.BaseArcSweep = 320;
            this.Gvspeed.BaseArcWidth = 2;
            this.Gvspeed.basesize = new System.Drawing.Size(150, 150);
            this.Gvspeed.Cap_Idx = ((byte)(0));
            this.Gvspeed.CapColor = System.Drawing.Color.White;
            this.Gvspeed.CapColors = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black,
        System.Drawing.Color.Black};
            this.Gvspeed.CapPosition = new System.Drawing.Point(63, 80);
            this.Gvspeed.CapsPosition = new System.Drawing.Point[] {
        new System.Drawing.Point(63, 80),
        new System.Drawing.Point(30, 55),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10),
        new System.Drawing.Point(10, 10)};
            this.Gvspeed.CapsText = new string[] {
        "VSI",
        "",
        "",
        "",
        ""};
            this.Gvspeed.CapText = "VSI";
            this.Gvspeed.Center = new System.Drawing.Point(75, 75);
            this.Gvspeed.DataBindings.Add(new System.Windows.Forms.Binding("Value0", this.bindingSource1, "climbrate", true));
            this.Gvspeed.MaxValue = 10F;
            this.Gvspeed.MinValue = -10F;
            this.Gvspeed.Name = "Gvspeed";
            this.Gvspeed.Need_Idx = ((byte)(4));
            this.Gvspeed.NeedleColor1 = AGaugeApp.AGauge.NeedleColorEnum.Gray;
            this.Gvspeed.NeedleColor2 = System.Drawing.Color.White;
            this.Gvspeed.NeedleEnabled = false;
            this.Gvspeed.NeedleRadius = 50;
            this.Gvspeed.NeedlesColor1 = new AGaugeApp.AGauge.NeedleColorEnum[] {
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray,
        AGaugeApp.AGauge.NeedleColorEnum.Gray};
            this.Gvspeed.NeedlesColor2 = new System.Drawing.Color[] {
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White,
        System.Drawing.Color.White};
            this.Gvspeed.NeedlesEnabled = new bool[] {
        true,
        false,
        false,
        false,
        false};
            this.Gvspeed.NeedlesRadius = new int[] {
        50,
        50,
        50,
        50,
        50};
            this.Gvspeed.NeedlesType = new int[] {
        0,
        0,
        0,
        0,
        0};
            this.Gvspeed.NeedlesWidth = new int[] {
        2,
        2,
        2,
        2,
        2};
            this.Gvspeed.NeedleType = 0;
            this.Gvspeed.NeedleWidth = 2;
            this.Gvspeed.Range_Idx = ((byte)(0));
            this.Gvspeed.RangeColor = System.Drawing.Color.LightGreen;
            this.Gvspeed.RangeEnabled = false;
            this.Gvspeed.RangeEndValue = 360F;
            this.Gvspeed.RangeInnerRadius = 1;
            this.Gvspeed.RangeOuterRadius = 60;
            this.Gvspeed.RangesColor = new System.Drawing.Color[] {
        System.Drawing.Color.LightGreen,
        System.Drawing.Color.Red,
        System.Drawing.Color.Orange,
        System.Drawing.SystemColors.Control,
        System.Drawing.SystemColors.Control};
            this.Gvspeed.RangesEnabled = new bool[] {
        false,
        false,
        false,
        false,
        false};
            this.Gvspeed.RangesEndValue = new float[] {
        360F,
        200F,
        150F,
        0F,
        0F};
            this.Gvspeed.RangesInnerRadius = new int[] {
        1,
        1,
        1,
        70,
        70};
            this.Gvspeed.RangesOuterRadius = new int[] {
        60,
        60,
        60,
        80,
        80};
            this.Gvspeed.RangesStartValue = new float[] {
        0F,
        150F,
        75F,
        0F,
        0F};
            this.Gvspeed.RangeStartValue = 0F;
            this.Gvspeed.ScaleLinesInterColor = System.Drawing.Color.White;
            this.Gvspeed.ScaleLinesInterInnerRadius = 52;
            this.Gvspeed.ScaleLinesInterOuterRadius = 60;
            this.Gvspeed.ScaleLinesInterWidth = 1;
            this.Gvspeed.ScaleLinesMajorColor = System.Drawing.Color.White;
            this.Gvspeed.ScaleLinesMajorInnerRadius = 50;
            this.Gvspeed.ScaleLinesMajorOuterRadius = 60;
            this.Gvspeed.ScaleLinesMajorStepValue = 2F;
            this.Gvspeed.ScaleLinesMajorWidth = 2;
            this.Gvspeed.ScaleLinesMinorColor = System.Drawing.Color.White;
            this.Gvspeed.ScaleLinesMinorInnerRadius = 55;
            this.Gvspeed.ScaleLinesMinorNumOf = 9;
            this.Gvspeed.ScaleLinesMinorOuterRadius = 60;
            this.Gvspeed.ScaleLinesMinorWidth = 1;
            this.Gvspeed.ScaleNumbersColor = System.Drawing.Color.White;
            this.Gvspeed.ScaleNumbersFormat = "";
            this.Gvspeed.ScaleNumbersRadius = 42;
            this.Gvspeed.ScaleNumbersRotation = 0;
            this.Gvspeed.ScaleNumbersStartScaleLine = 1;
            this.Gvspeed.ScaleNumbersStepScaleLines = 1;
            this.Gvspeed.Value = 0F;
            this.Gvspeed.Value0 = 0F;
            this.Gvspeed.Value1 = 0F;
            this.Gvspeed.Value2 = 0F;
            this.Gvspeed.Value3 = 0F;
            this.Gvspeed.Value4 = 0F;
            // 
            // tabStatus
            // 
            resources.ApplyResources(this.tabStatus, "tabStatus");
            this.tabStatus.Name = "tabStatus";
            this.tabStatus.UseVisualStyleBackColor = true;
            // 
            // tabTLogs
            // 
            this.tabTLogs.Controls.Add(this.lbl_logpercent);
            this.tabTLogs.Controls.Add(this.NUM_playbackspeed);
            this.tabTLogs.Controls.Add(this.BUT_log2kml);
            this.tabTLogs.Controls.Add(this.tracklog);
            this.tabTLogs.Controls.Add(this.BUT_playlog);
            this.tabTLogs.Controls.Add(this.BUT_loadtelem);
            resources.ApplyResources(this.tabTLogs, "tabTLogs");
            this.tabTLogs.Name = "tabTLogs";
            this.tabTLogs.UseVisualStyleBackColor = true;
            // 
            // lbl_logpercent
            // 
            resources.ApplyResources(this.lbl_logpercent, "lbl_logpercent");
            this.lbl_logpercent.Name = "lbl_logpercent";
            this.lbl_logpercent.resize = false;
            // 
            // NUM_playbackspeed
            // 
            this.NUM_playbackspeed.DecimalPlaces = 1;
            this.NUM_playbackspeed.Increment = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            resources.ApplyResources(this.NUM_playbackspeed, "NUM_playbackspeed");
            this.NUM_playbackspeed.Maximum = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.NUM_playbackspeed.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            65536});
            this.NUM_playbackspeed.Name = "NUM_playbackspeed";
            this.toolTip1.SetToolTip(this.NUM_playbackspeed, resources.GetString("NUM_playbackspeed.ToolTip"));
            this.NUM_playbackspeed.Value = new decimal(new int[] {
            1,
            0,
            0,
            0});
            // 
            // BUT_log2kml
            // 
            resources.ApplyResources(this.BUT_log2kml, "BUT_log2kml");
            this.BUT_log2kml.Name = "BUT_log2kml";
            this.BUT_log2kml.UseVisualStyleBackColor = true;
            this.BUT_log2kml.Click += new System.EventHandler(this.BUT_log2kml_Click);
            // 
            // tracklog
            // 
            this.tracklog.BackColor = System.Drawing.SystemColors.Control;
            resources.ApplyResources(this.tracklog, "tracklog");
            this.tracklog.Maximum = 100;
            this.tracklog.Name = "tracklog";
            this.tracklog.Scroll += new System.EventHandler(this.tracklog_Scroll);
            // 
            // BUT_playlog
            // 
            resources.ApplyResources(this.BUT_playlog, "BUT_playlog");
            this.BUT_playlog.Name = "BUT_playlog";
            this.BUT_playlog.UseVisualStyleBackColor = true;
            this.BUT_playlog.Click += new System.EventHandler(this.BUT_playlog_Click);
            // 
            // BUT_loadtelem
            // 
            resources.ApplyResources(this.BUT_loadtelem, "BUT_loadtelem");
            this.BUT_loadtelem.Name = "BUT_loadtelem";
            this.BUT_loadtelem.UseVisualStyleBackColor = true;
            this.BUT_loadtelem.Click += new System.EventHandler(this.BUT_loadtelem_Click);
            // 
            // tabCDnR
            // 
            this.tabCDnR.Controls.Add(this.CDnR_Send);
            this.tabCDnR.Controls.Add(this.CDnR_Time);
            this.tabCDnR.Controls.Add(this.CDnR_Altitude);
            this.tabCDnR.Controls.Add(this.CDnR_Airspeed);
            this.tabCDnR.Controls.Add(this.CDnR_Heading);
            this.tabCDnR.Controls.Add(this.CDnR_CheckTime);
            this.tabCDnR.Controls.Add(this.CDnR_CheckAltitude);
            this.tabCDnR.Controls.Add(this.CDnR_CheckAirspeed);
            this.tabCDnR.Controls.Add(this.CDnR_CheckHeading);
            resources.ApplyResources(this.tabCDnR, "tabCDnR");
            this.tabCDnR.Name = "tabCDnR";
            this.tabCDnR.UseVisualStyleBackColor = true;
            // 
            // CDnR_Send
            // 
            resources.ApplyResources(this.CDnR_Send, "CDnR_Send");
            this.CDnR_Send.Name = "CDnR_Send";
            this.CDnR_Send.UseVisualStyleBackColor = true;
            this.CDnR_Send.Click += new System.EventHandler(this.CDnR_Send_Click);
            // 
            // CDnR_Time
            // 
            resources.ApplyResources(this.CDnR_Time, "CDnR_Time");
            this.CDnR_Time.Maximum = new decimal(new int[] {
            300,
            0,
            0,
            0});
            this.CDnR_Time.Name = "CDnR_Time";
            // 
            // CDnR_Altitude
            // 
            resources.ApplyResources(this.CDnR_Altitude, "CDnR_Altitude");
            this.CDnR_Altitude.Maximum = new decimal(new int[] {
            1200,
            0,
            0,
            0});
            this.CDnR_Altitude.Minimum = new decimal(new int[] {
            100,
            0,
            0,
            0});
            this.CDnR_Altitude.Name = "CDnR_Altitude";
            this.CDnR_Altitude.Value = new decimal(new int[] {
            100,
            0,
            0,
            0});
            // 
            // CDnR_Airspeed
            // 
            resources.ApplyResources(this.CDnR_Airspeed, "CDnR_Airspeed");
            this.CDnR_Airspeed.Maximum = new decimal(new int[] {
            40,
            0,
            0,
            0});
            this.CDnR_Airspeed.Minimum = new decimal(new int[] {
            18,
            0,
            0,
            0});
            this.CDnR_Airspeed.Name = "CDnR_Airspeed";
            this.CDnR_Airspeed.Value = new decimal(new int[] {
            18,
            0,
            0,
            0});
            // 
            // CDnR_Heading
            // 
            resources.ApplyResources(this.CDnR_Heading, "CDnR_Heading");
            this.CDnR_Heading.Maximum = new decimal(new int[] {
            360,
            0,
            0,
            0});
            this.CDnR_Heading.Name = "CDnR_Heading";
            // 
            // CDnR_CheckTime
            // 
            resources.ApplyResources(this.CDnR_CheckTime, "CDnR_CheckTime");
            this.CDnR_CheckTime.Name = "CDnR_CheckTime";
            this.CDnR_CheckTime.UseVisualStyleBackColor = true;
            // 
            // CDnR_CheckAltitude
            // 
            resources.ApplyResources(this.CDnR_CheckAltitude, "CDnR_CheckAltitude");
            this.CDnR_CheckAltitude.Name = "CDnR_CheckAltitude";
            this.CDnR_CheckAltitude.UseVisualStyleBackColor = true;
            // 
            // CDnR_CheckAirspeed
            // 
            resources.ApplyResources(this.CDnR_CheckAirspeed, "CDnR_CheckAirspeed");
            this.CDnR_CheckAirspeed.Name = "CDnR_CheckAirspeed";
            this.CDnR_CheckAirspeed.UseVisualStyleBackColor = true;
            // 
            // CDnR_CheckHeading
            // 
            resources.ApplyResources(this.CDnR_CheckHeading, "CDnR_CheckHeading");
            this.CDnR_CheckHeading.Name = "CDnR_CheckHeading";
            this.CDnR_CheckHeading.UseVisualStyleBackColor = true;
            // 
            // tabPage1
            // 
            this.tabPage1.Controls.Add(this.simSendMessage);
            this.tabPage1.Controls.Add(this.label3);
            this.tabPage1.Controls.Add(this.label2);
            this.tabPage1.Controls.Add(this.trafficTimeToWpt);
            this.tabPage1.Controls.Add(this.trafficSimTime);
            resources.ApplyResources(this.tabPage1, "tabPage1");
            this.tabPage1.Name = "tabPage1";
            this.tabPage1.UseVisualStyleBackColor = true;
            // 
            // simSendMessage
            // 
            resources.ApplyResources(this.simSendMessage, "simSendMessage");
            this.simSendMessage.Name = "simSendMessage";
            this.simSendMessage.UseVisualStyleBackColor = true;
            this.simSendMessage.Click += new System.EventHandler(this.simSendMessage_Click);
            // 
            // label3
            // 
            resources.ApplyResources(this.label3, "label3");
            this.label3.Name = "label3";
            // 
            // label2
            // 
            resources.ApplyResources(this.label2, "label2");
            this.label2.Name = "label2";
            // 
            // trafficTimeToWpt
            // 
            resources.ApplyResources(this.trafficTimeToWpt, "trafficTimeToWpt");
            this.trafficTimeToWpt.Name = "trafficTimeToWpt";
            // 
            // trafficSimTime
            // 
            resources.ApplyResources(this.trafficSimTime, "trafficSimTime");
            this.trafficSimTime.Name = "trafficSimTime";
            // 
            // tableMap
            // 
            resources.ApplyResources(this.tableMap, "tableMap");
            this.tableMap.Controls.Add(this.splitContainer1, 0, 0);
            this.tableMap.Controls.Add(this.panel1, 0, 1);
            this.tableMap.Name = "tableMap";
            // 
            // splitContainer1
            // 
            resources.ApplyResources(this.splitContainer1, "splitContainer1");
            this.splitContainer1.Name = "splitContainer1";
            // 
            // splitContainer1.Panel1
            // 
            this.splitContainer1.Panel1.Controls.Add(this.zg1);
            this.splitContainer1.Panel1Collapsed = true;
            // 
            // splitContainer1.Panel2
            // 
            this.splitContainer1.Panel2.Controls.Add(this.lbl_winddir);
            this.splitContainer1.Panel2.Controls.Add(this.lbl_windvel);
            this.splitContainer1.Panel2.Controls.Add(this.gMapControl1);
            // 
            // zg1
            // 
            resources.ApplyResources(this.zg1, "zg1");
            this.zg1.Name = "zg1";
            this.zg1.ScrollGrace = 0D;
            this.zg1.ScrollMaxX = 0D;
            this.zg1.ScrollMaxY = 0D;
            this.zg1.ScrollMaxY2 = 0D;
            this.zg1.ScrollMinX = 0D;
            this.zg1.ScrollMinY = 0D;
            this.zg1.ScrollMinY2 = 0D;
            this.zg1.Click += new System.EventHandler(this.zg1_Click);
            this.zg1.DoubleClick += new System.EventHandler(this.zg1_DoubleClick);
            // 
            // lbl_winddir
            // 
            this.lbl_winddir.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.bindingSource1, "wind_dir", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, null, "Dir: 0"));
            resources.ApplyResources(this.lbl_winddir, "lbl_winddir");
            this.lbl_winddir.Name = "lbl_winddir";
            this.lbl_winddir.resize = true;
            this.toolTip1.SetToolTip(this.lbl_winddir, resources.GetString("lbl_winddir.ToolTip"));
            // 
            // lbl_windvel
            // 
            this.lbl_windvel.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.bindingSource1, "wind_vel", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, null, "Vel: 0"));
            resources.ApplyResources(this.lbl_windvel, "lbl_windvel");
            this.lbl_windvel.Name = "lbl_windvel";
            this.lbl_windvel.resize = true;
            this.toolTip1.SetToolTip(this.lbl_windvel, resources.GetString("lbl_windvel.ToolTip"));
            // 
            // gMapControl1
            // 
            this.gMapControl1.BackColor = System.Drawing.Color.Transparent;
            this.gMapControl1.Bearing = 0F;
            this.gMapControl1.CanDragMap = true;
            this.gMapControl1.ContextMenuStrip = this.contextMenuStrip1;
            resources.ApplyResources(this.gMapControl1, "gMapControl1");
            this.gMapControl1.GrayScaleMode = false;
            this.gMapControl1.LevelsKeepInMemmory = 5;
            this.gMapControl1.MarkersEnabled = true;
            this.gMapControl1.MaxZoom = 2;
            this.gMapControl1.MinZoom = 2;
            this.gMapControl1.MouseWheelZoomType = GMap.NET.MouseWheelZoomType.MousePositionAndCenter;
            this.gMapControl1.Name = "gMapControl1";
            this.gMapControl1.NegativeMode = false;
            this.gMapControl1.PolygonsEnabled = true;
            this.gMapControl1.RetryLoadTile = 0;
            this.gMapControl1.RoutesEnabled = true;
            this.gMapControl1.ShowTileGridLines = false;
            this.gMapControl1.streamjpg = ((System.IO.MemoryStream)(resources.GetObject("gMapControl1.streamjpg")));
            this.gMapControl1.Zoom = 0D;
            this.gMapControl1.Click += new System.EventHandler(this.gMapControl1_Click);
            this.gMapControl1.MouseDown += new System.Windows.Forms.MouseEventHandler(this.gMapControl1_MouseDown);
            this.gMapControl1.MouseMove += new System.Windows.Forms.MouseEventHandler(this.gMapControl1_MouseMove);
            // 
            // panel1
            // 
            this.panel1.Controls.Add(this.TXT_lat);
            this.panel1.Controls.Add(this.Zoomlevel);
            this.panel1.Controls.Add(this.label1);
            this.panel1.Controls.Add(this.TXT_long);
            this.panel1.Controls.Add(this.TXT_alt);
            this.panel1.Controls.Add(this.CHK_autopan);
            this.panel1.Controls.Add(this.CB_tuning);
            resources.ApplyResources(this.panel1, "panel1");
            this.panel1.Name = "panel1";
            // 
            // TXT_lat
            // 
            resources.ApplyResources(this.TXT_lat, "TXT_lat");
            this.TXT_lat.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.bindingSource1, "lat", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, "Lat 0"));
            this.TXT_lat.Name = "TXT_lat";
            this.TXT_lat.resize = false;
            // 
            // Zoomlevel
            // 
            resources.ApplyResources(this.Zoomlevel, "Zoomlevel");
            this.Zoomlevel.DecimalPlaces = 1;
            this.Zoomlevel.Increment = new decimal(new int[] {
            5,
            0,
            0,
            65536});
            this.Zoomlevel.Maximum = new decimal(new int[] {
            18,
            0,
            0,
            0});
            this.Zoomlevel.Minimum = new decimal(new int[] {
            1,
            0,
            0,
            0});
            this.Zoomlevel.Name = "Zoomlevel";
            this.toolTip1.SetToolTip(this.Zoomlevel, resources.GetString("Zoomlevel.ToolTip"));
            this.Zoomlevel.Value = new decimal(new int[] {
            10,
            0,
            0,
            0});
            this.Zoomlevel.ValueChanged += new System.EventHandler(this.Zoomlevel_ValueChanged);
            // 
            // label1
            // 
            resources.ApplyResources(this.label1, "label1");
            this.label1.Name = "label1";
            this.label1.resize = false;
            // 
            // TXT_long
            // 
            resources.ApplyResources(this.TXT_long, "TXT_long");
            this.TXT_long.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.bindingSource1, "lng", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, "Lng 0"));
            this.TXT_long.Name = "TXT_long";
            this.TXT_long.resize = false;
            // 
            // TXT_alt
            // 
            resources.ApplyResources(this.TXT_alt, "TXT_alt");
            this.TXT_alt.DataBindings.Add(new System.Windows.Forms.Binding("Text", this.bindingSource1, "alt", true, System.Windows.Forms.DataSourceUpdateMode.OnValidation, "Alt 0"));
            this.TXT_alt.Name = "TXT_alt";
            this.TXT_alt.resize = false;
            // 
            // CHK_autopan
            // 
            resources.ApplyResources(this.CHK_autopan, "CHK_autopan");
            this.CHK_autopan.Checked = true;
            this.CHK_autopan.CheckState = System.Windows.Forms.CheckState.Checked;
            this.CHK_autopan.Name = "CHK_autopan";
            this.toolTip1.SetToolTip(this.CHK_autopan, resources.GetString("CHK_autopan.ToolTip"));
            this.CHK_autopan.UseVisualStyleBackColor = true;
            // 
            // CB_tuning
            // 
            resources.ApplyResources(this.CB_tuning, "CB_tuning");
            this.CB_tuning.Name = "CB_tuning";
            this.toolTip1.SetToolTip(this.CB_tuning, resources.GetString("CB_tuning.ToolTip"));
            this.CB_tuning.UseVisualStyleBackColor = true;
            this.CB_tuning.CheckedChanged += new System.EventHandler(this.CB_tuning_CheckedChanged);
            // 
            // dataGridViewImageColumn1
            // 
            dataGridViewCellStyle1.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            this.dataGridViewImageColumn1.DefaultCellStyle = dataGridViewCellStyle1;
            resources.ApplyResources(this.dataGridViewImageColumn1, "dataGridViewImageColumn1");
            this.dataGridViewImageColumn1.Image = global::ArdupilotMega.Properties.Resources.up;
            this.dataGridViewImageColumn1.ImageLayout = System.Windows.Forms.DataGridViewImageCellLayout.Stretch;
            this.dataGridViewImageColumn1.Name = "dataGridViewImageColumn1";
            // 
            // dataGridViewImageColumn2
            // 
            dataGridViewCellStyle2.Alignment = System.Windows.Forms.DataGridViewContentAlignment.MiddleCenter;
            this.dataGridViewImageColumn2.DefaultCellStyle = dataGridViewCellStyle2;
            resources.ApplyResources(this.dataGridViewImageColumn2, "dataGridViewImageColumn2");
            this.dataGridViewImageColumn2.Image = global::ArdupilotMega.Properties.Resources.down;
            this.dataGridViewImageColumn2.ImageLayout = System.Windows.Forms.DataGridViewImageCellLayout.Stretch;
            this.dataGridViewImageColumn2.Name = "dataGridViewImageColumn2";
            // 
            // ZedGraphTimer
            // 
            this.ZedGraphTimer.Tick += new System.EventHandler(this.timer1_Tick);
            // 
            // toolTip1
            // 
            this.toolTip1.BackColor = System.Drawing.Color.FromArgb(((int)(((byte)(205)))), ((int)(((byte)(226)))), ((int)(((byte)(150)))));
            this.toolTip1.ForeColor = System.Drawing.Color.FromArgb(((int)(((byte)(121)))), ((int)(((byte)(148)))), ((int)(((byte)(41)))));
            // 
            // label6
            // 
            resources.ApplyResources(this.label6, "label6");
            this.label6.Name = "label6";
            this.label6.resize = false;
            // 
            // FlightData
            // 
            resources.ApplyResources(this, "$this");
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.Controls.Add(this.MainH);
            this.Controls.Add(this.label6);
            this.MinimumSize = new System.Drawing.Size(1344, 567);
            this.Name = "FlightData";
            this.Load += new System.EventHandler(this.FlightData_Load);
            this.Resize += new System.EventHandler(this.FlightData_Resize);
            this.ParentChanged += new System.EventHandler(this.FlightData_ParentChanged);
            this.contextMenuStrip1.ResumeLayout(false);
            this.MainH.Panel1.ResumeLayout(false);
            this.MainH.Panel2.ResumeLayout(false);
            this.MainH.ResumeLayout(false);
            this.SubMainLeft.Panel1.ResumeLayout(false);
            this.SubMainLeft.Panel2.ResumeLayout(false);
            this.SubMainLeft.ResumeLayout(false);
            this.contextMenuStrip2.ResumeLayout(false);
            ((System.ComponentModel.ISupportInitialize)(this.bindingSource1)).EndInit();
            this.Tabs.ResumeLayout(false);
            this.tabActions.ResumeLayout(false);
            this.tabGauges.ResumeLayout(false);
            this.tabTLogs.ResumeLayout(false);
            this.tabTLogs.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.NUM_playbackspeed)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.tracklog)).EndInit();
            this.tabCDnR.ResumeLayout(false);
            this.tabCDnR.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Time)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Altitude)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Airspeed)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.CDnR_Heading)).EndInit();
            this.tabPage1.ResumeLayout(false);
            this.tabPage1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.trafficTimeToWpt)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.trafficSimTime)).EndInit();
            this.tableMap.ResumeLayout(false);
            this.splitContainer1.Panel1.ResumeLayout(false);
            this.splitContainer1.Panel2.ResumeLayout(false);
            this.splitContainer1.ResumeLayout(false);
            this.panel1.ResumeLayout(false);
            this.panel1.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.Zoomlevel)).EndInit();
            this.ResumeLayout(false);

        }

        private System.Windows.Forms.DataGridViewImageColumn dataGridViewImageColumn1;
        private System.Windows.Forms.DataGridViewImageColumn dataGridViewImageColumn2;
        private ArdupilotMega.MyLabel label6;
        private System.Windows.Forms.BindingSource bindingSource1;
        private System.Windows.Forms.Timer ZedGraphTimer;
        private System.Windows.Forms.SplitContainer MainH;
        private System.Windows.Forms.SplitContainer SubMainLeft;
        private System.Windows.Forms.ContextMenuStrip contextMenuStrip1;
        private System.Windows.Forms.ToolStripMenuItem goHereToolStripMenuItem;
        private hud.HUD hud1;
        private MyButton BUT_clear_track;
        private System.Windows.Forms.CheckBox CB_tuning;
        private MyButton BUT_RAWSensor;
        private MyButton BUTactiondo;
        private MyButton BUTrestartmission;
        private System.Windows.Forms.ComboBox CMB_action;
        private MyButton BUT_Homealt;
        private System.Windows.Forms.TrackBar tracklog;
        private MyButton BUT_playlog;
        private MyButton BUT_loadtelem;
        private AGaugeApp.AGauge Galt;
        private AGaugeApp.AGauge Gspeed;
        private AGaugeApp.AGauge Gvspeed;
#if SLV_ADDED
        private AGaugeApp.AGauge Grud;
        private AGaugeApp.AGauge Gaftcurrent;
        private AGaugeApp.AGauge Gfwdcurrent;
#endif
        private System.Windows.Forms.TableLayoutPanel tableMap;
        private System.Windows.Forms.Panel panel1;
        private ArdupilotMega.MyLabel TXT_lat;
        private System.Windows.Forms.NumericUpDown Zoomlevel;
        private ArdupilotMega.MyLabel label1;
        private ArdupilotMega.MyLabel TXT_long;
        private ArdupilotMega.MyLabel TXT_alt;
        private System.Windows.Forms.CheckBox CHK_autopan;
        private myGMAP gMapControl1;
        private ZedGraph.ZedGraphControl zg1;
        private System.Windows.Forms.TabControl Tabs;
        private System.Windows.Forms.TabPage tabGauges;
        private System.Windows.Forms.TabPage tabStatus;
        private System.Windows.Forms.TabPage tabActions;
        private System.Windows.Forms.TabPage tabTLogs;
        private System.Windows.Forms.ComboBox CMB_modes;
        private MyButton BUT_setmode;
        private System.Windows.Forms.ComboBox CMB_setwp;
        private MyButton BUT_setwp;
        private MyButton BUT_quickmanual;
        private MyButton BUT_quickrtl;
        private MyButton BUT_quickauto;
        private MyButton BUT_log2kml;
        private ArdupilotMega.MyLabel lbl_windvel;
        private ArdupilotMega.MyLabel lbl_winddir;
        private MyButton BUT_joystick;
        private System.Windows.Forms.ToolTip toolTip1;
        private System.Windows.Forms.NumericUpDown NUM_playbackspeed;
        private System.Windows.Forms.ContextMenuStrip contextMenuStrip2;
        private System.Windows.Forms.ToolStripMenuItem recordHudToAVIToolStripMenuItem;
        private System.Windows.Forms.ToolStripMenuItem stopRecordToolStripMenuItem;
        private MyLabel lbl_logpercent;
        private System.Windows.Forms.ToolStripMenuItem pointCameraHereToolStripMenuItem;
        private System.Windows.Forms.SplitContainer splitContainer1;
        private MyButton BUT_script;
        private AGaugeApp.AGauge Gthrottle;
        private AGaugeApp.AGauge Gheading;
        private System.Windows.Forms.TabPage tabCDnR;
        private System.Windows.Forms.Button CDnR_Send;
        private System.Windows.Forms.NumericUpDown CDnR_Time;
        private System.Windows.Forms.NumericUpDown CDnR_Altitude;
        private System.Windows.Forms.NumericUpDown CDnR_Airspeed;
        private System.Windows.Forms.NumericUpDown CDnR_Heading;
        private System.Windows.Forms.CheckBox CDnR_CheckTime;
        private System.Windows.Forms.CheckBox CDnR_CheckAltitude;
        private System.Windows.Forms.CheckBox CDnR_CheckAirspeed;
        private System.Windows.Forms.CheckBox CDnR_CheckHeading;
        private System.Windows.Forms.TabPage tabPage1;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.NumericUpDown trafficTimeToWpt;
        private System.Windows.Forms.NumericUpDown trafficSimTime;
        private System.Windows.Forms.Button simSendMessage;
        private AGaugeApp.AGauge GaccZ;
        private AGaugeApp.AGauge GaccY;
        private AGaugeApp.AGauge GaccX;
        private MyButton myButton2;
        private MyButton myButton1;
        private AGaugeApp.AGauge Grpm;
    }
}