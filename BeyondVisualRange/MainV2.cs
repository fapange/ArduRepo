﻿#define SLV_ADDED

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Configuration;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO.Ports;
using System.IO;
using System.Xml;
using System.Collections;
using System.Net;
using System.Text.RegularExpressions;
using System.Web;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Speech.Synthesis;
using System.Globalization;
using System.Threading;
using System.Net.Sockets;
using IronPython.Hosting;
using log4net;
using ArdupilotMega.Controls;
#if SLV_ADDED
    using GMap.NET;
#endif

namespace ArdupilotMega
{
    public partial class MainV2 : Form
    {
#if SLV_ADDED
        public class Aircraft
        {
            public string ID { get; set; }
            public float Lat { get; set; }
            public float Lng { get; set; }
            public float Alt { get; set; }
            public float Heading { get; set; }
            public float pdist { get; set; }
            public int ownship { get; set; }


            public Aircraft() { ID = "BLANK"; Lat = 0; Lng = 0; Alt = 0;  Heading = 0;}
            public Aircraft(string name, float lat, float lng, float alt, float head) { ID = name; Lat = lat; Lng = lng; Alt = alt; Heading = head;}
            public void Update(float lat, float lng, float alt, float head, int own)  { Lat = lat; Lng = lng; Alt = alt; Heading = head; ownship = own;}
        }
        public static bool useThrottleFit;
        public static bool useGrndSpd;
        public static bool wpt4D;
        public static String GIT_ID;
        public static bool b_traffic_alert = false;
        public static int selectedAircraft = 0;
        public static int numWpts = 0;
        public static Byte[] ID = new Byte[8];
        public static double Latitude;    //Degrees
        public static double Longitude;    //Degrees
        public static double Elevation;    //Feet
        public static double Vx;
        public static double Vy;
        public static double Vz;
        public static int Own;
        public static float maxThrottle;
        public static float minThrottle;
        public static List<Aircraft> trafficTable = new List<Aircraft>();
        public static IPEndPoint odometerIPEP = new IPEndPoint(IPAddress.Any, 5500);
        public static IPEndPoint trafficIPEP = new IPEndPoint(IPAddress.Any, 6700);
        public static Socket trafficSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        //public static Socket odometerSocket = new Socket(AddressFamily.InterNetwork, SocketType.Dgram, ProtocolType.Udp);
        public static int fpw;
        public static float missionDist = 0.0f;
        public static float tripTimer = 0; // { get; set; }
        public static float tripOdometer = 0.0f;
        public static float soc1 = 100.0f;
        public static float soc2 = 100.0f;
        public static float soc3 = 100.0f;
        public static float soc4 = 100.0f;
        public static List<float> timeTable = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> distTable = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> soc1Table = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> soc2Table = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> soc3Table = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> soc4Table = new List<float> { 0, 0, 0, 0, 0 };

        public static List<float> instSpeed = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgSpeed  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndSpeed = new List<float> { 0, 0, 0, 0, 0 };

        public static List<float> instCons1_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons1_t  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons1_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons2_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons2_t  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons2_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons3_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons3_t  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons3_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons4_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons4_t  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons4_t = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons1_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons1_d  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons1_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons2_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons2_d  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons2_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons3_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons3_d  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons3_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> instCons4_d = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> avgCons4_d  = new List<float> { 0, 0, 0, 0, 0 };
        public static List<float> trndCons4_d = new List<float> { 0, 0, 0, 0, 0 };

        public static PointLatLng home;
        public static float retHomeDist;
        public static float retHomeTime;
        public static float remDist;
        public static float etaTime;
        public static float eod1Time;
        public static float eod2Time;
        public static float eod3Time;
        public static float eod4Time;
        public static float eodTime;
        public static float eod1Dist;
        public static float eod2Dist;
        public static float eod3Dist;
        public static float eod4Dist;
        public static float eodDist;
        public static float abortDist;
        public static float abortTime;
        public static float abortHomeDist;
        public static float abortHomeTime;
        public static int segmentNo = 1;
        public static float maxDistLeft;
        public static float maxTimeLeft;
        public static PointLatLng maxLoc = new PointLatLng(0, 0);
        public static PointLatLng abortLoc = new PointLatLng(0, 0);

#endif
        private static readonly ILog log =
            LogManager.GetLogger(System.Reflection.MethodBase.GetCurrentMethod().DeclaringType);
        [DllImport("user32.dll")]
        public static extern int FindWindow(string szClass, string szTitle);
        [DllImport("user32.dll")]
        public static extern int ShowWindow(int Handle, int showState);

        const int SW_SHOWNORMAL = 1;
        const int SW_HIDE = 0;

        public static MAVLink comPort = new MAVLink();
        public static string comportname = "";
        public static Hashtable config = new Hashtable();
        public static bool givecomport = false;
        public static Firmwares APMFirmware = Firmwares.ArduPlane;
        public static bool MONO = false;

        public static bool speechenable = false;
        public static Speech talk = null;

        public static Joystick joystick = null;
        DateTime lastjoystick = DateTime.Now;

        public static WebCamService.Capture cam = null;

        public static CurrentState cs = new CurrentState();

        bool serialthread = false;

        TcpListener listener;

        DateTime heatbeatsend = DateTime.Now;

        public static List<System.Threading.Thread> threads = new List<System.Threading.Thread>();
        public static MainV2 instance = null;
        /*
         * "PITCH_KP",
"PITCH_KI",
"PITCH_LIM",

         */

        public enum Firmwares
        {
            ArduPlane,
            ArduCopter2,
        }

        //GCSViews.FlightData FlightData;
        GCSViews.FlightPlanner FlightPlanner;
        //GCSViews.Configuration Configuration;
        //GCSViews.Simulation Simulation;
        //GCSViews.Firmware Firmware;
        //GCSViews.Terminal Terminal;

        public MainV2()
        {
            Form splash = new Splash();
            splash.Show();

            string _version = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString();
            //string _repo = ((System.Reflection.AssemblyTrademarkAttribute)System.Reflection.Assembly.GetExecutingAssembly().GetCustomAttributes(true)[4]).Trademark;
            string _repo = "null";
            string _date = Application.ProductVersion;
            splash.Text = "Beyond Visual Range " + _version + " " + _repo + " " + _date + " By Michael Oborne";

            splash.Refresh();

            Application.DoEvents();

            instance = this;

            InitializeComponent();

            srtm.datadirectory = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "srtm";

            var t = Type.GetType("Mono.Runtime");
            MONO = (t != null);

            talk = new Speech();

            //talk.SpeakAsync("Welcome to Beyond Visual Range");

            MyRenderer.currentpressed = MenuFlightPlanner;

            MainMenu.Renderer = new MyRenderer();

            List<object> list = new List<object>();
            foreach (object obj in Enum.GetValues(typeof(Firmwares)))
            {
                TOOL_APMFirmware.Items.Add(obj);
            }

            if (TOOL_APMFirmware.Items.Count > 0)
                TOOL_APMFirmware.SelectedIndex = 0;

            this.Text = splash.Text;

            comPort.BaseStream.BaudRate = 115200;

            CMB_serialport.Items.AddRange(GetPortNames());
            CMB_serialport.Items.Add("TCP");
            CMB_serialport.Items.Add("UDP");
            if (CMB_serialport.Items.Count > 0)
            {
                CMB_baudrate.SelectedIndex = 7;
                CMB_serialport.SelectedIndex = 0;
            }

            splash.Refresh();
            Application.DoEvents();

            // set this before we reset it
            MainV2.config["NUM_tracklength"] = "200";

            xmlconfig(false);

            if (config.ContainsKey("language") && !string.IsNullOrEmpty((string)config["language"]))
                changelanguage(CultureInfoEx.GetCultureInfo((string)config["language"]));

            if (!MONO) // windows only
            {
                if (MainV2.config["showconsole"] != null && MainV2.config["showconsole"].ToString() == "True")
                {
                }
                else
                {
                    int win = FindWindow("ConsoleWindowClass", null);
                    ShowWindow(win, SW_HIDE); // hide window
                }
            }

            try
            {
                //FlightData = new GCSViews.FlightData();
                FlightPlanner = new GCSViews.FlightPlanner();
                //Configuration = new GCSViews.Configuration();
                //Simulation = new GCSViews.Simulation();
                //Firmware = new GCSViews.Firmware();
                //Terminal = new GCSViews.Terminal();

                // preload
                Python.CreateEngine();
            }
            catch (Exception e) { MessageBox.Show("A Major error has occured : " + e.ToString()); this.Close(); }

            //if (MainV2.config["CHK_GDIPlus"] != null)
            //    GCSViews.FlightData.myhud.UseOpenGL = !bool.Parse(MainV2.config["CHK_GDIPlus"].ToString());

            changeunits();

            try
            {
                if (config["MainLocX"] != null && config["MainLocY"] != null)
                {
                    this.StartPosition = FormStartPosition.Manual;
                    Point startpos = new Point(int.Parse(config["MainLocX"].ToString()), int.Parse(config["MainLocY"].ToString()));
                    this.Location = startpos;
                }

                if (config["MainHeight"] != null)
                    this.Height = int.Parse(config["MainHeight"].ToString());
                if (config["MainWidth"] != null)
                    this.Width = int.Parse(config["MainWidth"].ToString());
                if (config["MainMaximised"] != null)
                    this.WindowState = (FormWindowState)Enum.Parse(typeof(FormWindowState), config["MainMaximised"].ToString());

                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.rateattitude = byte.Parse(config["CMB_rateattitude"].ToString());
                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.rateposition = byte.Parse(config["CMB_rateposition"].ToString());
                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.ratestatus = byte.Parse(config["CMB_ratestatus"].ToString());
                if (config["CMB_rateattitude"] != null)
                    MainV2.cs.raterc = byte.Parse(config["CMB_raterc"].ToString());

                if (config["speechenable"] != null)
                    MainV2.speechenable = bool.Parse(config["speechenable"].ToString());

                //int fixme;
                /*
                MainV2.cs.rateattitude = 50;
                MainV2.cs.rateposition = 50;
                MainV2.cs.ratestatus = 50;
                MainV2.cs.raterc = 50;
                MainV2.cs.ratesensors = 50;
                */
                try
                {
                    if (config["TXT_homelat"] != null)
                        cs.HomeLocation.Lat = double.Parse(config["TXT_homelat"].ToString());

                    if (config["TXT_homelng"] != null)
                        cs.HomeLocation.Lng = double.Parse(config["TXT_homelng"].ToString());

                    if (config["TXT_homealt"] != null)
                        cs.HomeLocation.Alt = double.Parse(config["TXT_homealt"].ToString());
                }
                catch { }

            }
            catch { }

            if (cs.rateattitude == 0) // initilised to 10, configured above from save
            {
                MessageBox.Show("NOTE: your attitude rate is 0, the hud will not work\nChange in Configuration > Planner > Telemetry Rates");
            }


            //System.Threading.Thread.Sleep(2000);

            // make sure new enough .net framework is installed
            if (!MONO)
            {
                Microsoft.Win32.RegistryKey installed_versions = Microsoft.Win32.Registry.LocalMachine.OpenSubKey(@"SOFTWARE\Microsoft\NET Framework Setup\NDP");
                string[] version_names = installed_versions.GetSubKeyNames();
                //version names start with 'v', eg, 'v3.5' which needs to be trimmed off before conversion
                double Framework = Convert.ToDouble(version_names[version_names.Length - 1].Remove(0, 1), CultureInfo.InvariantCulture);
                int SP = Convert.ToInt32(installed_versions.OpenSubKey(version_names[version_names.Length - 1]).GetValue("SP", 0));

                if (Framework < 3.5)
                {
                    MessageBox.Show("This program requires .NET Framework 3.5. You currently have " + Framework);
                }
            }
            {
                MyView.Controls.Clear();

                //GCSViews.Terminal.threadrun = false;

                UserControl temp = FlightPlanner;

                ThemeManager.ApplyThemeTo(temp);

                temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

                temp.Location = new Point(0, MainMenu.Height);

                temp.Dock = DockStyle.Fill;

                MyView.Controls.Add(temp);
            }
            Application.DoEvents();

            splash.Close();
        }

        private string[] GetPortNames()
        {
            string[] devs = new string[0];


            log.Debug("Geting Comports");

            if (MONO)
            {
                if (Directory.Exists("/dev/"))
                {
                    if (Directory.Exists("/dev/serial/by-id/"))
                        devs = Directory.GetFiles("/dev/serial/by-id/", "*");
                    devs = Directory.GetFiles("/dev/", "*ACM*");
                    devs = Directory.GetFiles("/dev/", "ttyUSB*");
                }
            }

            string[] ports = SerialPort.GetPortNames();

            for (int a = 0; a < ports.Length; a++)
            {
                ports[a] = ports[a].TrimEnd();
            }

            string[] all = new string[devs.Length + ports.Length];

            devs.CopyTo(all, 0);
            ports.CopyTo(all, devs.Length);

            return all;
        }

        internal void ScreenShot()
        {
            Rectangle bounds = Screen.GetBounds(Point.Empty);
            using (Bitmap bitmap = new Bitmap(bounds.Width, bounds.Height))
            {
                using (Graphics g = Graphics.FromImage(bitmap))
                {
                    g.CopyFromScreen(Point.Empty, Point.Empty, bounds.Size);
                }
                string name = "ss" + DateTime.Now.ToString("hhmmss") + ".jpg";
                bitmap.Save(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + name, System.Drawing.Imaging.ImageFormat.Jpeg);
                MessageBox.Show("Screenshot saved to " + name);
            }

        }

        private void CMB_serialport_Click(object sender, EventArgs e)
        {
            string oldport = CMB_serialport.Text;
            CMB_serialport.Items.Clear();
            CMB_serialport.Items.AddRange(GetPortNames());
            CMB_serialport.Items.Add("TCP");
            CMB_serialport.Items.Add("UDP");
            if (CMB_serialport.Items.Contains(oldport))
                CMB_serialport.Text = oldport;
        }


        private void MenuFlightData_Click(object sender, EventArgs e)
        {
            /*MyView.Controls.Clear();

            //GCSViews.Terminal.threadrun = false;

            UserControl temp = FlightData;

            ThemeManager.ApplyThemeTo(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, MainMenu.Height);

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);

            if (MainV2.config["FlightSplitter"] != null)
                ((GCSViews.FlightData)temp).MainHcopy.SplitterDistance = int.Parse(MainV2.config["FlightSplitter"].ToString());*/
        }

        private void MenuFlightPlanner_Click(object sender, EventArgs e)
        {
            MyView.Controls.Clear();

            //GCSViews.Terminal.threadrun = false;

            UserControl temp = FlightPlanner;

            ThemeManager.ApplyThemeTo(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, MainMenu.Height);

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);
        }

        private void MenuConfiguration_Click(object sender, EventArgs e)
        {
            /*MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            // dispose of old else memory leak
            if (Configuration != null)
            {
                try
                {
                    Configuration.Dispose();
                }
                catch { }
            }

            Configuration = new GCSViews.Configuration();

            UserControl temp = Configuration;

            ThemeManager.ApplyThemeTo(temp);

            //temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, 0);

            temp.Dock = DockStyle.Fill;

            temp.Size = MyView.Size;

            //temp.Parent = MyView;

            MyView.Controls.Add(temp);*/
        }

        private void MenuSimulation_Click(object sender, EventArgs e)
        {
            /*MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            UserControl temp = Simulation;

            ThemeManager.ApplyThemeTo(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Location = new Point(0, MainMenu.Height);

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);*/
        }

        private void MenuFirmware_Click(object sender, EventArgs e)
        {
            /*MyView.Controls.Clear();

            GCSViews.Terminal.threadrun = false;

            UserControl temp = Firmware;

            ThemeManager.ApplyThemeTo(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);*/
        }

        private void MenuTerminal_Click(object sender, EventArgs e)
        {
            /*if (comPort.BaseStream.IsOpen)
            {
                MenuConnect_Click(sender, e);
            }

            givecomport = true;

            MyView.Controls.Clear();

            this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.disconnect;

            // dispose of old else memory leak
            if (Terminal != null)
            {
                try
                {
                    Terminal.Dispose();
                }
                catch { }
            }

            Terminal = new GCSViews.Terminal();

            UserControl temp = Terminal;

            ThemeManager.ApplyThemeTo(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);*/

        }

        private void MenuConnect_Click(object sender, EventArgs e)
        {
            givecomport = false;

            if (comPort.BaseStream.IsOpen && cs.groundspeed > 4)
            {
                if (DialogResult.No == MessageBox.Show("Your model is still moving are you sure you want to disconnect?", "Disconnect", MessageBoxButtons.YesNo))
                {
                    return;
                }
            }

            if (comPort.BaseStream.IsOpen)
            {
                try
                {
                    try
                    {
                        if (talk != null) // cancel all pending speech
                            talk.SpeakAsyncCancelAll();
                    }
                    catch { }

                    if (comPort.logfile != null)
                        comPort.logfile.Close();

                    comPort.BaseStream.DtrEnable = false;
                    comPort.Close();
                }
                catch (Exception ex) { log.Debug(ex.ToString()); }

                this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.connect;
            }
            else
            {
                if (CMB_serialport.Text == "TCP")
                {
                    comPort.BaseStream = new TcpSerial();
                }
                else
                    if (CMB_serialport.Text == "UDP")
                    {
                        comPort.BaseStream = new UdpSerial();
                    }
                    else
                    {
                        comPort.BaseStream = new SerialPort();
                    }
                try
                {
                    comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);
                }
                catch { }
                comPort.BaseStream.DataBits = 8;
                comPort.BaseStream.StopBits = (StopBits)Enum.Parse(typeof(StopBits), "1");
                comPort.BaseStream.Parity = (Parity)Enum.Parse(typeof(Parity), "None");

                comPort.BaseStream.DtrEnable = false;

                if (config["CHK_resetapmonconnect"] == null || bool.Parse(config["CHK_resetapmonconnect"].ToString()) == true)
                    comPort.BaseStream.toggleDTR();

                try
                {
                    if (comPort.logfile != null)
                        comPort.logfile.Close();
                    try
                    {
                        Directory.CreateDirectory(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs");
                        comPort.logfile = new BinaryWriter(File.Open(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"logs" + Path.DirectorySeparatorChar + DateTime.Now.ToString("yyyy-MM-dd hh-mm-ss") + ".tlog", FileMode.CreateNew));
                    }
                    catch { MessageBox.Show("Failed to create log - wont log this session"); } // soft fail

                    comPort.BaseStream.PortName = CMB_serialport.Text;
                    comPort.Open(true);

#if SLV_ADDED       // Initializes values upon connecting to ArduPilot
                    if (comPort.param["RC3_MIN"] != null)
                        MainV2.minThrottle = float.Parse(comPort.param["RC3_MIN"].ToString());

                    if (comPort.param["RC3_MAX"] != null)
                        MainV2.maxThrottle = float.Parse(comPort.param["RC3_MAX"].ToString());
#endif

                    if (comPort.param["SYSID_SW_TYPE"] != null)
                    {
                        if (float.Parse(comPort.param["SYSID_SW_TYPE"].ToString()) == 10)
                        {
                            TOOL_APMFirmware.SelectedIndex = TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduCopter2);
                        }
                        else if (float.Parse(comPort.param["SYSID_SW_TYPE"].ToString()) == 0)
                        {
                            TOOL_APMFirmware.SelectedIndex = TOOL_APMFirmware.Items.IndexOf(Firmwares.ArduPlane);
                        }
                    }

                    cs.firmware = APMFirmware;

                    config[CMB_serialport.Text + "_BAUD"] = CMB_baudrate.Text;

                    if (config["loadwpsonconnect"] != null && bool.Parse(config["loadwpsonconnect"].ToString()) == true)
                    {
                        MenuFlightPlanner_Click(null, null);
                        FlightPlanner.BUT_read_Click(null, null);
                    }

                    this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.disconnect;
                }
                catch (Exception ex)
                {
                    try
                    {
                        comPort.Close();
                    }
                    catch { }
                    try
                    {
                        string version = ArduinoDetect.DetectVersion(comPort.BaseStream.PortName);
                        ArduinoComms port = new ArduinoSTK();
                        if (version == "1280")
                        {
                            port = new ArduinoSTK();
                            port.BaudRate = 57600;
                        }
                        else if (version == "2560")
                        {
                            port = new ArduinoSTKv2();
                            port.BaudRate = 115200;
                        }
                        else { throw new Exception("Can not determine APM board type"); }
                        port.PortName = comPort.BaseStream.PortName;
                        port.DtrEnable = true;
                        port.Open();
                        if (port.connectAP())
                        {
                            byte[] buffer = port.download(20);
                            port.Close();

                            if ((buffer[0] == 'A' || buffer[0] == 'P') && (buffer[1] == 'A' || buffer[1] == 'P')) // this is the apvar header
                            {
                                log.Info("Valid eeprom contents");
                            }
                            else
                            {
                                MessageBox.Show("You dont appear to have uploaded a firmware yet,\n\nPlease goto the firmware page and upload one.");
                                return;
                            }
                        }
                    }
                    catch { }
                    log.Debug(ex.ToString());
                    //MessageBox.Show("Can not establish a connection\n\n" + ex.ToString());
                    return;
                }
            }
        }

        private void CMB_serialport_SelectedIndexChanged(object sender, EventArgs e)
        {
            comportname = CMB_serialport.Text;
            if (comportname == "UDP" || comportname == "TCP")
            {
                CMB_baudrate.Enabled = false;
                if (comportname == "TCP")
                    MainV2.comPort.BaseStream = new TcpSerial();
                if (comportname == "UDP")
                    MainV2.comPort.BaseStream = new UdpSerial();
            }
            else
            {
                CMB_baudrate.Enabled = true;
                MainV2.comPort.BaseStream = new ArdupilotMega.SerialPort();
            }

            try
            {
                comPort.BaseStream.PortName = CMB_serialport.Text;

                MainV2.comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);

                if (config[CMB_serialport.Text + "_BAUD"] != null)
                {
                    CMB_baudrate.Text = config[CMB_serialport.Text + "_BAUD"].ToString();
                }
            }
            catch { }
        }

        private void toolStripMenuItem2_Click(object sender, EventArgs e)
        {
            //Form temp = new Main();
            //temp.Show();
        }

        private void MainV2_FormClosed(object sender, FormClosedEventArgs e)
        {
            //GCSViews.FlightData.threadrun = 0;
            //GCSViews.Simulation.threadrun = 0;

            serialthread = false;

            try
            {
                if (comPort.BaseStream.IsOpen)
                    comPort.Close();
            }
            catch { } // i get alot of these errors, the port is still open, but not valid - user has unpluged usb
            /*try
            {
                FlightData.Dispose();
            }
            catch { }*/
            try
            {
                FlightPlanner.Dispose();
            }
            catch { }
            /*try
            {
                Simulation.Dispose();
            }
            catch { }*/

            xmlconfig(true);
        }


        private void xmlconfig(bool write)
        {
            if (write || !File.Exists(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml"))
            {
                try
                {
                    //System.Configuration.Configuration appconfig = System.Configuration.ConfigurationManager.OpenExeConfiguration(System.Configuration.ConfigurationUserLevel.None);

                    XmlTextWriter xmlwriter = new XmlTextWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml", Encoding.ASCII);
                    xmlwriter.Formatting = Formatting.Indented;

                    xmlwriter.WriteStartDocument();

                    xmlwriter.WriteStartElement("Config");

                    xmlwriter.WriteElementString("comport", comportname);

                    xmlwriter.WriteElementString("baudrate", CMB_baudrate.Text);

                    xmlwriter.WriteElementString("APMFirmware", APMFirmware.ToString());

                    //appconfig.AppSettings.Settings.Add("comport", comportname);
                    //appconfig.AppSettings.Settings.Add("baudrate", CMB_baudrate.Text);
                    //appconfig.AppSettings.Settings.Add("APMFirmware", APMFirmware.ToString());

                    foreach (string key in config.Keys)
                    {
                        try
                        {
                            if (key == "" || key.Contains("/")) // "/dev/blah"
                                continue;
                            xmlwriter.WriteElementString(key, config[key].ToString());

                            //appconfig.AppSettings.Settings.Add(key, config[key].ToString());
                        }
                        catch { }
                    }

                    xmlwriter.WriteEndElement();

                    xmlwriter.WriteEndDocument();
                    xmlwriter.Close();

                    //appconfig.Save();
                }
                catch (Exception ex) { MessageBox.Show(ex.ToString()); }
            }
            else
            {
                try
                {
                    using (XmlTextReader xmlreader = new XmlTextReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml"))
                    {
                        while (xmlreader.Read())
                        {
                            xmlreader.MoveToElement();
                            try
                            {
                                switch (xmlreader.Name)
                                {
                                    case "comport":
                                        string temp = xmlreader.ReadString();

                                        CMB_serialport.SelectedIndex = CMB_serialport.FindString(temp);
                                        if (CMB_serialport.SelectedIndex == -1)
                                        {
                                            CMB_serialport.Text = temp; // allows ports that dont exist - yet
                                        }
                                        comPort.BaseStream.PortName = temp;
                                        comportname = temp;
                                        break;
                                    case "baudrate":
                                        string temp2 = xmlreader.ReadString();

                                        CMB_baudrate.SelectedIndex = CMB_baudrate.FindString(temp2);
                                        if (CMB_baudrate.SelectedIndex == -1)
                                        {
                                            CMB_baudrate.Text = temp2;
                                            //CMB_baudrate.SelectedIndex = CMB_baudrate.FindString("57600"); ; // must exist
                                        }
                                        //bau = int.Parse(CMB_baudrate.Text);
                                        break;
                                    case "APMFirmware":
                                        string temp3 = xmlreader.ReadString();
                                        TOOL_APMFirmware.SelectedIndex = TOOL_APMFirmware.FindStringExact(temp3);
                                        if (TOOL_APMFirmware.SelectedIndex == -1)
                                            TOOL_APMFirmware.SelectedIndex = 0;
                                        APMFirmware = (MainV2.Firmwares)Enum.Parse(typeof(MainV2.Firmwares), TOOL_APMFirmware.Text);
                                        break;
                                    case "Config":
                                        break;
                                    case "xml":
                                        break;
                                    default:
                                        if (xmlreader.Name == "") // line feeds
                                            break;
                                        config[xmlreader.Name] = xmlreader.ReadString();
                                        break;
                                }
                            }
                            catch (Exception ee) { log.Info(ee.Message); } // silent fail on bad entry
                        }
                    }
                }
                catch (Exception ex) { log.Info("Bad Config File: " + ex.ToString()); } // bad config file
            }
        }

        private void CMB_baudrate_SelectedIndexChanged(object sender, EventArgs e)
        {
            try
            {
                comPort.BaseStream.BaudRate = int.Parse(CMB_baudrate.Text);
            }
            catch { }
        }

        private void joysticksend()
        {
            while (true)
            {
                try
                {
                    if (!MONO)
                    {
                        //joystick stuff

                        if (joystick != null && joystick.enabled)
                        {
                            MAVLink.__mavlink_rc_channels_override_t rc = new MAVLink.__mavlink_rc_channels_override_t();

                            rc.target_component = comPort.compid;
                            rc.target_system = comPort.sysid;

                            if (joystick.getJoystickAxis(1) != Joystick.joystickaxis.None)
                                rc.chan1_raw = cs.rcoverridech1;//(ushort)(((int)state.Rz / 65.535) + 1000);
                            if (joystick.getJoystickAxis(2) != Joystick.joystickaxis.None)
                                rc.chan2_raw = cs.rcoverridech2;//(ushort)(((int)state.Y / 65.535) + 1000);
                            if (joystick.getJoystickAxis(3) != Joystick.joystickaxis.None)
                                rc.chan3_raw = cs.rcoverridech3;//(ushort)(1000 - ((int)slider[0] / 65.535 ) + 1000);
                            if (joystick.getJoystickAxis(4) != Joystick.joystickaxis.None)
                                rc.chan4_raw = cs.rcoverridech4;//(ushort)(((int)state.X / 65.535) + 1000);
                            if (joystick.getJoystickAxis(5) != Joystick.joystickaxis.None)
                                rc.chan5_raw = cs.rcoverridech5;
                            if (joystick.getJoystickAxis(6) != Joystick.joystickaxis.None)
                                rc.chan6_raw = cs.rcoverridech6;
                            if (joystick.getJoystickAxis(7) != Joystick.joystickaxis.None)
                                rc.chan7_raw = cs.rcoverridech7;
                            if (joystick.getJoystickAxis(8) != Joystick.joystickaxis.None)
                                rc.chan8_raw = cs.rcoverridech8;

                            if (lastjoystick.AddMilliseconds(50) < DateTime.Now)
                            {
                                //                                Console.WriteLine(DateTime.Now.Millisecond + " {0} {1} {2} {3} ", rc.chan1_raw, rc.chan2_raw, rc.chan3_raw, rc.chan4_raw);
                                comPort.sendPacket(rc);
                                lastjoystick = DateTime.Now;
                            }

                        }
                    }
                    System.Threading.Thread.Sleep(50);
                }
                catch { } // cant fall out
            }
        }




        private void SerialReader()
        {
            if (serialthread == true)
                return;
            serialthread = true;

            int minbytes = 10;

            if (MONO)
                minbytes = 0;

            DateTime menuupdate = DateTime.Now;

            DateTime speechcustomtime = DateTime.Now;

            DateTime linkqualitytime = DateTime.Now;

            while (serialthread)
            {
                try
                {
                    System.Threading.Thread.Sleep(5);
                    if ((DateTime.Now - menuupdate).Milliseconds > 500)
                    {
                        //                        Console.WriteLine(DateTime.Now.Millisecond);
                        if (comPort.BaseStream.IsOpen)
                        {
                            if ((string)this.MenuConnect.BackgroundImage.Tag != "Disconnect")
                            {
                                this.Invoke((MethodInvoker)delegate
                                {
                                    this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.disconnect;
                                    this.MenuConnect.BackgroundImage.Tag = "Disconnect";
                                    CMB_baudrate.Enabled = false;
                                    CMB_serialport.Enabled = false;
                                });
                            }
                        }
                        else
                        {
                            if ((string)this.MenuConnect.BackgroundImage.Tag != "Connect")
                            {
                                this.Invoke((MethodInvoker)delegate
                                {
                                    this.MenuConnect.BackgroundImage = global::ArdupilotMega.Properties.Resources.connect;
                                    this.MenuConnect.BackgroundImage.Tag = "Connect";
                                    CMB_baudrate.Enabled = true;
                                    CMB_serialport.Enabled = true;
                                });
                            }
                        }
                        menuupdate = DateTime.Now;
                    }

                    if (speechenable && talk != null && (DateTime.Now - speechcustomtime).TotalSeconds > 30 && MainV2.cs.lat != 0 && (MainV2.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        //speechbatteryvolt
                        float warnvolt = 0;
                        float.TryParse(MainV2.getConfig("speechbatteryvolt"), out warnvolt);

                        if (MainV2.getConfig("speechbatteryenabled") == "True" && MainV2.cs.battery_voltage <= warnvolt)
                        {
                            if (!MainV2.b_traffic_alert)
                                MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechbattery")));
                        }

                        if (MainV2.getConfig("speechcustomenabled") == "True")
                        {
                            if (!MainV2.b_traffic_alert)
                                MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechcustom")));
                        }

                        speechcustomtime = DateTime.Now;
                    }

                    if ((DateTime.Now - comPort.lastvalidpacket).TotalSeconds > 10)
                    {
                        MainV2.cs.linkqualitygcs = 0;
                    }

                    /*if ((DateTime.Now - comPort.lastvalidpacket).TotalSeconds >= 1)
                    {
                        if (linkqualitytime.Second != DateTime.Now.Second)
                        {
                            MainV2.cs.linkqualitygcs = (ushort)(MainV2.cs.linkqualitygcs * 0.8f);
                            linkqualitytime = DateTime.Now;

                            GCSViews.FlightData.myhud.Invalidate();
                        }

                        GC.Collect();
                    }*/

                    if (speechenable && talk != null && (MainV2.comPort.logreadmode || comPort.BaseStream.IsOpen))
                    {
                        float warnalt = float.MaxValue;
                        float.TryParse(MainV2.getConfig("speechaltheight"), out warnalt);
                        try
                        {
                            if (MainV2.getConfig("speechaltenabled") == "True" && (MainV2.cs.alt - (int)double.Parse(MainV2.getConfig("TXT_homealt"))) <= warnalt)
                            {
                                if (MainV2.talk.State == SynthesizerState.Ready)
                                    if (!MainV2.b_traffic_alert)
                                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechalt")));
                            }
                        }
                        catch { } // silent fail
                    }

                    if (!comPort.BaseStream.IsOpen || givecomport == true)
                    {
                        System.Threading.Thread.Sleep(100);
                        continue;
                    }

                    if (heatbeatsend.Second != DateTime.Now.Second)
                    {
                        //                        Console.WriteLine("remote lost {0}", cs.packetdropremote);

                        MAVLink.__mavlink_heartbeat_t htb = new MAVLink.__mavlink_heartbeat_t();

#if MAVLINK10
                        htb.type = (byte)MAVLink.MAV_TYPE.MAV_TYPE_GCS;
                        htb.autopilot = (byte)MAVLink.MAV_AUTOPILOT.MAV_AUTOPILOT_ARDUPILOTMEGA;
                        htb.mavlink_version = 3;
#else
                        // slv - Ground Station also sends out selected Aircraft as type
                        htb.type = (byte)MAVLink.MAV_TYPE.MAV_GENERIC;
                        htb.autopilot = (byte)MAVLink.MAV_AUTOPILOT_TYPE.MAV_AUTOPILOT_ARDUPILOTMEGA;
                        htb.mavlink_version = 2;
                        htb.vehicleID = (byte)selectedAircraft;
                        htb.numWpts = (byte)numWpts;
#endif

                        comPort.sendPacket(htb);
                        heatbeatsend = DateTime.Now;
                    }

                    // data loss warning
                    if ((DateTime.Now - comPort.lastvalidpacket).TotalSeconds > 10)
                    {
                        if (speechenable && talk != null)
                        {
                            if (MainV2.talk.State == SynthesizerState.Ready)
                                if (!MainV2.b_traffic_alert)
                                    MainV2.talk.SpeakAsync("WARNING No Data for " + (int)(DateTime.Now - comPort.lastvalidpacket).TotalSeconds + " Seconds");
                        }
                    }

                    //Console.WriteLine(comPort.BaseStream.BytesToRead);

                    while (comPort.BaseStream.BytesToRead > minbytes && givecomport == false)
                        comPort.readPacket();
                }
                catch (Exception e)
                {
                    log.Info("Serial Reader fail :" + e.Message);
                    try
                    {
                        comPort.Close();
                    }
                    catch { }
                }
            }
        }

        private class MyRenderer : ToolStripProfessionalRenderer
        {
            public static ToolStripItem currentpressed;
            protected override void OnRenderButtonBackground(ToolStripItemRenderEventArgs e)
            {
                //BackgroundImage
                if (e.Item.BackgroundImage == null) base.OnRenderButtonBackground(e);
                else
                {
                    Rectangle bounds = new Rectangle(Point.Empty, e.Item.Size);
                    e.Graphics.DrawImage(e.Item.BackgroundImage, bounds);
                    if (e.Item.Pressed || e.Item == currentpressed)
                    {
                        SolidBrush brush = new SolidBrush(Color.FromArgb(73, 0x2b, 0x3a, 0x03));
                        e.Graphics.FillRectangle(brush, bounds);
                        if (e.Item.Name != "MenuConnect")
                        {
                            //Console.WriteLine("new " + e.Item.Name + " old " + currentpressed.Name );
                            //e.Item.GetCurrentParent().Invalidate();
                            if (currentpressed != e.Item)
                                currentpressed.Invalidate();
                            currentpressed = e.Item;
                        }

                        // Something...
                    }
                    else if (e.Item.Selected) // mouse over
                    {
                        SolidBrush brush = new SolidBrush(Color.FromArgb(73, 0x2b, 0x3a, 0x03));
                        e.Graphics.FillRectangle(brush, bounds);
                        // Something...
                    }
                    using (Pen pen = new Pen(Color.Black))
                    {
                        //e.Graphics.DrawRectangle(pen, bounds.X, bounds.Y, bounds.Width - 1, bounds.Height - 1);
                    }
                }
            }

            protected override void OnRenderItemImage(ToolStripItemImageRenderEventArgs e)
            {
                //base.OnRenderItemImage(e);
            }
        }

        private void MainV2_Load(object sender, EventArgs e)
        {
            // generate requestall for jani and sandro
            comPort.sysid = 7;
            comPort.compid = 1;
            //comPort.requestDatastream((byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_ALL,3);
            //

            MenuFlightData_Click(sender, e);

            try
            {
                listener = new TcpListener(IPAddress.Any, 56781);
                var t13 = new Thread(listernforclients)
                {
                    Name = "motion jpg stream",
                    IsBackground = true
                };
                // wait for tcp connections               
                t13.Start();
            }
            catch (Exception ex)
            {
                log.Error("Error starting TCP listener thread: ", ex);
                MessageBox.Show(ex.ToString());
            }

            var t12 = new Thread(new ThreadStart(joysticksend))
            {
                IsBackground = true,
                Priority = ThreadPriority.AboveNormal,
                Name = "Main joystick sender"
            };
            t12.Start();

            var t11 = new Thread(SerialReader)
            {
                IsBackground = true,
                Name = "Main Serial reader"
            };
            t11.Start();

            if (Debugger.IsAttached)
            {
                log.Info("Skipping update test as it appears we are debugging");
            }
            else
            {
#if SLV_ADDED
#else
                try
                {
                    CheckForUpdate();
                }
                catch (Exception ex)
                {
                    log.Error("Update check failed", ex);
                }
#endif
            }
        }

        public static String ComputeWebSocketHandshakeSecurityHash09(String secWebSocketKey)
        {
            const String MagicKEY = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
            String secWebSocketAccept = String.Empty;

            // 1. Combine the request Sec-WebSocket-Key with magic key.
            String ret = secWebSocketKey + MagicKEY;

            // 2. Compute the SHA1 hash
            System.Security.Cryptography.SHA1 sha = new System.Security.Cryptography.SHA1CryptoServiceProvider();
            byte[] sha1Hash = sha.ComputeHash(Encoding.UTF8.GetBytes(ret));

            // 3. Base64 encode the hash
            secWebSocketAccept = Convert.ToBase64String(sha1Hash);

            return secWebSocketAccept;
        }

        /// <summary>          
        /// little web server for sending network link kml's          
        /// </summary>          

        void listernforclients()
        {
            try
            {
                listener.Start();
            }
            catch { log.Info("do you have the planner open already"); return; } // in use
            // Enter the listening loop.               
            while (true)
            {
                // Perform a blocking call to accept requests.           
                // You could also user server.AcceptSocket() here.               
                try
                {
                    log.Info("Listening for client - 1 client at a time");
                    TcpClient client = listener.AcceptTcpClient();
                    // Get a stream object for reading and writing          
                    log.Info("Accepted Client " + client.Client.RemoteEndPoint.ToString());
                    //client.SendBufferSize = 100 * 1024; // 100kb
                    //client.LingerState.Enabled = true;
                    //client.NoDelay = true;

                    // makesure we have valid image
                    //GCSViews.FlightData.mymap.streamjpgenable = true;
                    //GCSViews.FlightData.myhud.streamjpgenable = true;

                    MethodInvoker m = delegate()
                    {
                        //GCSViews.FlightData.mymap.Refresh();
                    };
                    this.Invoke(m);

                    NetworkStream stream = client.GetStream();

                    System.Text.ASCIIEncoding encoding = new System.Text.ASCIIEncoding();

                    byte[] request = new byte[1024];

                    int len = stream.Read(request, 0, request.Length);
                    string head = System.Text.ASCIIEncoding.ASCII.GetString(request, 0, len);
                    log.Info(head);

                    int index = head.IndexOf('\n');

                    string url = head.Substring(0, index - 1);
                    //url = url.Replace("\r", "");
                    //url = url.Replace("GET ","");
                    //url = url.Replace(" HTTP/1.0", "");
                    //url = url.Replace(" HTTP/1.1", "");

                    if (url.Contains("websocket"))
                    {
                        using (var writer = new StreamWriter(stream, Encoding.Default))
                        {
                            writer.WriteLine("HTTP/1.1 101 WebSocket Protocol Handshake");
                            writer.WriteLine("Upgrade: WebSocket");
                            writer.WriteLine("Connection: Upgrade");
                            writer.WriteLine("WebSocket-Location: ws://localhost:56781/websocket/server");

                            int start = head.IndexOf("Sec-WebSocket-Key:") + 19;
                            int end = head.IndexOf('\r', start);
                            if (end == -1)
                                end = head.IndexOf('\n', start);
                            string accept = ComputeWebSocketHandshakeSecurityHash09(head.Substring(start, end - start));

                            writer.WriteLine("Sec-WebSocket-Accept: " + accept);

                            writer.WriteLine("Server: Beyond Visual Range");

                            writer.WriteLine("");

                            writer.Flush();

                            while (client.Connected)
                            {
                                System.Threading.Thread.Sleep(200);
                                log.Info(stream.DataAvailable + " " + client.Available);

                                while (client.Available > 0)
                                {
                                    Console.Write(stream.ReadByte());
                                }

                                byte[] packet = new byte[256];

                                string sendme = cs.roll + "," + cs.pitch + "," + cs.yaw;

                                packet[0] = 0x81; // fin - binary
                                packet[1] = (byte)sendme.Length;

                                int i = 2;
                                foreach (char ch in sendme)
                                {
                                    packet[i++] = (byte)ch;
                                }

                                stream.Write(packet, 0, i);

                                //break;
                            }
                        }
                    }
                    else if (url.Contains("network.kml"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: application/vnd.google-earth.kml+xml\n\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        SharpKml.Dom.Document kml = new SharpKml.Dom.Document();

                        SharpKml.Dom.Placemark pmplane = new SharpKml.Dom.Placemark();
                        pmplane.Name = "P/Q ";

                        pmplane.Visibility = true;

                        SharpKml.Dom.Location loc = new SharpKml.Dom.Location();
                        loc.Latitude = cs.lat;
                        loc.Longitude = cs.lng;
                        loc.Altitude = cs.alt;

                        if (loc.Altitude < 0)
                            loc.Altitude = 0.01;

                        SharpKml.Dom.Orientation ori = new SharpKml.Dom.Orientation();
                        ori.Heading = cs.yaw;
                        ori.Roll = -cs.roll;
                        ori.Tilt = -cs.pitch;

                        SharpKml.Dom.Scale sca = new SharpKml.Dom.Scale();

                        sca.X = 2;
                        sca.Y = 2;
                        sca.Z = 2;

                        SharpKml.Dom.Model model = new SharpKml.Dom.Model();
                        model.Location = loc;
                        model.Orientation = ori;
                        model.AltitudeMode = SharpKml.Dom.AltitudeMode.Absolute;
                        model.Scale = sca;

                        SharpKml.Dom.Link link = new SharpKml.Dom.Link();
                        link.Href = new Uri("block_plane_0.dae", UriKind.Relative);

                        model.Link = link;

                        pmplane.Geometry = model;

                        SharpKml.Dom.LookAt la = new SharpKml.Dom.LookAt()
                        {
                            Altitude = loc.Altitude.Value,
                            Latitude = loc.Latitude.Value,
                            Longitude = loc.Longitude.Value,
                            Tilt = 80,
                            Heading = cs.yaw,
                            AltitudeMode = SharpKml.Dom.AltitudeMode.Absolute,
                            Range = 50
                        };

                        kml.Viewpoint = la;

                        kml.AddFeature(pmplane);

                        SharpKml.Base.Serializer serializer = new SharpKml.Base.Serializer();
                        serializer.Serialize(kml);

                        byte[] buffer = Encoding.ASCII.GetBytes(serializer.Xml);

                        stream.Write(buffer, 0, buffer.Length);

                        stream.Close();
                    }
                    else if (url.Contains("block_plane_0.dae"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: text/plain\n\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        BinaryReader file = new BinaryReader(File.Open("block_plane_0.dae", FileMode.Open, FileAccess.Read, FileShare.Read));
                        byte[] buffer = new byte[1024];
                        while (file.PeekChar() != -1)
                        {

                            int leng = file.Read(buffer, 0, buffer.Length);

                            stream.Write(buffer, 0, leng);
                        }
                        file.Close();
                        stream.Close();
                    }
                    else if (url.Contains("hud.html"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: text/html\n\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        BinaryReader file = new BinaryReader(File.Open("hud.html", FileMode.Open, FileAccess.Read, FileShare.Read));
                        byte[] buffer = new byte[1024];
                        while (file.PeekChar() != -1)
                        {

                            int leng = file.Read(buffer, 0, buffer.Length);

                            stream.Write(buffer, 0, leng);
                        }
                        file.Close();
                        stream.Close();
                    }
                    else if (url.ToLower().Contains("hud.jpg") || url.ToLower().Contains("map.jpg") || url.ToLower().Contains("both.jpg"))
                    {
                        string header = "HTTP/1.1 200 OK\r\nContent-Type: multipart/x-mixed-replace;boundary=APMPLANNER\n\n--APMPLANNER\r\n";
                        byte[] temp = encoding.GetBytes(header);
                        stream.Write(temp, 0, temp.Length);

                        while (client.Connected)
                        {
                            System.Threading.Thread.Sleep(200); // 5hz
                            byte[] data = null;

                            if (url.ToLower().Contains("hud"))
                            {
                                //GCSViews.FlightData.myhud.streamjpgenable = true;
                                //data = GCSViews.FlightData.myhud.streamjpg.ToArray();
                            }
                            else if (url.ToLower().Contains("map"))
                            {
                                //GCSViews.FlightData.mymap.streamjpgenable = true;
                                //data = GCSViews.FlightData.mymap.streamjpg.ToArray();
                            }
                            else
                            {
                                /*GCSViews.FlightData.mymap.streamjpgenable = true;
                                GCSViews.FlightData.myhud.streamjpgenable = true;
                                Image img1 = Image.FromStream(GCSViews.FlightData.myhud.streamjpg);
                                Image img2 = Image.FromStream(GCSViews.FlightData.mymap.streamjpg);
                                int bigger = img1.Height > img2.Height ? img1.Height : img2.Height;
                                Image imgout = new Bitmap(img1.Width + img2.Width, bigger);

                                Graphics grap = Graphics.FromImage(imgout);

                                grap.DrawImageUnscaled(img1, 0, 0);
                                grap.DrawImageUnscaled(img2, img1.Width, 0);

                                MemoryStream streamjpg = new MemoryStream();
                                imgout.Save(streamjpg, System.Drawing.Imaging.ImageFormat.Jpeg);
                                data = streamjpg.ToArray();*/

                            }

                            header = "Content-Type: image/jpeg\r\nContent-Length: " + data.Length + "\r\n\r\n";
                            temp = encoding.GetBytes(header);
                            stream.Write(temp, 0, temp.Length);

                            stream.Write(data, 0, data.Length);

                            header = "\r\n--APMPLANNER\r\n";
                            temp = encoding.GetBytes(header);
                            stream.Write(temp, 0, temp.Length);

                        }
                        //GCSViews.FlightData.mymap.streamjpgenable = false;
                        //GCSViews.FlightData.myhud.streamjpgenable = false;
                        stream.Close();

                    }
                    stream.Close();
                }
                catch (Exception ee) { log.Info("Failed mjpg " + ee.Message); }
            }
        }

        private void TOOL_APMFirmware_SelectedIndexChanged(object sender, EventArgs e)
        {
            APMFirmware = (MainV2.Firmwares)Enum.Parse(typeof(MainV2.Firmwares), TOOL_APMFirmware.Text);
            MainV2.cs.firmware = APMFirmware;
        }

        private void MainV2_Resize(object sender, EventArgs e)
        {
            log.Info("myview width " + MyView.Width + " height " + MyView.Height);
            log.Info("this   width " + this.Width + " height " + this.Height);
        }

        private void MenuHelp_Click(object sender, EventArgs e)
        {
            /*MyView.Controls.Clear();

            UserControl temp = new GCSViews.Help();

            ThemeManager.ApplyThemeTo(temp);

            temp.Anchor = AnchorStyles.Bottom | AnchorStyles.Left | AnchorStyles.Right | AnchorStyles.Top;

            temp.Dock = DockStyle.Fill;

            MyView.Controls.Add(temp);

            temp.ForeColor = Color.White;

            temp.BackColor = Color.FromArgb(0x26, 0x27, 0x28);*/
        }


        public static void updatecheck(ProgressReporterDialogue frmProgressReporter)
        {
            var baseurl = ConfigurationManager.AppSettings["UpdateLocation"];
            try
            {
                bool update = updatecheck(frmProgressReporter, baseurl, "");
                var process = new Process();
                string exePath = Path.GetDirectoryName(Application.ExecutablePath);
                if (MONO)
                {
                    process.StartInfo.FileName = "mono";
                    process.StartInfo.Arguments = " \"" + exePath + Path.DirectorySeparatorChar + "Updater.exe\"";
                }
                else
                {
                    process.StartInfo.FileName = exePath + Path.DirectorySeparatorChar + "Updater.exe";
                    process.StartInfo.Arguments = "";
                    try
                    {
                        foreach (string newupdater in Directory.GetFiles(exePath, "Updater.exe*.new"))
                        {
                            File.Copy(newupdater, newupdater.Remove(newupdater.Length - 4), true);
                            File.Delete(newupdater);
                        }
                    }
                    catch (Exception ex)
                    {
                        log.Error("Exception during update", ex);
                    }
                }
                if (frmProgressReporter != null)
                    frmProgressReporter.UpdateProgressAndStatus(-1, "Starting Updater");
                log.Info("Starting new process: " + process.StartInfo.FileName + " with " + process.StartInfo.Arguments);
                process.Start();
                log.Info("Quitting existing process");
                try
                {
                    MainV2.instance.BeginInvoke((MethodInvoker)delegate() {
                        Application.Exit();
                    });
                }
                catch
                {
                    Application.Exit();
                }
            }
            catch (Exception ex)
            {
                log.Error("Update Failed", ex);
                MessageBox.Show("Update Failed " + ex.Message);
            }
        }

        private static void UpdateLabel(Label loadinglabel, string text)
        {
            MainV2.instance.Invoke((MethodInvoker)delegate
            {
                loadinglabel.Text = text;

                Application.DoEvents();
            });
        }

        private static void CheckForUpdate()
        {
            var baseurl = ConfigurationManager.AppSettings["UpdateLocation"];
            string path = Path.GetFileName(Application.ExecutablePath);

            // Create a request using a URL that can receive a post. 
            string requestUriString = baseurl + path;
            log.Debug("Checking for update at: " + requestUriString);
            var webRequest = WebRequest.Create(requestUriString);
            webRequest.Timeout = 5000;

            // Set the Method property of the request to POST.
            webRequest.Method = "HEAD";

            ((HttpWebRequest)webRequest).IfModifiedSince = File.GetLastWriteTimeUtc(path);

            // Get the response.
            var response = webRequest.GetResponse();
            // Display the status.
            log.Debug("Response status: " + ((HttpWebResponse)response).StatusDescription);
            // Get the stream containing content returned by the server.
            //dataStream = response.GetResponseStream();
            // Open the stream using a StreamReader for easy access.

            bool shouldGetFile = false;

            if (File.Exists(path))
            {
                var fi = new FileInfo(path);

                string CurrentEtag = "";

                if (File.Exists(path + ".etag"))
                {
                    using (Stream fs = File.OpenRead(path + ".etag"))
                    {
                        using (StreamReader sr = new StreamReader(fs))
                        {
                            CurrentEtag = sr.ReadLine();
                            sr.Close();
                        }
                        fs.Close();
                    }
                }

                log.Info("New file Check: " + fi.Length + " vs " + response.ContentLength + " " + response.Headers[HttpResponseHeader.ETag] + " vs " + CurrentEtag);

                if (fi.Length != response.ContentLength || response.Headers[HttpResponseHeader.ETag] != CurrentEtag)
                {
                    using (var sw = new StreamWriter(path + ".etag"))
                    {
                        sw.WriteLine(response.Headers[HttpResponseHeader.ETag]);
                        sw.Close();
                    }
                    shouldGetFile = true;
                    log.Info("Newer file found: " + path + " " + fi.Length + " vs " + response.ContentLength);
                }
            }
            else
            {
                shouldGetFile = true;
                log.Info("File does not exist: Getting " + path);
                // get it
            }

            response.Close();

            if (shouldGetFile)
            {
                var dr = MessageBox.Show("Update Found\n\nDo you wish to update now?", "Update Now", MessageBoxButtons.YesNo);
                if (dr == DialogResult.Yes)
                {
                    DoUpdate();
                }
                else
                {
                    return;
                }
            }
        }

        public static void DoUpdate()
        {
            ProgressReporterDialogue frmProgressReporter = new ProgressReporterDialogue()
            {
                Text = "Check for Updates",
                StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen
            };

            ThemeManager.ApplyThemeTo(frmProgressReporter);

            frmProgressReporter.DoWork += new Controls.ProgressReporterDialogue.DoWorkEventHandler(frmProgressReporter_DoWork);

            frmProgressReporter.UpdateProgressAndStatus(-1, "Checking for Updates");

            frmProgressReporter.RunBackgroundOperationAsync();
        }

        static void frmProgressReporter_DoWork(object sender, Controls.ProgressWorkerEventArgs e)
        {
            ((ProgressReporterDialogue)sender).UpdateProgressAndStatus(-1, "Getting Base URL");
            MainV2.updatecheck((ProgressReporterDialogue)sender);
        }

        private static bool updatecheck(ProgressReporterDialogue frmProgressReporter, string baseurl, string subdir)
        {
            bool update = false;
            List<string> files = new List<string>();

            // Create a request using a URL that can receive a post. 
            log.Info(baseurl);
            WebRequest request = WebRequest.Create(baseurl);
            request.Timeout = 10000;
            // Set the Method property of the request to POST.
            request.Method = "GET";
            // Get the request stream.
            Stream dataStream; //= request.GetRequestStream();
            // Get the response.
            WebResponse response = request.GetResponse();
            // Display the status.
            log.Info(((HttpWebResponse)response).StatusDescription);
            // Get the stream containing content returned by the server.
            dataStream = response.GetResponseStream();
            // Open the stream using a StreamReader for easy access.
            StreamReader reader = new StreamReader(dataStream);
            // Read the content.
            string responseFromServer = reader.ReadToEnd();
            // Display the content.
            Regex regex = new Regex("href=\"([^\"]+)\"", RegexOptions.IgnoreCase);
            if (regex.IsMatch(responseFromServer))
            {
                MatchCollection matchs = regex.Matches(responseFromServer);
                for (int i = 0; i < matchs.Count; i++)
                {
                    if (matchs[i].Groups[1].Value.ToString().Contains(".."))
                        continue;
                    if (matchs[i].Groups[1].Value.ToString().Contains("http"))
                        continue;
                    files.Add(System.Web.HttpUtility.UrlDecode(matchs[i].Groups[1].Value.ToString()));
                }
            }

            //Console.WriteLine(responseFromServer);
            // Clean up the streams.
            reader.Close();
            dataStream.Close();
            response.Close();

            string dir = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + subdir;
            if (!Directory.Exists(dir))
                Directory.CreateDirectory(dir);
            foreach (string file in files)
            {
                if (frmProgressReporter.doWorkArgs.CancelRequested)
                {
                    frmProgressReporter.doWorkArgs.CancelAcknowledged = true;
                    throw new Exception("Cancel");
                }

                if (file.Equals("/"))
                {
                    continue;
                }
                if (file.EndsWith("/"))
                {
                    update = updatecheck(frmProgressReporter, baseurl + file, subdir.Replace('/', Path.DirectorySeparatorChar) + file) && update;
                    continue;
                }
                if (frmProgressReporter != null)
                    frmProgressReporter.UpdateProgressAndStatus(-1, "Checking " + file);

                string path = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + subdir + file;


                // Create a request using a URL that can receive a post. 
                request = WebRequest.Create(baseurl + file);
                log.Info(baseurl + file + " ");
                // Set the Method property of the request to POST.
                request.Method = "HEAD";

                ((HttpWebRequest)request).IfModifiedSince = File.GetLastWriteTimeUtc(path);

                // Get the response.
                response = request.GetResponse();
                // Display the status.
                log.Info(((HttpWebResponse)response).StatusDescription);
                // Get the stream containing content returned by the server.
                //dataStream = response.GetResponseStream();
                // Open the stream using a StreamReader for easy access.

                bool updateThisFile = false;

                if (File.Exists(path))
                {
                    FileInfo fi = new FileInfo(path);

                    //log.Info(response.Headers[HttpResponseHeader.ETag]);
                    string CurrentEtag = "";

                    if (File.Exists(path + ".etag"))
                    {
                        using (Stream fs = File.OpenRead(path + ".etag"))
                        {
                            using (StreamReader sr = new StreamReader(fs))
                            {
                                CurrentEtag = sr.ReadLine();
                                sr.Close();
                            }
                            fs.Close();
                        }
                    }

                    log.Debug("New file Check: " + fi.Length + " vs " + response.ContentLength + " " + response.Headers[HttpResponseHeader.ETag] + " vs " + CurrentEtag);

                    if (fi.Length != response.ContentLength || response.Headers[HttpResponseHeader.ETag] != CurrentEtag)
                    {
                        using (StreamWriter sw = new StreamWriter(path + ".etag"))
                        {
                            sw.WriteLine(response.Headers[HttpResponseHeader.ETag]);
                            sw.Close();
                        }
                        updateThisFile = true;
                        log.Info("NEW FILE " + file);
                    }
                }
                else
                {
                    updateThisFile = true;
                    log.Info("NEW FILE " + file);
                    // get it
                }

                reader.Close();
                //dataStream.Close();
                response.Close();

                if (updateThisFile)
                {
                    if (!update)
                    {
                        //DialogResult dr = MessageBox.Show("Update Found\n\nDo you wish to update now?", "Update Now", MessageBoxButtons.YesNo);
                        //if (dr == DialogResult.Yes)
                        {
                            update = true;
                        }
                        //else
                        {
                            //    return;
                        }
                    }
                    if (frmProgressReporter != null)
                        frmProgressReporter.UpdateProgressAndStatus(-1, "Getting " + file);

                    // from head
                    long bytes = response.ContentLength;

                    // Create a request using a URL that can receive a post. 
                    request = HttpWebRequest.Create(baseurl + file);
                    // Set the Method property of the request to POST.
                    request.Method = "GET";

                    ((HttpWebRequest)request).AutomaticDecompression = DecompressionMethods.GZip | DecompressionMethods.Deflate;

                    request.Headers.Add("Accept-Encoding", "gzip,deflate"); 

                    // Get the response.
                    response = request.GetResponse();
                    // Display the status.
                    log.Info(((HttpWebResponse)response).StatusDescription);
                    // Get the stream containing content returned by the server.
                    dataStream = response.GetResponseStream();

                    long contlen = bytes;

                    byte[] buf1 = new byte[1024];

                    FileStream fs = new FileStream(path + ".new", FileMode.Create); // 

                    DateTime dt = DateTime.Now;

                    dataStream.ReadTimeout = 30000;

                    while (dataStream.CanRead)
                    {
                        try
                        {
                            if (dt.Second != DateTime.Now.Second)
                            {
                                if (frmProgressReporter != null)
                                    frmProgressReporter.UpdateProgressAndStatus((int)(((double)(contlen - bytes) / (double)contlen) * 100), "Getting " + file + ": " + (((double)(contlen - bytes) / (double)contlen) * 100).ToString("0.0") + "%"); //+ Math.Abs(bytes) + " bytes");
                                dt = DateTime.Now;
                            }
                        }
                        catch { }
                        log.Info(file + " " + bytes);
                        int len = dataStream.Read(buf1, 0, 1024);
                        if (len == 0)
                            break;
                        bytes -= len;
                        fs.Write(buf1, 0, len);
                    }

                    fs.Close();
                    dataStream.Close();
                    response.Close();
                }


            }

            //P.StartInfo.CreateNoWindow = true;
            //P.StartInfo.RedirectStandardOutput = true;
            return update;


        }

        protected override bool ProcessCmdKey(ref Message msg, Keys keyData)
        {
            if (keyData == (Keys.Control | Keys.F))
            {
                Form frm = new temp();
                ThemeManager.ApplyThemeTo(frm);
                frm.Show();
                return true;
            }
            if (keyData == (Keys.Control | Keys.S))
            {
                ScreenShot();
                return true;
            }
            if (keyData == (Keys.Control | Keys.G)) // test
            {
                Form frm = new SerialOutput();
                ThemeManager.ApplyThemeTo(frm);
                frm.Show();
                return true;
            }
            if (keyData == (Keys.Control | Keys.A)) // test
            {
                Form frm = new _3DRradio();
                ThemeManager.ApplyThemeTo(frm);
                frm.Show();
                return true;
            }
            if (keyData == (Keys.Control | Keys.T)) // for override connect
            {
                try
                {
                    MainV2.comPort.Open(false);
                }
                catch (Exception ex) { MessageBox.Show(ex.ToString()); }
                return true;
            }
            if (keyData == (Keys.Control | Keys.Y)) // for ryan beall
            {
#if MAVLINK10
                // write
                MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_STORAGE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
                //read
                ///////MainV2.comPort.doCommand(MAVLink.MAV_CMD.PREFLIGHT_STORAGE, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
#else
                MainV2.comPort.doAction(MAVLink.MAV_ACTION.MAV_ACTION_STORAGE_WRITE);
#endif
                MessageBox.Show("Done MAV_ACTION_STORAGE_WRITE");
                return true;
            }
            if (keyData == (Keys.Control | Keys.J)) // for jani
            {
                string data = "!!";
                Common.InputBox("inject", "enter data to be written", ref data);
                MainV2.comPort.Write(data + "\r");
                return true;
            }
            return base.ProcessCmdKey(ref msg, keyData);
        }

        public void changelanguage(CultureInfo ci)
        {
            if (ci != null && !Thread.CurrentThread.CurrentUICulture.Equals(ci))
            {
                Thread.CurrentThread.CurrentUICulture = ci;
                config["language"] = ci.Name;
                //System.Threading.Thread.CurrentThread.CurrentCulture = ci;

                //HashSet<Control> views = new HashSet<Control> { FlightData, FlightPlanner, Simulation, Firmware };
                HashSet<Control> views = new HashSet<Control> { FlightPlanner};

                foreach (Control view in MyView.Controls)
                    views.Add(view);

                foreach (Control view in views)
                {
                    if (view != null)
                    {
                        ComponentResourceManager rm = new ComponentResourceManager(view.GetType());
                        foreach (Control ctrl in view.Controls)
                            rm.ApplyResource(ctrl);
                        rm.ApplyResources(view, "$this");
                    }
                }
            }
        }

        private void MainV2_FormClosing(object sender, FormClosingEventArgs e)
        {
            config["MainHeight"] = this.Height;
            config["MainWidth"] = this.Width;
            config["MainMaximised"] = this.WindowState.ToString();

            config["MainLocX"] = this.Location.X.ToString();
            config["MainLocY"] = this.Location.Y.ToString();

            try
            {
                comPort.logreadmode = false;
                if (comPort.logfile != null)
                    comPort.logfile.Close();
            }
            catch { }

        }

        public static string getConfig(string paramname)
        {
            if (config[paramname] != null)
                return config[paramname].ToString();
            return "";
        }

        public void changeunits()
        {
            try
            {
                // dist
                if (MainV2.config["distunits"] != null)
                {
                    switch ((Common.distances)Enum.Parse(typeof(Common.distances), MainV2.config["distunits"].ToString()))
                    {
                        case Common.distances.Meters:
                            MainV2.cs.multiplierdist = 1;
                            break;
                        case Common.distances.Feet:
                            MainV2.cs.multiplierdist = 3.2808399f;
                            break;
                    }
                }

                // speed
                if (MainV2.config["speedunits"] != null)
                {
                    switch ((Common.speeds)Enum.Parse(typeof(Common.speeds), MainV2.config["speedunits"].ToString()))
                    {
                        case Common.speeds.ms:
                            MainV2.cs.multiplierspeed = 1;
                            break;
                        case Common.speeds.fps:
                            MainV2.cs.multiplierdist = 3.2808399f;
                            break;
                        case Common.speeds.kph:
                            MainV2.cs.multiplierspeed = 3.6f;
                            break;
                        case Common.speeds.mph:
                            MainV2.cs.multiplierspeed = 2.23693629f;
                            break;
                        case Common.speeds.knots:
                            MainV2.cs.multiplierspeed = 1.94384449f;
                            break;
                    }
                }
            }
            catch { }

        }
        private void CMB_baudrate_TextChanged(object sender, EventArgs e)
        {
            StringBuilder sb = new StringBuilder();
            int baud = 0;
            for (int i = 0; i < CMB_baudrate.Text.Length; i++)
                if (char.IsDigit(CMB_baudrate.Text[i]))
                {
                    sb.Append(CMB_baudrate.Text[i]);
                    baud = baud * 10 + CMB_baudrate.Text[i] - '0';
                }
            if (CMB_baudrate.Text != sb.ToString())
                CMB_baudrate.Text = sb.ToString();
            try
            {
                if (baud > 0 && comPort.BaseStream.BaudRate != baud)
                    comPort.BaseStream.BaudRate = baud;
            }
            catch (Exception) { }
        }

        private void CMB_serialport_Enter(object sender, EventArgs e)
        {
            CMB_serialport_Click(sender, e);
        }
    }
}