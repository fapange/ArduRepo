#define SLV_ADDED

using System;
using System.Collections.Generic;
using System.Reflection;
using System.Text;
using System.ComponentModel;
using ArdupilotMega.Mavlink;
using log4net;
#if SLV_ADDED
using System.Net;
using System.Net.Sockets;
#endif

namespace ArdupilotMega
{
    public class CurrentState : ICloneable
    {
        const int SLV_HILPLANE =	 0;
        const int SLV_N801RE =		 1;
        const int SLV_N802RE =		 2;
        const int SLV_N803RE =		 3;
        const int SLV_N381NA =		81;
        const int SLV_N382NA =		82;
        const int SLV_N383NA =		83;
        const int SLV_N384NA =		84;
        const int SLV_N385NA =      85;
        const int SLV_N386NA =      86;
        const int SLV_N387NA =      87;

        const int BUF_SIZE = 1024;
        const int PACKET_SIZE = 64;
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        // multipliers
        public float multiplierdist = 1;
        public float multiplierspeed = 1;

        // orientation - rads
        public float roll { get; set; }
        public float pitch { get; set; }
        public float yaw { get { return _yaw; } set { if (value < 0) { _yaw = value + 360; } else { _yaw = value; } } }
        private float _yaw = 0;

        public float groundcourse { get { return _groundcourse; } set { if (value < 0) { _groundcourse = value + 360; } else { _groundcourse = value; } } }
        private float _groundcourse = 0;

        /// <summary>
        /// time over target in seconds
        /// </summary>
        public int tot { get { if (groundspeed <= 0) return 0; return (int)(wp_dist / groundspeed); } }

        // speeds
        public float airspeed { get { return _airspeed * multiplierspeed; } set { _airspeed = value; } }
        public float groundspeed { get { return _groundspeed * multiplierspeed; } set { _groundspeed = value; } }
        float _airspeed;
        float _groundspeed;
        float _verticalspeed;
        public float verticalspeed { get { if (float.IsNaN(_verticalspeed)) _verticalspeed = 0; return _verticalspeed; } set { _verticalspeed = _verticalspeed * 0.8f + value * 0.2f; } }
        public float wind_dir { get; set; }
        public float wind_vel { get; set; }
        /// <summary>
        /// used in wind calc
        /// </summary>
        //double Wn_fgo;
        /// <summary>
        /// used for wind calc
        /// </summary>
        //double We_fgo;

        //(alt_now - alt_then)/(time_now-time_then)

        private float _alt = 0;
        private float _gps_alt = 0;
        private float _baro_alt = 0;

        // position
        public float lat { get; set; }
        public float lng { get; set; }
        //public float alt { get { return (_alt - altoffsethome) * multiplierdist; } set { _alt = value; } }
        public float alt { get { return _alt * multiplierdist; } set { _alt = value; } }
        public float baro_alt { get { return _baro_alt * multiplierdist; } set { _baro_alt = value; } }
        public float gps_alt { get { return (_gps_alt - (float)HomeLocation.Alt) * multiplierdist; } set { _gps_alt = value; } }
        DateTime lastalt = DateTime.Now;
        float oldalt = 0;
        public float altoffsethome { get { return (float)HomeLocation.Alt; } }
        public float gpsstatus { get; set; }
        public float gpshdop { get; set; }
        public float satcount { get; set; }

        //public float altd1000 { get { return (alt / 1000) % 10; } }
        public float altd100 { get { return (alt / 100) % 10; } }
        public float baro_altd100 { get { return (baro_alt / 100) % 10; } }
        public float gps_altd100 { get { return (gps_alt / 100) % 10; } }

        // accel
        public float ax { get; set; }
        public float ay { get; set; }
        public float az { get; set; }
        public float ax_g { get { return (ax / 1000.0f); } }
        public float ay_g { get { return (ay / 1000.0f); } }
        public float az_g { get { return (az / 1000.0f); } }
        public float axmin_g { get; set; }
        public float aymin_g { get; set; }
        public float azmin_g { get; set; }
        public float axmax_g { get; set; }
        public float aymax_g { get; set; }
        public float azmax_g { get; set; }
        public float sensor_cal_status { get; set; }
        // gyro
        public float gx { get; set; }
        public float gy { get; set; }
        public float gz { get; set; }
        // mag
        public float mx { get; set; }
        public float my { get; set; }
        public float mz { get; set; }

#if SLV_ADDED
        // calculated turn rate
        public float turnrate { get { if (groundspeed <= 1) return 0; return (roll * 9.4f) / groundspeed; } }
        // turn radius
        public float radius { get { if (groundspeed <= 1) return 0; return ((groundspeed * groundspeed) / (float)(9.4f * Math.Tan(roll * deg2rad))); } }
#else
        // calced turn rate
        public float turnrate { get { if (groundspeed <= 1) return 0; return (roll * 9.8f) / groundspeed; } }
#endif

        //radio
        public float ch1in { get; set; }
        public float ch2in { get; set; }
        public float ch3in { get; set; }
        public float ch4in { get; set; }
        public float ch5in { get; set; }
        public float ch6in { get; set; }
        public float ch7in { get; set; }
        public float ch8in { get; set; }

        // motors
        public float ch1out { get; set; }
        public float ch2out { get; set; }
        public float ach3out { get; set; }
        public float fch3out{ get; set; }
        public float ch4out { get; set; }
        public float ch5out { get; set; }
        public float ch6out { get; set; }
        public float ch7out { get; set; }
        public float ch8out { get; set; }
#if SLV_ADDED
        public float ach3percent  { get; set; }
        public float fch3percent { get; set; }
        public float mycruisethrottle { get; set; }
        public float minThrottle { get; set; }
        public float maxThrottle { get; set; }
        public float minPitch { get; set; }
        public float maxPitch { get; set; }
        public float minRoll { get; set; }
        public float maxRoll { get; set; }
        //public float maxThrottle { get; set; }
        //public float minThrottle { get; set; }
#else
        public float ach3percent
        {
            get
            {
                try
                {
                    if (MainV2.comPort.param.ContainsKey("RC3_MIN"))
                    {
                        return (int)((ach3out - float.Parse(MainV2.comPort.param["RC3_MIN"].ToString())) / (float.Parse(MainV2.comPort.param["RC3_MAX"].ToString()) - float.Parse(MainV2.comPort.param["RC3_MIN"].ToString())) * 100F);
                    }
                    else
                    {
#if SLV_ADDED
                        return ((ach3out - 1124.0F) / (1517.0F - 1124.0F) * 100F);
#else
                        return 0;
#endif
                    }
                }
                catch
                {
                    return 0;
                }
            }
        }
#endif
        //nav state
        public float nav_roll { get; set; }
        public float nav_pitch { get; set; }
        public float nav_bearing { get; set; }
        public float target_bearing { get; set; }
        public float wp_dist { get { return (_wpdist * multiplierdist); } set { _wpdist = value; } }
#if SLV_ADDED
        public float wp_radius { get { return (_wp_radius * multiplierdist); } set { _wp_radius = value; } }
        public float alt_error { get { return (targetalt - alt); } }
        public int selected_Aircraft { get { return (_selectedAircraft); } set { _selectedAircraft = value; } }
        public int num_Wpts { get { return (_num_Wpts); } set { _num_Wpts = value; } }
        public float time_left { get; set; }
#else
        public float alt_error { get { return _alt_error * multiplierdist; } set { _alt_error = value; } }
#endif
        public float ber_error { get { float error = target_bearing - yaw; if (error>180) error-=360; if (error<-180) error+=360; return(error); } set { } }
#if SLV_ADDED
        public float aspd_error { get { return (targetairspeed - airspeed); } }
#else
        public float aspd_error { get { return _aspd_error * multiplierspeed; } set { _aspd_error = value; } }
#endif
        public float xtrack_error { get; set; }
        public float wpno { get; set; }
        public string mode { get; set; }
        public float climbrate { get; set; }
        float _wpdist;
#if SLV_ADDED
        int _selectedAircraft;
        int _num_Wpts;
        float _wp_radius;
        float _targetaspd;
        float _targetalt;
        public float targetaltd100 { get { return (targetalt / 100) % 10; } }
        public float targetalt { get { return _targetalt * multiplierdist; } set { _targetalt = value; } }
        public float targetairspeed { get { return _targetaspd * multiplierspeed; } set { _targetaspd = value; } }
#else
        float _aspd_error;
        float _alt_error;

        public float targetaltd100 { get { return ((alt + alt_error) / 100) % 10; } }
        public float targetalt { get { return (float)Math.Round(alt + alt_error, 0); } }

        //airspeed_error = (airspeed_error - airspeed);
        public float targetairspeed { get { return (float)Math.Round(airspeed + aspd_error / 100, 0); } }
#endif
        
        //message
        public List<string> messages { get; set; }
        public string message { get { if (messages.Count == 0) return ""; return messages[messages.Count - 1]; } set { } }

        //battery
        public float battery_voltage { get { return _battery_voltage; } set { _battery_voltage = value / 1000; } }
        private float _battery_voltage;
        public float battery_remaining { get { return _battery_remaining; } set { _battery_remaining = value / 1000; if (_battery_remaining < 0 || _battery_remaining > 1) _battery_remaining = 0; } }
        private float _battery_remaining;

        // pressure
        public float press_abs { get; set; }
        public int press_temp { get; set; }

        // sensor offsets
        public int mag_ofs_x { get; set; }
        public int mag_ofs_y { get; set; }
        public int mag_ofs_z { get; set; }
        public float mag_declination { get; set; }
        public int raw_press { get; set; }
        public int raw_temp { get; set; }
        public float gyro_cal_x { get; set; }
        public float gyro_cal_y { get; set; }
        public float gyro_cal_z { get; set; }
        public float accel_cal_x { get; set; }
        public float accel_cal_y { get; set; }
        public float accel_cal_z { get; set; }

#if SLV_ADDED
        // HIL
        public float aileron_servo { get; set; }
        public float elevator_servo { get; set; }
        public float throttle_servo { get; set; }
        public float rudder_servo { get; set; }
        public float hilch5 { get; set; }
        public float hilch6 { get; set; }
        public float hilch7 { get; set; }
        public float hilch8 { get; set; }

        // Surface Commands
        public float ail_cmd { get; set; }
        public float ele_cmd { get; set; }
        public float thr_cmd { get; set; }
        public float rud_cmd { get; set; }
        public float ch5_cmd { get; set; }
        public float fla_cmd { get; set; }
        public float ch7_cmd { get; set; }
        public float ch8_cmd { get; set; }

        // ANALOG CHANNELS
        public byte stateVehicle { get; set; }
        public byte stateLoiter { get; set; }
        public byte stateMode { get; set; }
        public byte RC_state { get; set; }
        public float odometer { get; set; }
        public float left_ail { get; set; }
        public float left_flap { get; set; }
        public float left_elev { get; set; }
        public float right_elev { get; set; }
        public float right_flap { get; set; }
        public float right_ail { get; set; }
        public float rudder { get; set; }
        public float alpha { get; set; }
        public float beta { get; set; }
        public float mtrTemp { get; set; }
        public float notUsed { get; set; }
        public float aRPM { get; set; }
        public float fRPM{ get; set; }
        public float aRPMpwm { get; set; }
        public float fRPMpwm{ get; set; }
        public float fwdCurr { get; set; }
        public float aftCurr { get; set; }
        public float MUX { get; set; }
        public float sync { get; set; }
        public ulong usec { get; set; }
        public float fPWM { get; set; }
        public float f20vV { get; set; }
        public float f40vV { get; set; }
        public float aPWM { get; set; }
        public float a20vV { get; set; }
        public float a40vV { get; set; }
        public float geoWeight { get; set; }
#else
        // HIL
        public int hilch1 { get; set; }
        public int hilch2 { get; set; }
        public int hilch3 { get; set; }
        public int hilch4 { get; set; }
        public int hilch5;
        public int hilch6;
        public int hilch7;
        public int hilch8;
#endif

        // rc override
        public ushort rcoverridech1 { get; set; }
        public ushort rcoverridech2 { get; set; }
        public ushort rcoverridech3 { get; set; }
        public ushort rcoverridech4 { get; set; }
        public ushort rcoverridech5 { get; set; }
        public ushort rcoverridech6 { get; set; }
        public ushort rcoverridech7 { get; set; }
        public ushort rcoverridech8 { get; set; }

        internal PointLatLngAlt HomeLocation = new PointLatLngAlt();
        public float DistToMAV
        {
            get
            {
                // shrinking factor for longitude going to poles direction
                double rads = Math.Abs(HomeLocation.Lat) * 0.0174532925;
                double scaleLongDown = Math.Cos(rads);
                double scaleLongUp = 1.0f / Math.Cos(rads);

                //DST to Home
                double dstlat = Math.Abs(HomeLocation.Lat - lat) * 111319.5;
                double dstlon = Math.Abs(HomeLocation.Lng - lng) * 111319.5 * scaleLongDown;
                return (float)Math.Sqrt((dstlat * dstlat) + (dstlon * dstlon));
            }
        }

        public float ELToMAV
        {
            get
            {
                float dist = DistToMAV;

                float altdiff = (float)(alt - HomeLocation.Alt);

                float angle = (float)Math.Atan(altdiff / dist) * rad2deg;

                return angle;
            }
        }

        public float AZToMAV
        {
            get
            {
                // shrinking factor for longitude going to poles direction
                double rads = Math.Abs(HomeLocation.Lat) * 0.0174532925;
                double scaleLongDown = Math.Cos(rads);
                double scaleLongUp = 1.0f / Math.Cos(rads);

                //DIR to Home
                double dstlon = (HomeLocation.Lng - lng); //OffSet_X
                double dstlat = (HomeLocation.Lat - lat) * scaleLongUp; //OffSet Y
                double bearing = 90 + (Math.Atan2(dstlat, -dstlon) * 57.295775); //absolut home direction
                if (bearing < 0) bearing += 360;//normalization
                //bearing = bearing - 180;//absolut return direction
                //if (bearing < 0) bearing += 360;//normalization

                return (float)bearing;
            }
        }
        // current firmware
        public MainV2.Firmwares firmware = MainV2.Firmwares.ArduPlane;
        public float freemem { get; set; }
        public float brklevel { get; set; }
        public int armed { get; set; }

        // stats
        public ushort packetdropremote { get; set; }
        public ushort linkqualitygcs { get; set; }

        // requested stream rates
        public byte rateattitude { get; set; }
        public byte rateposition { get; set; }
        public byte ratestatus { get; set; }
        public byte ratesensors { get; set; }
        public byte raterc { get; set; }

        // reference
        public DateTime datetime { get; set; }

        // Odometer udp port
        UdpClient client = new UdpClient();
        IPEndPoint ep = new IPEndPoint(IPAddress.Parse("127.0.0.1"), 11000);
        //UdpClient OdometerSEND;
        byte[] odata;


        public CurrentState()
        {
            mode = "";
            messages = new List<string>();
            rateattitude = 3;
            rateposition = 3;
            ratestatus = 1;
            ratesensors = 3;
            raterc = 3;
            datetime = DateTime.MinValue;
            wp_radius = 100.0f;
            client.Connect(ep);
            //OdometerSEND = new UdpClient("localhost", 5500);
            //OdometerSEND = new UdpClient("127.0.0.1", 5500);
            //odata = new byte[4];
            odata = new byte[28];
        }

        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        private DateTime lastupdate = DateTime.Now;
        private DateTime odoupdate = DateTime.Now;
        private double tripTimer = 0;
        private float batVal = 100f;

        //private DateTime lastwindcalc = DateTime.Now;

        public void UpdateCurrentSettings(System.Windows.Forms.BindingSource bs)
        {
            if (bs != null)
                UpdateCurrentSettings(bs, false, MainV2.comPort);
        }
        /*
        public void UpdateCurrentSettings(System.Windows.Forms.BindingSource bs, bool updatenow)
        {
            UpdateCurrentSettings(bs, false, MainV2.comPort);
        }
        */
        static bool alert = true;
        static bool noplanalert = true;
        static DateTime alerttimer = DateTime.Now;
        static DateTime noplantimer = DateTime.Now;
        public void UpdateCurrentSettings(System.Windows.Forms.BindingSource bs, bool updatenow, MAVLink mavinterface)
        {
            int bytes = 0;
            Byte[] achar = new Byte[2];
            Byte[] buffer = new Byte[BUF_SIZE];
            bool newTraffic;
            int trafficIdx;
            string aname;
            int npackets;
            Char n0, n1, n2, n3, n4, n5, n6, n7;

            if (DateTime.Now > lastupdate.AddMilliseconds(19) || updatenow) // 50 hz
            {
                lastupdate = DateTime.Now;

                /*
                if (DateTime.Now.Second != lastwindcalc.Second)
                {
                    lastwindcalc = DateTime.Now;
                    dowindcalc();
                }*/

                // Get UDP packets
                #if SLV_ADDED
                    if (MainV2.trafficSocket.Available > 0)
                    {
                        bytes = MainV2.trafficSocket.Receive(buffer, buffer.Length, (SocketFlags)0);
                        npackets = bytes / PACKET_SIZE;
                        //Console.Write("PACKETS: "); Console.WriteLine(npackets);
                        while (npackets > 0)
                        {
                            MainV2.ID[0] = buffer[4]; MainV2.ID[1] = buffer[5]; MainV2.ID[2] = buffer[6]; MainV2.ID[3] = buffer[7];
                            MainV2.ID[4] = buffer[8]; MainV2.ID[5] = buffer[9]; MainV2.ID[6] = buffer[10]; MainV2.ID[7] = buffer[11];
                            MainV2.Latitude = BitConverter.ToDouble(buffer, 12);    //Degrees
                            MainV2.Longitude = BitConverter.ToDouble(buffer, 20);    //Degrees
                            MainV2.Elevation = BitConverter.ToDouble(buffer, 28)/3.19;    //Feet
                            MainV2.Vx = BitConverter.ToDouble(buffer, 36);
                            MainV2.Vy = BitConverter.ToDouble(buffer, 44);
                            MainV2.Vz = BitConverter.ToDouble(buffer, 52);
                            MainV2.Own = buffer[60];
                            #if true
                                achar[0] = MainV2.ID[0]; achar[1] = 0; n0 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[1]; achar[1] = 0; n1 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[2]; achar[1] = 0; n2 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[3]; achar[1] = 0; n3 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[4]; achar[1] = 0; n4 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[5]; achar[1] = 0; n5 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[6]; achar[1] = 0; n6 = BitConverter.ToChar(achar, 0);
                                achar[0] = MainV2.ID[7]; achar[1] = 0; n7 = BitConverter.ToChar(achar, 0);
                                aname = n0.ToString() + n1.ToString() + n2.ToString() + n3.ToString() + n4.ToString() + n5.ToString() + n6.ToString() + n7.ToString();
                                
                                newTraffic = true;
                                for (trafficIdx = 0; ((trafficIdx < MainV2.trafficTable.Count) && newTraffic); trafficIdx++)
                                {
                                    if (MainV2.trafficTable[trafficIdx].ID.Equals(aname))
                                    {
                                        newTraffic = false;
                                    }
                                }
                                if (newTraffic)
                                {
                                    MainV2.trafficTable.Add(new MainV2.Aircraft(aname, (float)MainV2.Latitude, (float)MainV2.Longitude, (float)MainV2.Elevation, (float)MainV2.Vz));
                                    //Console.Write("                                              Added  : "); Console.WriteLine(aname);
                                }
                                else
                                {
                                    MainV2.trafficTable[trafficIdx - 1].Update((float)MainV2.Latitude, (float)MainV2.Longitude, (float)MainV2.Elevation, (float)MainV2.Vz, (int)MainV2.Own);
                                    //Console.Write("                                              Updated: "); Console.WriteLine(MainV2.trafficTable[trafficIdx - 1].ID);
                                }
                                npackets--;
                            #endif
                        }
                    }
                //else
                //    {
                //        Console.WriteLine("Socket is empty!");
                //    }
                #endif


                if (mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT] != null) // status text 
                {
                    string logdata = DateTime.Now + " " + Encoding.ASCII.GetString(mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT], 6, mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT].Length - 6);
                    int ind = logdata.IndexOf('\0');
                    if (ind != -1)
                        logdata = logdata.Substring(0, ind);

                    try
                    {
                        while (messages.Count > 5)
                        {
                            messages.RemoveAt(0);
                        }
                        messages.Add(logdata + "\n");

                    }
                    catch { }
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATUSTEXT] = null;
                }

                byte[] bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_FENCE_STATUS];
                if (bytearray != null) // hil
                {
                    byte[] b = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_FENCE_STATUS];
                    var fence = bytearray.ByteArrayToStructure<MAVLink.__mavlink_fence_status_t>(6);
                    //Console.WriteLine("Fence Status: c={0} s={1} t={2} ty={3} bytes[{5}{6}:{4}:{8}{9}{10}{11}:{7}]", fence.breach_count, fence.breach_status, fence.breach_time, fence.breach_type
                    //                , b[6], b[7], b[8], b[9], b[10], b[11], b[12], b[13]);

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_FENCE_STATUS] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED];
                if (bytearray != null) // hil
                {
                    var hil = bytearray.ByteArrayToStructure<MAVLink.__mavlink_rc_channels_scaled_t>(6);

                    aileron_servo = (float)hil.chan1_scaled / 100.0f;
                    elevator_servo = (float)hil.chan2_scaled / 100.0f;
                    throttle_servo = (float)hil.chan3_scaled / 100.0f;
                    // FWD Motor
                    ach3percent = (float)Math.Abs(hil.chan3_scaled / 100.0f);
                    //Console.Write("Throttle={0}    ", ach3percent);
                    // AFT Motor
                    fch3percent = (float)Math.Abs(hil.chan3_scaled / 100.0f);
                    rudder_servo = (float)hil.chan4_scaled / 100.0f;
                    hilch5 = (float)hil.chan5_scaled / 100.0f;
                    hilch6 = (float)hil.chan6_scaled / 100.0f;
                    hilch7 = (float)hil.chan7_scaled / 100.0f;
                    hilch8 = (float)hil.chan8_scaled / 100.0f;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_SCALED] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT];
                if (bytearray != null)
                {
                    var nav = bytearray.ByteArrayToStructure<MAVLink.__mavlink_nav_controller_output_t>(6);

                    nav_roll = nav.nav_roll;
                    nav_pitch = nav.nav_pitch;
                    nav_bearing = nav.nav_bearing;
                    target_bearing = nav.target_bearing;
                    wp_dist = nav.wp_dist;
#if SLV_ADDED
                    targetalt = nav.alt_error;
                    targetairspeed = nav.aspd_error;
#else
                    alt_error = nav.alt_error;
                    aspd_error = nav.aspd_error;
#endif
                    xtrack_error = nav.xtrack_error;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT] = null;
                }
#if MAVLINK10
                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_HEARTBEAT];
                if (bytearray != null)
                {
                    var hb = bytearray.ByteArrayToStructure<MAVLink.__mavlink_heartbeat_t>(6);

                    string oldmode = mode;

                    mode = "Unknown";

                    if ((hb.base_mode & (byte)MAVLink.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) != 0)
                    {
                        if (hb.type == (byte)MAVLink.MAV_TYPE.MAV_TYPE_FIXED_WING)
                        {
                            switch (hb.custom_mode)
                            {
                                case (byte)(Common.apmmodes.MANUAL):
                                    mode = "Manual";
                                    break;
                                case (byte)(Common.apmmodes.GUIDED):
                                    mode = "Guided";
                                    break;
                                case (byte)(Common.apmmodes.STABILIZE):
                                    mode = "Stabilize";
                                    break;
                                case (byte)(Common.apmmodes.FLY_BY_WIRE_A):
                                    mode = "FBW A";
                                    break;
                                case (byte)(Common.apmmodes.FLY_BY_WIRE_B):
                                    mode = "FBW B";
                                    break;
                                case (byte)(Common.apmmodes.AUTO):
                                    mode = "Auto";
                                    break;
                                case (byte)(Common.apmmodes.RTL):
                                    mode = "RTL";
                                    break;
                                case (byte)(Common.apmmodes.LOITER):
                                    mode = "Loiter";
                                    break;
                                case (byte)(Common.apmmodes.CIRCLE):
                                    mode = "Circle";
                                    break;
                                default:
                                    mode = "Unknown";
                                    break;
                            }
                        }
                        else if (hb.type == (byte)MAVLink.MAV_TYPE.MAV_TYPE_QUADROTOR) 
                        {
                            switch (hb.custom_mode)
                            {
                                case (byte)(Common.ac2modes.STABILIZE):
                                    mode = "Stabilize";
                                    break;
                                case (byte)(Common.ac2modes.ACRO):
                                    mode = "Acro";
                                    break;
                                case (byte)(Common.ac2modes.ALT_HOLD):
                                    mode = "Alt Hold";
                                    break;
                                case (byte)(Common.ac2modes.AUTO):
                                    mode = "Auto";
                                    break;
                                case (byte)(Common.ac2modes.GUIDED):
                                    mode = "Guided";
                                    break;
                                case (byte)(Common.ac2modes.LOITER):
                                    mode = "Loiter";
                                    break;
                                case (byte)(Common.ac2modes.RTL):
                                    mode = "RTL";
                                    break;
                                case (byte)(Common.ac2modes.CIRCLE):
                                    mode = "Circle";
                                    break;
                                        case (byte)(Common.ac2modes.LAND):
                            mode = "Land";
                            break;
                                default:
                                    mode = "Unknown";
                                    break;
                            }
                        }
                    }

                    if (oldmode != mode && MainV2.speechenable && MainV2.getConfig("speechmodeenabled") == "True")
                    {
                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechmode")));
                    }
                }


                bytearray = mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS];
                if (bytearray != null)
                {
                    var sysstatus = bytearray.ByteArrayToStructure<MAVLink.__mavlink_sys_status_t>(6);

                    battery_voltage = sysstatus.voltage_battery;
                    battery_remaining = sysstatus.battery_remaining;

                    packetdropremote = sysstatus.drop_rate_comm;

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                }
#else
                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_HEARTBEAT];
                if (bytearray != null)
                {
                    var hb = bytearray.ByteArrayToStructure<MAVLink.__mavlink_heartbeat_t>(6);
                    // Using the heartbeat vehicleID to tell ground station which aircraft is in use
                    MainV2.selectedAircraft = hb.vehicleID;
                    selected_Aircraft = hb.vehicleID;
                    MainV2.numWpts = hb.numWpts;
                    num_Wpts = hb.numWpts;
                    if (num_Wpts == 0)
                    {
                        if (noplanalert)
                        {
                            //graphicsObject.DrawLine(redPen, centercircle.Left - 80, centercircle.Bottom - 80, centercircle.Right+80, centercircle.Top+80);
                            //graphicsObject.DrawLine(redPen, centercircle.Left - 80, centercircle.Top + 80, centercircle.Right + 80, centercircle.Bottom - 80);
                            //drawstring(graphicsObject, "Alert: Low Airspeed " + airspeed, font, fontsize + 5, Brushes.Red, centercircle.Left - 170, centercircle.Top - 17);
                            MainV2.talk.SpeakAsync("Alert, No Flight Plan");
                            noplanalert = false;
                            noplantimer = DateTime.Now;
                        }
                        if (DateTime.Now > noplantimer.AddSeconds(2.0))
                            noplanalert = true;
                    }
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_HEARTBEAT] = null;
                    if (MainV2.comPort.param["THR_MIN"] != null) { minThrottle = (float)MainV2.comPort.param["THR_MIN"]; }
                    if (MainV2.comPort.param["THR_MAX"] != null) { maxThrottle = (float)MainV2.comPort.param["THR_MAX"]; }
                    if (MainV2.comPort.param["LIM_PITCH_MIN"] != null) { minPitch = (float)MainV2.comPort.param["LIM_PITCH_MIN"]; }
                    if (MainV2.comPort.param["LIM_PITCH_MAX"] != null) { maxPitch = (float)MainV2.comPort.param["LIM_PITCH_MAX"]; }
                    if (MainV2.comPort.param["LIM_ROLL_CD"] != null) { minRoll = -(float)MainV2.comPort.param["LIM_ROLL_CD"]; maxRoll = (float)MainV2.comPort.param["LIM_ROLL_CD"]; }
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SYS_STATUS];
                if (bytearray != null)
                {
                    var sysstatus = bytearray.ByteArrayToStructure<MAVLink.__mavlink_sys_status_t>(6);

                    armed = sysstatus.status;

                    string oldmode = mode;

                    switch (sysstatus.mode)
                    {
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_UNINIT:
                            switch (sysstatus.nav_mode)
                            {
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_GROUNDED:
                                    mode = "Initialising";
                                    break;
                            }
                            break;
                        case (byte)(100 + Common.ac2modes.STABILIZE):
                            mode = "Stabilize";
                            break;
                        case (byte)(100 + Common.ac2modes.ACRO):
                            mode = "Acro";
                            break;
                        case (byte)(100 + Common.ac2modes.ALT_HOLD):
                            mode = "Alt Hold";
                            break;
                        case (byte)(100 + Common.ac2modes.AUTO):
                            mode = "Auto";
                            break;
                        case (byte)(100 + Common.ac2modes.GUIDED):
                            mode = "Guided";
                            break;
                        case (byte)(100 + Common.ac2modes.LOITER):
                            mode = "Loiter";
                            break;
                        case (byte)(100 + Common.ac2modes.RTL):
                            mode = "RTL";
                            break;
                        case (byte)(100 + Common.ac2modes.CIRCLE):
                            mode = "Circle";
                            break;
                        case (byte)(100 + Common.ac2modes.LAND):
                            mode = "Land";
                            break;
                        case (byte)(100 + Common.ac2modes.POSITION):
                            mode = "Position";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_MANUAL:
                            mode = "Manual";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_GUIDED:
                            mode = "Guided";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_TEST1:
                            mode = "Stabilize";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_TEST2:
                            mode = "FBW A"; // fall though  old
                            switch (sysstatus.nav_mode)
                            {
                                case (byte)1:
                                    mode = "FBW A";
                                    break;
                                case (byte)2:
                                    mode = "FBW B";
                                    break;
                                case (byte)3:
                                    mode = "FBW C";
                                    break;
                            }
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_TEST3:
                            mode = "Circle";
                            break;
                        case (byte)ArdupilotMega.MAVLink.MAV_MODE.MAV_MODE_AUTO:
                            switch (sysstatus.nav_mode)
                            {
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_WAYPOINT:
                                    mode = "Auto";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_RETURNING:
                                    mode = "RTL";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_HOLD:
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_LOITER:
                                    mode = "Loiter";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_LIFTOFF:
                                    mode = "Takeoff";
                                    break;
                                case (byte)ArdupilotMega.MAVLink.MAV_NAV.MAV_NAV_LANDING:
                                    mode = "Land";
                                    break;
                            }

                            break;
                        default:
                            mode = "Unknown";
                            break;
                    }

                    battery_voltage = sysstatus.vbat;
#if SLV_ADDED
                    battery_remaining = sysstatus.load;
#else
                    battery_remaining = sysstatus.battery_remaining;
#endif
                    packetdropremote = sysstatus.packet_drop;

                    if (oldmode != mode && MainV2.speechenable && MainV2.talk != null && MainV2.getConfig("speechmodeenabled") == "True")
                    {
                        if (!MainV2.b_traffic_alert)
                            MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechmode")));
                    }

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SYS_STATUS] = null;
                }
#endif

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SCALED_PRESSURE];
                if (bytearray != null)
                {
                    var pres = bytearray.ByteArrayToStructure<MAVLink.__mavlink_scaled_pressure_t>(6);
                    press_abs = pres.press_abs;
                    press_temp = pres.temperature;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SCALED_PRESSURE] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SENSOR_OFFSETS];
                if (bytearray != null)
                {
                    var sensofs = bytearray.ByteArrayToStructure<MAVLink.__mavlink_sensor_offsets_t>(6);

                    mag_ofs_x = sensofs.mag_ofs_x;
                    mag_ofs_y = sensofs.mag_ofs_y;
                    mag_ofs_z = sensofs.mag_ofs_z;
                    mag_declination = sensofs.mag_declination;

                    raw_press = sensofs.raw_press;
                    raw_temp = sensofs.raw_temp;

                    gyro_cal_x = sensofs.gyro_cal_x;
                    gyro_cal_y = sensofs.gyro_cal_y;
                    gyro_cal_z = sensofs.gyro_cal_z;

                    accel_cal_x = sensofs.accel_cal_x;
                    accel_cal_y = sensofs.accel_cal_y;
                    accel_cal_z = sensofs.accel_cal_z;

                    sensor_cal_status = (float)(22.0955 * accel_cal_x + 96.725 * accel_cal_y + 2.23345 * accel_cal_z - 25.3617);

                    /*
                    if (sensor_cal_status < 0)
                    {
                        if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                        {
                            if (DateTime.Now > alerttimer.AddSeconds(10))
                            {
                                //MainV2.talk.SpeakAsyncCancelAll();
                                string msg = "Alert, Sensor Calibration Off Nominal";
                                MainV2.talk.SpeakAsync(msg);
                                alerttimer = DateTime.Now;
                            }
                        }
                    }
                    */

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SENSOR_OFFSETS] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE];
                if (bytearray != null)
                {
                    var att = bytearray.ByteArrayToStructure<MAVLink.__mavlink_attitude_t>(6);

                    roll = att.roll * rad2deg;
                    pitch = att.pitch * rad2deg;
                    yaw = att.yaw * rad2deg;

                    //Console.WriteLine("R="+roll + " P=" + pitch + " Y=" + yaw);
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ATTITUDE] = null;
                }
#if MAVLINK10
                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW_INT];
                if (bytearray != null)
                {
                    var gps = bytearray.ByteArrayToStructure<MAVLink.__mavlink_gps_raw_int_t>(6);

                    lat = gps.lat * 1.0e-7f;
                    lng = gps.lon * 1.0e-7f;
                    //                alt = gps.alt; // using vfr as includes baro calc

                    gpsstatus = gps.fix_type;
                    //                    Console.WriteLine("gpsfix {0}",gpsstatus);

                    gpshdop = gps.eph;

                    groundspeed = gps.vel * 1.0e-2f;
                    groundcourse = gps.cog * 1.0e-2f;

                    //MAVLink.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW] = null;
                }
#else

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW];
                if (bytearray != null)
                {
                    var gps = bytearray.ByteArrayToStructure<MAVLink.__mavlink_gps_raw_t>(6);

                    lat = gps.lat;
                    lng = gps.lon;
                    gps_alt = gps.alt;
                    //                alt = gps.alt; // using vfr as includes baro calc

                    gpsstatus = gps.fix_type;
                    //                    Console.WriteLine("gpsfix {0}",gpsstatus);

                    gpshdop = gps.eph;

                    groundspeed = gps.v;
                    groundcourse = gps.hdg;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_RAW] = null;
                }
#endif

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_STATUS];
                if (bytearray != null)
                {
                    var gps = bytearray.ByteArrayToStructure<MAVLink.__mavlink_gps_status_t>(6);
                    satcount = gps.satellites_visible;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GPS_STATUS] = null;
                }


                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT];
                if (bytearray != null)
                {
                    var loc = bytearray.ByteArrayToStructure<MAVLink.__mavlink_global_position_int_t>(6);

                    //alt = loc.alt / 1000.0f;
                    lat = loc.lat / 10000000.0f;
                    lng = loc.lon / 10000000.0f;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT] = null;
                }
#if MAVLINK10
                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_MISSION_CURRENT];
                if (bytearray != null)
                {
                    var wpcur = bytearray.ByteArrayToStructure<MAVLink.__mavlink_mission_current_t>(6);
              
                    int oldwp = (int)wpno;

                    wpno = wpcur.seq;

                    if (oldwp != wpno && MainV2.speechenable && MainV2.getConfig("speechwaypointenabled") == "True")
                    {
                        MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechwaypoint")));
                    }

                    //MAVLink.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT] = null;
                }
#else

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION];
                if (bytearray != null)
                {
                    var loc = bytearray.ByteArrayToStructure<MAVLink.__mavlink_global_position_t>(6);
                    alt = loc.alt;
                    lat = loc.lat;
                    lng = loc.lon;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_GLOBAL_POSITION] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT];
                if (bytearray != null)
                {
                    var wpcur = bytearray.ByteArrayToStructure<MAVLink.__mavlink_waypoint_current_t>(6);

                    int oldwp = (int)wpno;

                    wpno = wpcur.seq;

                    if (oldwp != wpno && MainV2.speechenable && MainV2.talk != null && MainV2.getConfig("speechwaypointenabled") == "True")
                    {
                        if (!MainV2.b_traffic_alert)
                            MainV2.talk.SpeakAsync(Common.speechConversion(MainV2.getConfig("speechwaypoint")));
                    }

                    mavinterface.packets[ArdupilotMega.MAVLink.MAVLINK_MSG_ID_WAYPOINT_CURRENT] = null;
                }

#endif
                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW];
                if (bytearray != null)
                {
                    var rcin = bytearray.ByteArrayToStructure<MAVLink.__mavlink_rc_channels_raw_t>(6);

                    ch1in = rcin.chan1_raw;
                    ch2in = rcin.chan2_raw;
                    ch3in = rcin.chan3_raw;
                    ch4in = rcin.chan4_raw;
                    ch5in = rcin.chan5_raw;
                    ch6in = rcin.chan6_raw;
                    ch7in = rcin.chan7_raw;
                    ch8in = rcin.chan8_raw;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RC_CHANNELS_RAW] = null;
                }

#if SLV_ADDED
                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATE_DATA];
                if (bytearray != null)
                {
                    var ac = bytearray.ByteArrayToStructure<MAVLink.__mavlink_state_data_t>(6);
                    
                    stateVehicle = ac.num;
                    stateLoiter = ac.atloiter;
                    stateMode = ac.mode;
                    usec = ac.usec;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_STATE_DATA] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ANALOG_RAW];
                if (bytearray != null)
                {
                    var ac = bytearray.ByteArrayToStructure<MAVLink.__mavlink_analog_raw_t>(6);

                    left_ail = (40.0f / 360f) * (float)ac.chan01 - (61.77f);                         // %%%CALIBRATED
                    left_flap = (20.0f / 238f) * (float)ac.chan02 - (48.99f);                         // %%%CALIBRATED
                    left_elev = (45.0f / 424f) * (float)ac.chan03 - (60.18f);                         // %%%CALIBRATED
                    right_elev = (40.5f / 406f) * (float)ac.chan04 - (52.37f);                         // %%%CALIBRATED
                    right_flap = (20.0f / 203f) * (float)ac.chan05 - (57.14f);                         // %%%CALIBRATED
                    right_ail = (41.5f / 355f) * (float)ac.chan06 - (61.77f);                         // %%%CALIBRATED
                    rudder = (50.0f / 293f) * (float)ac.chan07 - (98.12f);                         // %%%CALIBRATED
                    alpha = (360.0f / 860f) * (float)ac.chan08 - (229.20f);                        // %%%CALIBRATED
                    beta = (360.0f / 860f) * (float)ac.chan09 - (226.30f);                        // %%%CALIBRATED
                    mtrTemp = (0.165f) * (float)ac.chan10 + (59.211f);                        // %%%CALIBRATED
                    notUsed = (1.0f) * (float)ac.chan11 + (0.0f);                           // %%%NOT USED
                    //RPM        = (8.0424f)       * (float)ac.ch12 + (0.0f);                         // %%%CALIBRATED
                    aRPM = (8.040868f) * (float)ac.chan12;                                  // %%%CALIBRATED
                    fwdCurr = 0.5f * fwdCurr + 0.5f * ((0.1230f) * (float)ac.chan13 - (2.2160f));    // %%%CALIBRATED
                    aftCurr = 0.5f * aftCurr + 0.5f * ((0.1239f) * (float)ac.chan14 - (1.8683f));    // %%%CALIBRATED
                    MUX = (1.0f) * (float)ac.chan15 + (0.0f);
                    sync = (1.0f) * (float)ac.chan16 + (0.0f);
                    usec = ac.usec;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ANALOG_RAW] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ANALOG_EU];
                if (bytearray != null)
                {
                    var ac = bytearray.ByteArrayToStructure<MAVLink.__mavlink_analog_eu_t>(6);

                    switch(MainV2.selectedAircraft)
                    {
                        case SLV_N801RE:
                            fPWM    = ac.chan01;
                            f20vV   = ac.chan02;
                            f40vV   = ac.chan03;
                            fwdCurr = ac.chan04;
                            fRPM    = ac.chan05;
                            aPWM    = ac.chan06;
                            a20vV   = ac.chan07;
                            a40vV   = ac.chan08;
                            aftCurr = ac.chan09;
                            aRPM    = ac.chan10;
                            wp_radius = ac.chan11;
                            usec = ac.usec;
                            ach3out = aPWM; // AFT Motor
                            fch3out = fPWM; // FWD Motor
                            break;
                        case SLV_HILPLANE:
                            MUX = ac.chan01;
                            RC_state = (byte)ac.chan02;
                            wind_dir = ac.chan03;
                            wind_vel = ac.chan04;
                            wp_radius = ac.chan05;
                            time_left = ac.chan06;
                            axmin_g = ac.chan07;
                            aymin_g = ac.chan08;
                            azmin_g = ac.chan09;
                            axmax_g = ac.chan10;
                            aymax_g = ac.chan11;
                            azmax_g = ac.chan12;
                            mycruisethrottle = ac.chan13;
                            odometer = ac.chan14;   // *6.976536f - 42.398147f;
                            aRPM = 0.0f;   // *6.976536f - 42.398147f;
                            fRPM = aRPM;        // *6.976536f - 42.398147f;
                            geoWeight = ac.chan15;
                            //ArdupilotMega.GCSViews.FlightData.Gheading.RangesInnerRadius[0] = 20;
                            //fwdCurr = ac.chan15 * 0.1230f - 2.2160f;
                            aftCurr = ac.chan16 * 0.1239f - 1.8688f;
                            usec = ac.usec;
                            tripTimer += (double)(DateTime.Now - odoupdate).Milliseconds/1000;
                            odoupdate = DateTime.Now;
                            batVal -= 0.005f;
                            if (batVal < 25) batVal = 100;
                            Array.Copy(BitConverter.GetBytes((double)tripTimer), 0, odata, 0, 8); // packet index
                            Array.Copy(BitConverter.GetBytes((float)odometer), 0, odata, 8, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 12, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 16, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 20, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 24, 4); // packet index
                            try
                            {
                                client.Send(odata,28);
                            }
                            catch (Exception e) { log.Info("Odometer udp send error " + e.Message); }
                            break;
                        case SLV_N802RE:
                        case SLV_N803RE:
                            MUX = ac.chan01;
                            RC_state = (byte)ac.chan02;
                            wind_dir = ac.chan03;
                            wind_vel = ac.chan04;
                            wp_radius = ac.chan05;
                            time_left = ac.chan06;
                            axmin_g = ac.chan07;
                            aymin_g = ac.chan08;
                            azmin_g = ac.chan09;
                            axmax_g = ac.chan10;
                            aymax_g = ac.chan11;
                            azmax_g = ac.chan12;
                            mycruisethrottle = ac.chan13;
                            odometer = ac.chan14;   // *6.976536f - 42.398147f;
                            aRPM = 0.0f;   // *6.976536f - 42.398147f;
                            fRPM = aRPM;        // *6.976536f - 42.398147f;
                            fwdCurr = ac.chan15 * 0.1230f - 2.2160f;
                            aftCurr = ac.chan16 * 0.1239f - 1.8688f;
                            usec = ac.usec;
                            tripTimer += (double)(DateTime.Now - odoupdate).Milliseconds/1000;
                            odoupdate = DateTime.Now;
                            //batVal -= 0.005f;
                            //if (batVal < 25) batVal = 100;
                            Array.Copy(BitConverter.GetBytes((double)tripTimer), 0, odata, 0, 8); // packet index
                            Array.Copy(BitConverter.GetBytes((float)odometer), 0, odata, 8, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 12, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 16, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 20, 4); // packet index
                            Array.Copy(BitConverter.GetBytes((float)batVal), 0, odata, 24, 4); // packet index
                            try
                            {
                                client.Send(odata,28);
                            }
                            catch (Exception e) { log.Info("Odometer udp send error " + e.Message); }
/*
                            aRPM = ac.chan14/1000.0f;   // *6.976536f - 42.398147f;
                            fRPM = aRPM;        // *6.976536f - 42.398147f;
                            fwdCurr = ac.chan15 * 0.1230f - 2.2160f;
                            aftCurr = ac.chan16 * 0.1239f - 1.8688f;
                            usec = ac.usec;
*/
/*
 *                          left_ail = ac.chan01;
                            left_flap = ac.chan02;
                            left_elev = ac.chan03;
                            right_elev = ac.chan04;
                            right_flap = ac.chan05;
                            right_ail = ac.chan06;
                            rudder = ac.chan07;
                            alpha = ac.chan08;
                            beta = ac.chan09;
                            mtrTemp = ac.chan10;
                            wp_radius = ac.chan11;
                            aRPM = ac.chan12;      // *6.976536f - 42.398147f;
                            fRPM = aRPM;      // *6.976536f - 42.398147f;
                            fwdCurr = ac.chan13;
                            aftCurr = ac.chan14;
                            MUX = ac.chan01;
                            usec = ac.usec;
 * */
                            break;
                        case SLV_N381NA:
                        case SLV_N382NA:
                        case SLV_N383NA:
                        case SLV_N384NA:
                        case SLV_N385NA:
                        case SLV_N386NA:
                        case SLV_N387NA:
                            MUX = ac.chan01;
                            RC_state = (byte)ac.chan02;
                            wind_dir = ac.chan03;
                            wind_vel = ac.chan04;
                            wp_radius = ac.chan05;
                            time_left = ac.chan06;
                            axmin_g = ac.chan07;
                            aymin_g = ac.chan08;
                            azmin_g = ac.chan09;
                            axmax_g = ac.chan10;
                            aymax_g = ac.chan11;
                            azmax_g = ac.chan12;
                            mycruisethrottle = ac.chan13;
                            usec = ac.usec;
                            break;
                        default:
                            if (!MainV2.b_traffic_alert)
                                MainV2.talk.SpeakAsync(Common.speechConversion("Invalid Aircraft Value"));
                            break;
                    }

                    if (wp_radius < 1.0f) wp_radius = 50.0f;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_ANALOG_EU] = null;
                }
#endif

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW];
                if (bytearray != null)
                {
                    var servoout = bytearray.ByteArrayToStructure<MAVLink.__mavlink_servo_output_raw_t>(6);

                    ch1out = servoout.servo1_raw;
                    ch2out = servoout.servo2_raw;
                    switch (MainV2.selectedAircraft)
                    {
                        case SLV_N801RE:
                            //ach3out = servoout.servo2_raw; // AFT Motor
                            //fch3out = servoout.servo1_raw; // FWD Motor
                            break;
                        default:
                            ach3out = servoout.servo3_raw; // AFT Motor
                            fch3out = servoout.servo3_raw; // FWD Motor
                            aRPMpwm = ach3out * 0.0148231f - 16.79642f;   // FWD Motor
                            fRPMpwm = aRPMpwm;   // AFT Motor
                            break;
                    }
#if SLV_nADDED
                    try
                    {
                        // FWD Motor
                        ach3percent = ((ach3out - MainV2.minThrottle) / (MainV2.maxThrottle - MainV2.minThrottle) * 100F);
                        // AFT Motor
                        fch3percent = ((fch3out - MainV2.minThrottle) / (MainV2.maxThrottle - MainV2.minThrottle) * 100F);
                        /*
                        if (MainV2.comPort.param.ContainsKey("RC3_MIN"))
                        {
                            ach3percent = ((ach3out - float.Parse(MainV2.comPort.param["RC3_MIN"].ToString())) / (float.Parse(MainV2.comPort.param["RC3_MAX"].ToString()) - float.Parse(MainV2.comPort.param["RC3_MIN"].ToString())) * 100F);
                        }
                        else
                        {
                            ach3percent = ((ach3out - 1124.0F) / (1517.0F - 1124.0F) * 100F);
                        }
                        */
                    }
                    catch
                    {
                        Console.WriteLine("Error calculating %throttle"); 
                        ach3percent = 0F;
                        fch3percent = 0F;
                    }
                #endif
                    ch4out = servoout.servo4_raw;
                    ch5out = servoout.servo5_raw;
                    ch6out = servoout.servo6_raw;
                    ch7out = servoout.servo7_raw;
                    ch8out = servoout.servo8_raw;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SERVO_OUTPUT_RAW] = null;
                }


                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU];
                if (bytearray != null)
                {
                    var imu = bytearray.ByteArrayToStructure<MAVLink.__mavlink_raw_imu_t>(6);

                    gx = imu.xgyro;
                    gy = imu.ygyro;
                    gz = imu.zgyro;

                    ax = imu.xacc;
                    ay = imu.yacc;
                    az = imu.zacc;

                    mx = imu.xmag;
                    my = imu.ymag;
                    mz = imu.zmag;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_RAW_IMU] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SCALED_IMU];
                if (bytearray != null)
                {
                    var imu = bytearray.ByteArrayToStructure<MAVLink.__mavlink_scaled_imu_t>(6);

                    gx = imu.xgyro;
                    gy = imu.ygyro;
                    gz = imu.zgyro;

                    ax = imu.xacc;
                    ay = imu.yacc;
                    az = imu.zacc;

                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_SCALED_IMU] = null;
                }


                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_VFR_HUD];
                if (bytearray != null)
                {
                    var vfr = bytearray.ByteArrayToStructure<MAVLink.__mavlink_vfr_hud_t>(6);

                    groundspeed = vfr.groundspeed;
                    airspeed = vfr.airspeed;
                    //time_left = vfr.climb;
                    alt = vfr.alt; // this might include baro
                    baro_alt = vfr.climb;

                    if ((DateTime.Now - lastalt).TotalSeconds >= 0.1 && oldalt != alt)
                    {
                        climbrate = (alt - oldalt) / (float)(DateTime.Now - lastalt).TotalSeconds;
                        verticalspeed = (alt - oldalt) / (float)(DateTime.Now - lastalt).TotalSeconds;
                        if (float.IsInfinity(_verticalspeed))
                            _verticalspeed = 0;
                        lastalt = DateTime.Now;
                        oldalt = alt;
                    }

                    if ((airspeed < 15) && (alt > 50) && (ach3percent > 40.0f))
                    {
                        if (alert)
                        {
                            //graphicsObject.DrawLine(redPen, centercircle.Left - 80, centercircle.Bottom - 80, centercircle.Right+80, centercircle.Top+80);
                            //graphicsObject.DrawLine(redPen, centercircle.Left - 80, centercircle.Top + 80, centercircle.Right + 80, centercircle.Bottom - 80);
                            //drawstring(graphicsObject, "Alert: Low Airspeed " + airspeed, font, fontsize + 5, Brushes.Red, centercircle.Left - 170, centercircle.Top - 17);
                            MainV2.talk.SpeakAsync("Low Airspeed " + (int)airspeed);
                            alert = false;
                            alerttimer = DateTime.Now;
                        }
                        if (DateTime.Now > alerttimer.AddSeconds(2.0))
                            alert = true;
                    }
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_VFR_HUD] = null;
                }

                bytearray = mavinterface.packets[MAVLink.MAVLINK_MSG_ID_MEMINFO];
                if (bytearray != null)
                {
                    var mem = bytearray.ByteArrayToStructure<MAVLink.__mavlink_meminfo_t>(6);
                    freemem = mem.freemem;
                    brklevel = mem.brkval;
                    mavinterface.packets[MAVLink.MAVLINK_MSG_ID_MEMINFO] = null;
                }
            }

            //Console.WriteLine(DateTime.Now.Millisecond + " start ");
            // update form
            try
            {
                if (bs != null)
                {
                    //System.Diagnostics.Debug.WriteLine(DateTime.Now.Millisecond);
                    //Console.WriteLine(DateTime.Now.Millisecond);
                    bs.DataSource = this;
                    //Console.WriteLine(DateTime.Now.Millisecond + " 1 " + updatenow);
                    bs.ResetBindings(false);
                    //Console.WriteLine(DateTime.Now.Millisecond + " done ");
                }
            }
            catch { log.InfoFormat("CurrentState Binding error"); }
        }

        public object Clone()
        {
            return this.MemberwiseClone();
        }

        /*
        public void dowindcalc()
        {
            //Wind Fixed gain Observer
            //Ryan Beall 
            //8FEB10

            double Kw = 0.010; // 0.01 // 0.10

            if (airspeed < 1 || groundspeed < 1)
                return;

#if SLV_ADDED
            double Wn_error = airspeed * Math.Cos((yaw) * deg2rad) * Math.Cos((pitch) * deg2rad) - groundspeed * Math.Cos((groundcourse) * deg2rad) - Wn_fgo;
            double We_error = airspeed * Math.Sin((yaw) * deg2rad) * Math.Cos((pitch) * deg2rad) - groundspeed * Math.Sin((groundcourse) * deg2rad) - We_fgo;
            //double Wn_error = airspeed * Math.Cos((yaw) * deg2rad) * Math.Cos((pitch - float.Parse(MainV2.comPort.param["TRIM_PITCH_CD"].ToString())) * deg2rad) - groundspeed * Math.Cos((groundcourse) * deg2rad) - Wn_fgo;
            //double We_error = airspeed * Math.Sin((yaw) * deg2rad) * Math.Cos((pitch - float.Parse(MainV2.comPort.param["TRIM_PITCH_CD"].ToString())) * deg2rad) - groundspeed * Math.Sin((groundcourse) * deg2rad) - We_fgo;
#else
            double Wn_error = airspeed * Math.Cos((yaw) * deg2rad) * Math.Cos((pitch) * deg2rad) - groundspeed * Math.Cos((groundcourse) * deg2rad) - Wn_fgo;
            double We_error = airspeed * Math.Sin((yaw) * deg2rad) * Math.Cos((pitch) * deg2rad) - groundspeed * Math.Sin((groundcourse) * deg2rad) - We_fgo;
#endif
            Wn_fgo = Wn_fgo + Kw * Wn_error;
            We_fgo = We_fgo + Kw * We_error;

            double wind_dir = (Math.Atan2(We_fgo, Wn_fgo) * (180 / Math.PI));
            double wind_vel = (Math.Sqrt(Math.Pow(We_fgo, 2) + Math.Pow(Wn_fgo, 2)));

            wind_dir = (wind_dir + 360) % 360;

            this.wind_dir = (float)wind_dir;// (float)(wind_dir * 0.5 + this.wind_dir * 0.5);
            this.wind_vel = (float)wind_vel;// (float)(wind_vel * 0.5 + this.wind_vel * 0.5);

            //Console.WriteLine("Wn_error = {0}\nWe_error = {1}\nWn_fgo =    {2}\nWe_fgo =  {3}\nWind_dir =    {4}\nWind_vel =    {5}\n",Wn_error,We_error,Wn_fgo,We_fgo,wind_dir,wind_vel);

            //Console.WriteLine("wind_dir: {0} wind_vel: {1}    as {4} yaw {5} pitch {6} gs {7} cog {8}", wind_dir, wind_vel, Wn_fgo, We_fgo , airspeed,yaw,pitch,groundspeed,groundcourse);

            //low pass the outputs for better results!
        }
        */
    }
}