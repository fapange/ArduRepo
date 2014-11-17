﻿
#define SLV_ADDED

using System;
using System.Collections.Generic;
using System.Text;
using System.IO.Ports;
using System.Runtime.InteropServices;
using System.Collections; // hashs
using System.Diagnostics; // stopwatch
using System.Reflection;
using System.Reflection.Emit;
using System.IO;
using System.Drawing;
using System.Threading;
using ArdupilotMega.Controls;
using ArdupilotMega.Mavlink;
using System.ComponentModel;
using log4net;

namespace ArdupilotMega
{
    public partial class MAVLink
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        public ICommsSerial BaseStream = new SerialPort();

        private const double CONNECT_TIMEOUT_SECONDS = 30;

        /// <summary>
        /// progress form to handle connect and param requests
        /// </summary>
        ProgressReporterDialogue frmProgressReporter;

        /// <summary>
        /// used for outbound packet sending
        /// </summary>
        byte packetcount = 0;
        /// <summary>
        /// mavlink remote sysid
        /// </summary>
        public byte sysid = 0;
        /// <summary>
        /// mavlink remove compid
        /// </summary>
        public byte compid = 0;
        /// <summary>
        /// set to true after the GCS receives its ID from AP
        /// </summary>
        public bool GCS_ID_set = false;
        /// <summary>
        /// storage for whole paramater list
        /// </summary>
        public Hashtable param = new Hashtable();
        /// <summary>
        /// storage of a previous packet recevied of a specific type
        /// </summary>
        public byte[][] packets = new byte[256][];
        /// <summary>
        /// used to calc packets per second on any single message type - used for stream rate comparaison
        /// </summary>
        public double[] packetspersecond = new double[256];
        /// <summary>
        /// time last seen a packet of a type
        /// </summary>
        DateTime[] packetspersecondbuild = new DateTime[256];
        /// <summary>
        /// used as a serial port write lock
        /// </summary>
        object objlock = new object();
        /// <summary>
        /// used for a readlock on readpacket
        /// </summary>
        object readlock = new object();
        /// <summary>
        /// used for tlog file lock
        /// </summary>
        object logwritelock = new object();
        /// <summary>
        /// time seen of last mavlink packet
        /// </summary>
        public DateTime lastvalidpacket = DateTime.Now;
        /// <summary>
        /// old log support
        /// </summary>
        bool oldlogformat = false;

        /// <summary>
        /// mavlink version
        /// </summary>
        byte mavlinkversion = 0;
        /// <summary>
        /// mavlink ap type
        /// </summary>
        public byte aptype = 0;
        /// <summary>
        /// used as a snapshot of what is loaded on the ap atm. - derived from the stream
        /// </summary>
        public PointLatLngAlt[] wps = new PointLatLngAlt[200];
        /// <summary>
        /// used as a snapshot of candidate plans from Stratway. - derived from the stream
        /// </summary>
        public PointLatLngAlt[] cwps = new PointLatLngAlt[200];
        /// <summary>
        /// turns on console packet display
        /// </summary>
        public bool debugmavlink = false;
        /// <summary>
        /// enabled read from file mode
        /// </summary>
        public bool logreadmode = false;
        public DateTime lastlogread = DateTime.MinValue;
        public BinaryReader logplaybackfile = null;
        public BinaryWriter logfile = null;

        int bps1 = 0;
        int bps2 = 0;
        public int bps = 0;
        public DateTime bpstime = DateTime.Now;
        int recvpacketcount = 0;

        float synclost;
        float packetslost = 0;
        float packetsnotlost = 0;
        DateTime packetlosttimer = DateTime.Now;

        //Stopwatch stopwatch = new Stopwatch();

        public void Close()
        {
            BaseStream.Close();
            GCS_ID_set = false;
        }

        public void Open()
        {
            Open(false);
        }

        public void Open(bool getparams)
        {
            if (BaseStream.IsOpen)
                return;

            frmProgressReporter = new ProgressReporterDialogue
                                      {
                                          StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen,
                                          Text = "Connecting Mavlink"
                                      };

            if (getparams)
            {
                frmProgressReporter.DoWork += FrmProgressReporterDoWorkAndParams;
            }
            else
            {
                frmProgressReporter.DoWork += FrmProgressReporterDoWorkNOParams;
            }
            frmProgressReporter.UpdateProgressAndStatus(-1, "Mavlink Connecting...");
            ThemeManager.ApplyThemeTo(frmProgressReporter);

            frmProgressReporter.RunBackgroundOperationAsync();

            cwps    = new PointLatLngAlt[7];
            cwps[0] = new PointLatLngAlt(37.017177, -76.588799,  10.000000, "Home");
            cwps[1] = new PointLatLngAlt(37.018078, -76.589897, 200.000000, "c1");
            cwps[2] = new PointLatLngAlt(37.017838, -76.591919, 200.000000, "c2");
            cwps[3] = new PointLatLngAlt(37.022224, -76.592751, 200.000000, "c3");
            cwps[4] = new PointLatLngAlt(37.022925, -76.588311, 200.000000, "c4");
            cwps[5] = new PointLatLngAlt(37.024502, -76.588699, 200.000000, "c5");
            cwps[6] = new PointLatLngAlt(37.024040, -76.591293, 200.000000, "c6");
        }

        void FrmProgressReporterDoWorkAndParams(object sender, ProgressWorkerEventArgs e)
        {
            OpenBg(true, e);
        }

        void FrmProgressReporterDoWorkNOParams(object sender, ProgressWorkerEventArgs e)
        {
            OpenBg(false, e);
        }

        private void OpenBg(bool getparams,  ProgressWorkerEventArgs progressWorkerEventArgs)
        {
            frmProgressReporter.UpdateProgressAndStatus(-1, "Mavlink Connecting...");

            // reset
            sysid = 0;
            compid = 0;
            param = new Hashtable();

            try
            {
                MainV2.givecomport = true;

                BaseStream.ReadBufferSize = 4 * 1024;

                lock (objlock) // so we dont have random traffic
                {
                    log.Info("Open port with " + BaseStream.PortName + " " + BaseStream.BaudRate);

                    BaseStream.Open();

                    BaseStream.DiscardInBuffer();

                    BaseStream.toggleDTR();

                    Thread.Sleep(1000);
                }

                byte[] buffer;
                byte[] buffer1;

                DateTime start = DateTime.Now;
                DateTime deadline = start.AddSeconds(CONNECT_TIMEOUT_SECONDS);

                var countDown = new System.Timers.Timer { Interval = 1000, AutoReset = false };
                countDown.Elapsed += (sender, e) =>
                {
                    int secondsRemaining = (deadline - e.SignalTime).Seconds;
                    //if (Progress != null)
                    //    Progress(-1, string.Format("Trying to connect.\nTimeout in {0}", secondsRemaining));
                    frmProgressReporter.UpdateProgressAndStatus(-1, string.Format("Trying to connect.\nTimeout in {0}", secondsRemaining));
                    if (secondsRemaining > 0) countDown.Start();
                };
                countDown.Start();

                int count = 0;

                while (true)
                {
                    if (progressWorkerEventArgs.CancelRequested)
                    {
                        progressWorkerEventArgs.CancelAcknowledged = true;
                        countDown.Stop();
                        if (BaseStream.IsOpen)
                            BaseStream.Close();
                        MainV2.givecomport = false;
                        return;
                    }

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    log.Info(DateTime.Now.Millisecond + " Start connect loop ");

                    if (lastbad[0] == '!' && lastbad[1] == 'G' || lastbad[0] == 'G' && lastbad[1] == '!') // waiting for gps lock
                    {
                        //if (Progress != null)
                        //    Progress(-1, "Waiting for GPS detection..");
                        frmProgressReporter.UpdateProgressAndStatus(-1, "Waiting for GPS detection..");
                        deadline = deadline.AddSeconds(5); // each round is 1.1 seconds
                    }

                    if (DateTime.Now > deadline)
                    {
                        //if (Progress != null)
                        //    Progress(-1, "No Heatbeat Packets");
                        this.Close();
                        progressWorkerEventArgs.ErrorMessage = "No Heatbeat Packets Received";
                        throw new Exception("No Mavlink Heartbeat Packets where read from this port - Verify Baud Rate and setup\nIt might also be waiting for GPS Lock\nAPM Planner waits for 2 valid heartbeat packets before connecting");
                    }

                    System.Threading.Thread.Sleep(1);

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    buffer = getHeartBeat();

                    // incase we are in setup mode
                    BaseStream.WriteLine("planner\rgcs\r");

                    System.Threading.Thread.Sleep(1);

                    buffer1 = getHeartBeat();

                    try
                    {
                        log.Debug("MAv Data: len " + buffer.Length + " btr " + BaseStream.BytesToRead);
                    }
                    catch { }

                    count++;

                    if (buffer.Length > 5 && buffer1.Length > 5 && buffer[3] == buffer1[3] && buffer[4] == buffer1[4])
                    {
                        __mavlink_heartbeat_t hb = buffer.ByteArrayToStructure<__mavlink_heartbeat_t>(6);

                        mavlinkversion = hb.mavlink_version;
                        aptype = hb.type;

                        sysid = buffer[3];
                        compid = buffer[4];
                        GCS_ID_set = true;

                        recvpacketcount = buffer[2];
                        log.InfoFormat("ID sys {0} comp {1} ver{2}", sysid, compid, mavlinkversion);
                        break;
                    }

                }

                countDown.Stop();

//                if (Progress != null)
//                    Progress(-1, "Getting Params.. (sysid " + sysid + " compid " + compid + ") ");
                frmProgressReporter.UpdateProgressAndStatus(0, "Getting Params.. (sysid " + sysid + " compid " + compid + ") ");

                if (getparams)
                    getParamListBG();

                if (frmProgressReporter.doWorkArgs.CancelAcknowledged == true)
                {
                    MainV2.givecomport = false;
                    if (BaseStream.IsOpen)
                        BaseStream.Close();
                    return;
                }
            }
            catch (Exception e)
            {
                try
                {
                    BaseStream.Close();
                }
                catch { }
                MainV2.givecomport = false;
//                if (Progress != null)
//                    Progress(-1, "Connect Failed\n" + e.Message);
                if (string.IsNullOrEmpty(progressWorkerEventArgs.ErrorMessage))
                    progressWorkerEventArgs.ErrorMessage = "Connect Failed";
                throw e;
            }
            //frmProgressReporter.Close();
            MainV2.givecomport = false;
            frmProgressReporter.UpdateProgressAndStatus(100, "Done.");
            log.Info("Done open " + sysid + " " + compid);
            packetslost = 0;
        }

        byte[] getHeartBeat()
        {
            DateTime start = DateTime.Now;
            while (true)
            {
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    log.Info("getHB packet received: " + buffer.Length + " btr " + BaseStream.BytesToRead + " type " + buffer[5] );
                    if (buffer[5] == MAVLINK_MSG_ID_HEARTBEAT)
                    {
                        return buffer;
                    }
                }
                if (DateTime.Now > start.AddMilliseconds(2200)) // was 1200 , now 2.2 sec
                    return new byte[0];
            }
        }

        public void sendPacket(object indata)
        {
            bool run = false;
            byte a = 0;
            foreach (Type ty in mavstructs)
            {
                if (ty == indata.GetType())
                {
                    run = true;
                    generatePacket(a, indata);
                    return;
                }
                a++;
            }
            if (!run)
            {
                log.Info("Mavlink : NOT VALID PACKET sendPacket() " + indata.GetType().ToString());
            }
        }

        /// <summary>
        /// Generate a Mavlink Packet and write to serial
        /// </summary>
        /// <param name="messageType">type number</param>
        /// <param name="indata">struct of data</param>
        void generatePacket(byte messageType, object indata)
        {
            byte[] data;

            if (mavlinkversion == 3)
            {
                data = MavlinkUtil.StructureToByteArray(indata);
            }
            else
            {
                data = MavlinkUtil.StructureToByteArrayBigEndian(indata);
            }

            //Console.WriteLine(DateTime.Now + " PC Doing req "+ messageType + " " + this.BytesToRead);
            byte[] packet = new byte[data.Length + 6 + 2];

            if (mavlinkversion == 3)
            {
                packet[0] = 254;
            }
            else if (mavlinkversion == 2)
            {
                packet[0] = (byte)'U';
            }
            packet[1] = (byte)data.Length;
            packet[2] = packetcount;
            packet[3] = compid;
            packet[4] = sysid;
            //            packet[3] = 255;    // (byte)(~sysid); // this is always 255 - MYGCS
//#if MAVLINK10
//            packet[4] = (byte)MAV_COMPONENT.MAV_COMP_ID_MISSIONPLANNER;
//#else
//            packet[4] = (byte)MAV_COMPONENT.MAV_COMP_ID_WAYPOINTPLANNER;
//#endif
            packet[5] = messageType;

            int i = 6;
            foreach (byte b in data)
            {
                packet[i] = b;
                i++;
            }

            ushort checksum = MavlinkCRC.crc_calculate(packet, packet[1] + 6);

            if (mavlinkversion == 3)
            {
                checksum = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_CRCS[messageType], checksum);
            }

            byte ck_a = (byte)(checksum & 0xFF); ///< High byte
            byte ck_b = (byte)(checksum >> 8); ///< Low byte

            packet[i] = ck_a;
            i += 1;
            packet[i] = ck_b;
            i += 1;

            if (BaseStream.IsOpen)
            {
                lock (objlock)
                {
                    BaseStream.Write(packet, 0, i);
                }
            }

            try
            {
                if (logfile != null)
                {
                    lock (logwritelock)
                    {
                        byte[] datearray = BitConverter.GetBytes((UInt64)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000)); //ASCIIEncoding.ASCII.GetBytes(DateTime.Now.ToBinary() + ":");
                        Array.Reverse(datearray);
                        logfile.Write(datearray, 0, datearray.Length);
                        logfile.Write(packet, 0, i);
                        logfile.Flush();

                        //string textoutput; // Telemetry from Ground Station to AP
                        //textoutput = string.Format("GSPacket {0:x2} {1:x2} {2:x2} {3:x2} {4:x2} {5:x2}\r\n"
                        //                          , packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);
                        //Console.Write(textoutput);
                    }
                }

            }
            catch { }

            if (messageType == ArdupilotMega.MAVLink.MAVLINK_MSG_ID_REQUEST_DATA_STREAM)
            {
                try
                {
                    BinaryWriter bw = new BinaryWriter(File.OpenWrite("serialsent.raw"));
                    bw.Seek(0, SeekOrigin.End);
                    bw.Write(packet, 0, i);
                    bw.Write((byte)'\n');
                    bw.Close();
                }
                catch { } // been getting errors from this. people must have it open twice.
            }

            packetcount++;



            //System.Threading.Thread.Sleep(1);
        }

        public bool Write(string line)
        {
            lock (objlock)
            {
                BaseStream.Write(line);
            }
            return true;
        }

        public bool setParam(string paramname, object flag)
        {
            int value = (int)(float)param[paramname];

            return setParam(paramname, value | (int)flag);
        }

        /// <summary>
        /// Set parameter on apm
        /// </summary>
        /// <param name="paramname">name as a string</param>
        /// <param name="value"></param>
        public bool setParam(string paramname, float value)
        {
            if (!param.ContainsKey(paramname))
            {
                log.Info("Param doesnt exist " + paramname);
                return false;
            }

            MainV2.givecomport = true;

            __mavlink_param_set_t req = new __mavlink_param_set_t();
            req.target_system = sysid;
            req.target_component = compid;

            byte[] temp = ASCIIEncoding.ASCII.GetBytes(paramname);

            modifyParamForDisplay(false, paramname, ref value);
#if MAVLINK10
            Array.Resize(ref temp, 16);
#else
            Array.Resize(ref temp, 15);
#endif
            req.param_id = temp;
            req.param_value = (value);

            generatePacket(MAVLINK_MSG_ID_PARAM_SET, req);

            log.InfoFormat("setParam '{0}' = '{1}' sysid {2} compid {3}", paramname, req.param_value, sysid, compid);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setParam Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_PARAM_SET, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setParam " + paramname);
                }

                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_PARAM_VALUE)
                    {
                        __mavlink_param_value_t par = buffer.ByteArrayToStructure<__mavlink_param_value_t>(6);

                        string st = System.Text.ASCIIEncoding.ASCII.GetString(par.param_id);

                        int pos = st.IndexOf('\0');

                        if (pos != -1)
                        {
                            st = st.Substring(0, pos);
                        }

                        if (st != paramname)
                        {
                            log.InfoFormat("MAVLINK bad param responce - {0} vs {1}", paramname, st);
                            continue;
                        }

                        modifyParamForDisplay(true, st, ref par.param_value);

                        param[st] = (par.param_value);

                        MainV2.givecomport = false;
                        //System.Threading.Thread.Sleep(100);//(int)(8.5 * 5)); // 8.5ms per byte
                        return true;
                    }
                }
            }
        }
        /*
        public Bitmap getImage()
        {
            MemoryStream ms = new MemoryStream();

        }
        */
        public void getParamList()
        {
            frmProgressReporter = new ProgressReporterDialogue
            {
                StartPosition = System.Windows.Forms.FormStartPosition.CenterScreen,
                Text = "Getting Params"
            };

            frmProgressReporter.DoWork += FrmProgressReporterGetParams;
            frmProgressReporter.UpdateProgressAndStatus(-1, "Getting Params...");
            ThemeManager.ApplyThemeTo(frmProgressReporter);

            frmProgressReporter.RunBackgroundOperationAsync();
        }

        void FrmProgressReporterGetParams(object sender, ProgressWorkerEventArgs e)
        {
            Hashtable old = new Hashtable(param);
            getParamListBG();
            if (frmProgressReporter.doWorkArgs.CancelRequested)
            {
                param = old;
            }
        }

        /// <summary>
        /// Get param list from apm
        /// </summary>
        /// <returns></returns>
        private Hashtable getParamListBG()
        {
            MainV2.givecomport = true;
            List<int> got = new List<int>();

            // clear old
            param = new Hashtable();

            int retrys = 3;
            int param_count = 0;
            int param_total = 5;

            goagain:

            __mavlink_param_request_list_t req = new __mavlink_param_request_list_t();
            req.target_system = sysid;
            req.target_component = compid;

            generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            DateTime restart = DateTime.Now;

            while (got.Count < param_total)
            {

                if (frmProgressReporter.doWorkArgs.CancelRequested)
                {
                    frmProgressReporter.doWorkArgs.CancelAcknowledged = true;
                    MainV2.givecomport = false;
                    frmProgressReporter.doWorkArgs.ErrorMessage = "User Canceled";
                    return param;
                }

                if (!(start.AddMilliseconds(5000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.InfoFormat("getParamList Retry {0} sys {1} comp {2}", retrys, sysid, compid);
                        generatePacket(MAVLINK_MSG_ID_PARAM_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    return param; // added by slv on 08/15/2014 to allow the GCS to remain connected
                                  // and just monitor the AP. There is a connection problem that 
                                  // prevents the GCS from sending commands to the AP
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getParamList " + param_count +" "+ param_total);
                }

                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
                    //stopwatch.Reset();
                    //stopwatch.Start();
                    if (buffer[5] == MAVLINK_MSG_ID_PARAM_VALUE)
                    {
                        restart = DateTime.Now;
                        start = DateTime.Now;

                        __mavlink_param_value_t par = buffer.ByteArrayToStructure<__mavlink_param_value_t>(6);

                        // set new target
                        param_total = (par.param_count);

                        
                        string paramID = System.Text.ASCIIEncoding.ASCII.GetString(par.param_id);

                        int pos = paramID.IndexOf('\0');
                        if (pos != -1)
                        {
                            paramID = paramID.Substring(0, pos);
                        }

                        // check if we already have it
                        if (got.Contains(par.param_index))
                        {
                            //Console.WriteLine("Already got '"+paramID+"'");
                            continue;
                        }

                        log.Info(DateTime.Now.Millisecond + " got param " + (par.param_index) + " of " + (param_total - 1) + " name: " + paramID);

                        modifyParamForDisplay(true, paramID, ref par.param_value);
                        param[paramID] = (par.param_value);
                        param_count++;
                        got.Add(par.param_index);

//                        if (Progress != null)
//                            Progress((param.Count * 100) / param_total, "Got param " + paramID);
                        this.frmProgressReporter.UpdateProgressAndStatus((got.Count * 100) / param_total, "Got param " + paramID);
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC paramlist " + buffer[5] + " want " + MAVLINK_MSG_ID_PARAM_VALUE + " btr " + BaseStream.BytesToRead);
                    }
                    //stopwatch.Stop();
                    //Console.WriteLine("Time elapsed: {0}", stopwatch.Elapsed);
                }
            }

            if (got.Count != param_total)
            {
                if (retrys > 0)
                {
                    this.frmProgressReporter.UpdateProgressAndStatus((got.Count * 100) / param_total, "Getting missed params");
                    retrys--;
                    goto goagain;
                }
                throw new Exception("Missing Params");
            }
            MainV2.givecomport = false;
            return param;
        }

        public static void modifyParamForDisplay(bool fromapm, string paramname, ref float value)
        {
            if (paramname.ToUpper().EndsWith("_IMAX") || paramname.ToUpper().EndsWith("ALT_HOLD_RTL") || paramname.ToUpper().EndsWith("TRIM_ARSPD_CM")
                || paramname.ToUpper().EndsWith("XTRK_ANGLE_CD") || paramname.ToUpper().EndsWith("LIM_PITCH_MAX") || paramname.ToUpper().EndsWith("LIM_PITCH_MIN")
                || paramname.ToUpper().EndsWith("LIM_ROLL_CD") || paramname.ToUpper().EndsWith("PITCH_MAX") || paramname.ToUpper().EndsWith("WP_SPEED_MAX"))
            {
                if (paramname.ToUpper().EndsWith("THR_HOLD_IMAX"))
                {
                    return;
                }

                if (fromapm)
                {
                    value /= 100.0f;
                }
                else
                {
                    value *= 100.0f;
                }
            }
            else if (paramname.ToUpper().StartsWith("TUNE_"))
            {
                if (fromapm)
                {
                    value /= 1000.0f;
                }
                else
                {
                    value *= 1000.0f;
                }
            }
        }

        /// <summary>
        /// Stops all requested data packets.
        /// </summary>
        public void stopall(bool forget)
        {
            __mavlink_request_data_stream_t req = new __mavlink_request_data_stream_t();
            req.target_system = sysid;
            req.target_component = compid;

            req.req_message_rate = 10;
            req.start_stop = 0; // stop
            req.req_stream_id = 0; // all

            // no error on bad
            try
            {
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                System.Threading.Thread.Sleep(20);
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                System.Threading.Thread.Sleep(20);
                generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
                log.Info("Stopall Done");

            }
            catch { }
        }

        public void setWPACK()
        {
#if MAVLINK10
            MAVLink.__mavlink_mission_ack_t req = new MAVLink.__mavlink_mission_ack_t();
            req.target_system = sysid;
            req.target_component = compid;
            req.type = 0;

            generatePacket(MAVLINK_MSG_ID_MISSION_ACK, req);
#else
            MAVLink.__mavlink_waypoint_ack_t req = new MAVLink.__mavlink_waypoint_ack_t();
            req.target_system = sysid;
            req.target_component = compid;
            req.type = 0;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_ACK, req);
#endif
        }

        public bool setWPCurrent(ushort index)
        {
#if MAVLINK10
            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_mission_set_current_t req = new __mavlink_mission_set_current_t();

            req.target_system = sysid;
            req.target_component = compid;
            req.seq = index;

            generatePacket(MAVLINK_MSG_ID_MISSION_SET_CURRENT, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(2000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setWPCurrent Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_SET_CURRENT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPCurrent");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_CURRENT)
                    {
                        MainV2.givecomport = false;
                        return true;
                    }
                }
            }
        }

        public bool doCommand(MAV_CMD actionid, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
        {

            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_command_long_t req = new __mavlink_command_long_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.command = (ushort)actionid;

            req.param1 = p1;
            req.param2 = p2;
            req.param3 = p3;
            req.param4 = p4;
            req.param5 = p5;
            req.param6 = p6;
            req.param7 = p7;

            generatePacket(MAVLINK_MSG_ID_COMMAND_LONG, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            int timeout = 2000;

            // imu calib take a little while
            if (actionid == MAV_CMD.PREFLIGHT_CALIBRATION)
            {
                retrys = 1;
                timeout = 6000;
            }

            while (true)
            {
                if (!(start.AddMilliseconds(timeout) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("doAction Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_COMMAND_LONG, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - doAction");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_COMMAND_ACK)
                    {


                        var ack = buffer.ByteArrayToStructure<__mavlink_command_ack_t>(6);


                        if (ack.result == (byte)MAV_RESULT.MAV_RESULT_ACCEPTED)
                        {
                            MainV2.givecomport = false;
                            return true;
                        }
                        else
                        {
                            MainV2.givecomport = false;
                            return false;
                        }
                    }
                }
            }
#else
            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_waypoint_set_current_t req = new __mavlink_waypoint_set_current_t();

            req.target_system = sysid;
            req.target_component = compid;
            req.seq = index;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(2000) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setWPCurrent Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPCurrent");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_CURRENT)
                    {
                        MainV2.givecomport = false;
                        return true;
                    }
                }
            }
        }

        public bool doAction(MAV_ACTION actionid)
        {
            MainV2.givecomport = true;
            byte[] buffer;

            __mavlink_action_t req = new __mavlink_action_t();

            req.target = sysid;
            req.target_component = compid;

            req.action = (byte)actionid;

            generatePacket(MAVLINK_MSG_ID_ACTION, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            int timeout = 2000;

            // imu calib take a little while
            if (actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_ACC ||
                actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_GYRO ||
                actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_MAG ||
                actionid == MAV_ACTION.MAV_ACTION_CALIBRATE_PRESSURE ||
                actionid == MAV_ACTION.MAV_ACTION_REBOOT)
            {
                retrys = 1;
                timeout = 20000;
            }

            while (true)
            {
                if (!(start.AddMilliseconds(timeout) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("doAction Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_ACTION, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - doAction");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_ACTION_ACK)
                    {
                        if (buffer[7] == 1)
                        {
                            MainV2.givecomport = false;
                            return true;
                        }
                        else
                        {
                            MainV2.givecomport = false;
                            return false;
                        }
                    }
                }
            }

#endif
        }

        public void requestDatastream(byte id, byte hzrate)
        {
            double pps = 0;

            switch (id)
            {
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_ALL:

                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTENDED_STATUS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_SYS_STATUS] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_SYS_STATUS];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA1:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_ATTITUDE] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_ATTITUDE];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA2:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_VFR_HUD] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_VFR_HUD];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_EXTRA3:

                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_POSITION:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_GLOBAL_POSITION_INT] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_GLOBAL_POSITION_INT];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_CONTROLLER:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RC_CHANNELS_SCALED] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RC_CHANNELS_SCALED];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RAW_SENSORS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RAW_IMU] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RAW_IMU];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
                case (byte)MAVLink.MAV_DATA_STREAM.MAV_DATA_STREAM_RC_CHANNELS:
                    if (packetspersecondbuild[MAVLINK_MSG_ID_RC_CHANNELS_RAW] < DateTime.Now.AddSeconds(-2))
                        break;
                    pps = packetspersecond[MAVLINK_MSG_ID_RC_CHANNELS_RAW];
                    if (hzratecheck(pps, hzrate))
                    {
                        return;
                    }
                    break;
            }

            //packetspersecond[temp[5]];

            if (pps == 0 && hzrate == 0)
            {
                return;
            }

#if SLV_ADDED
            //log.InfoFormat("Request stream {0} at {1} hz : currently {2}", Enum.Parse(typeof(MAV_DATA_STREAM), id.ToString()), hzrate, pps);
#endif
            getDatastream(id, hzrate);
        }

        // returns true for ok
        bool hzratecheck(double pps, int hzrate)
        {

            if (hzrate == 0 && pps == 0)
            {
                return true;
            }
            else if (hzrate == 1 && pps >= 0.5 && pps <= 2)
            {
                return true;
            }
            else if (hzrate == 3 && pps >= 2 && hzrate < 5)
            {
                return true;
            }
            else if (hzrate == 10 && pps > 5 && hzrate < 15)
            {
                return true;
            }
            else if (hzrate > 15 && pps > 15)
            {
                return true;
            }

            return false;

        }

        void getDatastream(byte id, byte hzrate)
        {
            __mavlink_request_data_stream_t req = new __mavlink_request_data_stream_t();
            req.target_system = sysid;
            req.target_component = compid;

            req.req_message_rate = hzrate;
            req.start_stop = 1; // start
            req.req_stream_id = id; // id

            generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
            generatePacket(MAVLINK_MSG_ID_REQUEST_DATA_STREAM, req);
        }

        /// <summary>
        /// Returns WP count
        /// </summary>
        /// <returns></returns>
        public byte getWPCount()
        {
            MainV2.givecomport = true;
            byte[] buffer;
#if MAVLINK10
            __mavlink_mission_request_list_t req = new __mavlink_mission_request_list_t();

            req.target_system = sysid;
            req.target_component = compid;

            // request list
            generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            int retrys = 6;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("getWPCount Retry " + retrys + " - giv com " + MainV2.givecomport);
                        generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                    throw new Exception("Timeout on read - getWPCount");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_COUNT)
                    {



                        var count = buffer.ByteArrayToStructure<__mavlink_mission_count_t>(6);


                        log.Info("wpcount: " + count.count);
                        MainV2.givecomport = false;
                        return (byte)count.count; // should be ushort, but apm has limited wp count < byte
                    }
                    else
                    {
                        log.Info(DateTime.Now + " PC wpcount " + buffer[5] + " need " + MAVLINK_MSG_ID_MISSION_COUNT + " " + this.BaseStream.BytesToRead);
                    }
                }
            }
#else

            __mavlink_waypoint_request_list_t req = new __mavlink_waypoint_request_list_t();

            req.target_system = sysid;
            req.target_component = compid;

            // request list
            generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST, req);

            DateTime start = DateTime.Now;
            int retrys = 6;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("getWPCount Retry " + retrys + " - giv com " + MainV2.givecomport);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST_LIST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    //return (byte)int.Parse(param["WP_TOTAL"].ToString());
                    throw new Exception("Timeout on read - getWPCount");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_COUNT)
                    {

                        log.Info("wpcount: " + buffer[9]);
                        MainV2.givecomport = false;
                        return buffer[9]; // should be ushort, but apm has limited wp count < byte
                    }
                    else
                    {
                        log.Info(DateTime.Now + " PC wpcount " + buffer[5] + " need " + MAVLINK_MSG_ID_WAYPOINT_COUNT + " " + this.BaseStream.BytesToRead);
                    }
                }
            }

#endif
        }
        /// <summary>
        /// Gets specfied WP
        /// </summary>
        /// <param name="index"></param>
        /// <returns>WP</returns>
        public Locationwp getWP(ushort index)
        {
            MainV2.givecomport = true;
            Locationwp loc = new Locationwp();
#if MAVLINK10
            __mavlink_mission_request_t req = new __mavlink_mission_request_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.seq = index;

            //Console.WriteLine("getwp req "+ DateTime.Now.Millisecond);

            // request
            generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(800) > DateTime.Now)) // apm times out after 1000ms
                {
                    if (retrys > 0)
                    {
                        log.Info("getWP Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_REQUEST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getWP");
                }
                //Console.WriteLine("getwp read " + DateTime.Now.Millisecond);
                byte[] buffer = readPacket();
                //Console.WriteLine("getwp readend " + DateTime.Now.Millisecond);
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_ITEM)
                    {
                        //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);


                        //Array.Copy(buffer, 6, buffer, 0, buffer.Length - 6);

                        var wp = buffer.ByteArrayToStructure<__mavlink_mission_item_t>(6);


#else

            __mavlink_waypoint_request_t req = new __mavlink_waypoint_request_t();

            req.target_system = sysid;
            req.target_component = compid;

            req.seq = index;

            //Console.WriteLine("getwp req "+ DateTime.Now.Millisecond);

            // request
            generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST, req);

            DateTime start = DateTime.Now;
            int retrys = 5;

            while (true)
            {
                if (!(start.AddMilliseconds(800) > DateTime.Now)) // apm times out after 1000ms
                {
                    if (retrys > 0)
                    {
                        log.Info("getWP Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_REQUEST, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getWP");
                }
                //Console.WriteLine("getwp read " + DateTime.Now.Millisecond);
                byte[] buffer = readPacket();
                //Console.WriteLine("getwp readend " + DateTime.Now.Millisecond);
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT)
                    {
                        //Console.WriteLine("getwp ans " + DateTime.Now.Millisecond);
                        __mavlink_waypoint_t wp = buffer.ByteArrayToStructure<__mavlink_waypoint_t>(6);

#endif

                        loc.options = (byte)(wp.frame & 0x1);
                        loc.id = (byte)(wp.command);
                        loc.p1 = (wp.param1);
                        loc.p2 = (wp.param2);
                        loc.p3 = (wp.param3);
                        loc.p4 = (wp.param4);

                        loc.alt = ((wp.z));
                        loc.lat = ((wp.x));
                        loc.lng = ((wp.y));
                        
                        if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
                        {
                            switch (loc.id)
                            {					// Switch to map APM command fields inot MAVLink command fields
                                case (byte)MAV_CMD.LOITER_TURNS:
                                case (byte)MAV_CMD.TAKEOFF:
                                case (byte)MAV_CMD.DO_SET_HOME:
                                    //case (byte)MAV_CMD.DO_SET_ROI:
                                    loc.alt = (float)((wp.z));
                                    loc.lat = (float)((wp.x));
                                    loc.lng = (float)((wp.y));
                                    loc.p1 = (float)wp.param1;
                                    break;

                                case (byte)MAV_CMD.CONDITION_CHANGE_ALT:
                                    loc.lat = (int)wp.param1;
                                    loc.p1 = 0;
                                    break;

                                case (byte)MAV_CMD.LOITER_TIME:
                                    if (MainV2.APMFirmware == MainV2.Firmwares.ArduPlane)
                                    {
                                        loc.p1 = (byte)(wp.param1 / 10);	// APM loiter time is in ten second increments
                                    }
                                    else
                                    {
                                        loc.p1 = (byte)wp.param1;
                                    }
                                    break;

                                case (byte)MAV_CMD.CONDITION_DELAY:
                                case (byte)MAV_CMD.CONDITION_DISTANCE:
                                    loc.lat = (int)wp.param1;
                                    break;

                                case (byte)MAV_CMD.DO_JUMP:
                                    loc.lat = (int)wp.param2;
                                    loc.p1 = (byte)wp.param1;
                                    break;

                                case (byte)MAV_CMD.DO_REPEAT_SERVO:
                                    loc.lng = (int)wp.param4;
                                    goto case (byte)MAV_CMD.DO_CHANGE_SPEED;
                                case (byte)MAV_CMD.DO_REPEAT_RELAY:
                                case (byte)MAV_CMD.DO_CHANGE_SPEED:
                                    loc.lat = (int)wp.param3;
                                    loc.alt = (int)wp.param2;
                                    loc.p1 = (byte)wp.param1;
                                    break;

                                case (byte)MAV_CMD.DO_SET_PARAMETER:
                                case (byte)MAV_CMD.DO_SET_RELAY:
                                case (byte)MAV_CMD.DO_SET_SERVO:
                                    loc.alt = (int)wp.param2;
                                    loc.p1 = (byte)wp.param1;
                                    break;

                                case (byte)MAV_CMD.WAYPOINT:
                                    loc.p1 = (byte)wp.param1;
                                    break;
                            }
                        }
                        
                        log.InfoFormat("getWP {0} {1} {2} {3} {4} opt {5}", loc.id, loc.p1, loc.alt, loc.lat, loc.lng, loc.options);

                        break;
                    }
                    else
                    {
                        log.Info(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }
            MainV2.givecomport = false;
            return loc;
        }

        public object DebugPacket(byte[] datin)
        {
            string text = "";
            return DebugPacket(datin, ref text,true);
        }

        public object DebugPacket(byte[] datin, bool PrintToConsole)
        {
            string text = "";
            return DebugPacket(datin, ref text, PrintToConsole);
        }

        public object DebugPacket(byte[] datin, ref string text)
        {
            return DebugPacket(datin, ref text, true);
        }

        /// <summary>
        /// Print entire decoded packet to console
        /// </summary>
        /// <param name="datin">packet byte array</param>
        /// <returns>struct of data</returns>
        public object DebugPacket(byte[] datin, ref string text, bool PrintToConsole)
        {
            string textoutput;
            try
            {
                if (datin.Length > 5)
                {
                    byte header = datin[0];
                    byte length = datin[1];
                    byte seq = datin[2];
                    byte sysid = datin[3];
                    byte compid = datin[4];
                    byte messid = datin[5];

                    textoutput = string.Format("{0:X2} {1:X2} {2:X2} {3:X2} {4:X2} {5:X2} ", header, length, seq, sysid, compid, messid);

                    object data = Activator.CreateInstance(mavstructs[messid]);

                    MavlinkUtil.ByteArrayToStructure(datin, ref data, 6);

                    Type test = data.GetType();

                    if (PrintToConsole)
                    {

                        textoutput = textoutput + test.Name + " ";

                        foreach (var field in test.GetFields())
                        {
                            // field.Name has the field's name.

                            object fieldValue = field.GetValue(data); // Get value

                            if (field.FieldType.IsArray)
                            {
                                textoutput = textoutput + field.Name + "=";
                                byte[] crap = (byte[])fieldValue;
                                foreach (byte fiel in crap)
                                {
                                    if (fiel == 0)
                                    {
                                        break;
                                    }
                                    else
                                    {
                                        textoutput = textoutput + (char)fiel;
                                    }
                                }
                                textoutput = textoutput + " ";
                            }
                            else
                            {
                                textoutput = textoutput + field.Name + "=" + fieldValue.ToString() + " ";
                            }
                        }
                        textoutput = textoutput + " Len:" + datin.Length + "\r\n";
                        if (PrintToConsole)
                            Console.Write(textoutput);

                        if (text != null)
                            text = textoutput;
                    }

                    return data;
                }
            }
            catch { }

            return null;
        }

        /// <summary>
        /// Sets wp total count
        /// </summary>
        /// <param name="wp_total"></param>
        public void setWPTotal(ushort wp_total)
        {
#if MAVLINK10		
            MainV2.givecomport = true;
            __mavlink_mission_count_t req = new __mavlink_mission_count_t();

            req.target_system = sysid;
            req.target_component = compid; // MAVLINK_MSG_ID_MISSION_COUNT

            req.count = wp_total;

            generatePacket(MAVLINK_MSG_ID_MISSION_COUNT, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(700) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setWPTotal Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_MISSION_COUNT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPTotal");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 9)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_REQUEST)
                    {



                        var request = buffer.ByteArrayToStructure<__mavlink_mission_request_t>(6);

                        if (request.seq == 0)
                        {
                            if (param["WP_TOTAL"] != null)
                                param["WP_TOTAL"] = (float)wp_total - 1;
                            if (param["CMD_TOTAL"] != null)
                                param["CMD_TOTAL"] = (float)wp_total - 1;
                            MainV2.givecomport = false;
                            return;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }
#else
            MainV2.givecomport = true;
            __mavlink_waypoint_count_t req = new __mavlink_waypoint_count_t();

            req.target_system = sysid;
            req.target_component = compid; // MAVLINK_MSG_ID_WAYPOINT_COUNT

            req.count = wp_total;

            generatePacket(MAVLINK_MSG_ID_WAYPOINT_COUNT, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(700) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setWPTotal Retry " + retrys);
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT_COUNT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWPTotal");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 9)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_REQUEST)
                    {
                        __mavlink_waypoint_request_t request = buffer.ByteArrayToStructure<__mavlink_waypoint_request_t>(6);

                        if (request.seq == 0)
                        {
                            if (param["WP_TOTAL"] != null)
                                param["WP_TOTAL"] = (float)wp_total - 1;
                            if (param["CMD_TOTAL"] != null)
                                param["CMD_TOTAL"] = (float)wp_total - 1;
                            MainV2.givecomport = false;
                            return;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC getwp " + buffer[5]);
                    }
                }
            }

#endif
        }

        /// <summary>
        /// Save wp to eeprom
        /// </summary>
        /// <param name="loc">location struct</param>
        /// <param name="index">wp no</param>
        /// <param name="frame">global or relative</param>
        /// <param name="current">0 = no , 2 = guided mode</param>
        public void setWP(Locationwp loc, ushort index, MAV_FRAME frame, byte current)
        {
            MainV2.givecomport = true;
#if MAVLINK10
            __mavlink_mission_item_t req = new __mavlink_mission_item_t();
#else
            __mavlink_waypoint_t req = new __mavlink_waypoint_t();
#endif

            req.target_system = sysid;
            req.target_component = compid; // MAVLINK_MSG_ID_MISSION_ITEM

            req.command = loc.id;
            req.param1 = loc.p1;

            req.current = current;

            req.frame = (byte)frame;
            req.y = (float)(loc.lng);
            req.x = (float)(loc.lat);
            req.z = (float)(loc.alt);

            req.param1 = loc.p1;
            req.param2 = loc.p2;
            req.param3 = loc.p3;
            req.param4 = loc.p4;
            /*
            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                switch (loc.id)
                {					// Switch to map APM command fields inot MAVLink command fields
                    case (byte)MAV_CMD.LOITER_TURNS:
                    case (byte)MAV_CMD.TAKEOFF:
                        req.param1 = loc.p1;
                        break;
                    case (byte)MAV_CMD.DO_SET_HOME:
                        req.param1 = loc.p1;
                        break;

                    case (byte)MAV_CMD.CONDITION_CHANGE_ALT:
                        req.param1 = loc.lat;
                        req.x = 0;
                        req.y = 0;
                        break;

                    case (byte)MAV_CMD.LOITER_TIME:
                        req.param1 = loc.p1 * 10;	// APM loiter time is in ten second increments
                        break;

                    case (byte)MAV_CMD.CONDITION_DELAY:
                    case (byte)MAV_CMD.CONDITION_DISTANCE:
                        req.param1 = loc.lat;
                        break;

                    case (byte)MAV_CMD.DO_JUMP:
                        req.param2 = loc.lat;
                        req.param1 = loc.p1;
                        break;

                    case (byte)MAV_CMD.DO_REPEAT_SERVO:
                        req.param4 = loc.lng;
                        goto case (byte)MAV_CMD.DO_CHANGE_SPEED;
                    case (byte)MAV_CMD.DO_REPEAT_RELAY:
                    case (byte)MAV_CMD.DO_CHANGE_SPEED:
                        req.param3 = loc.lat;
                        req.param2 = loc.alt;
                        req.param1 = loc.p1;
                        break;

                    case (byte)MAV_CMD.DO_SET_PARAMETER:
                    case (byte)MAV_CMD.DO_SET_RELAY:
                    case (byte)MAV_CMD.DO_SET_SERVO:
                        req.param2 = loc.alt;
                        req.param1 = loc.p1;
                        break;
                }
            }
            */
            req.seq = index;

            log.InfoFormat("setWP {6} frame {0} cmd {1} p1 {2} x {3} y {4} z {5}", req.frame, req.command, req.param1, req.x, req.y, req.z, index);

            // request
#if MAVLINK10
            generatePacket(MAVLINK_MSG_ID_MISSION_ITEM, req);
#else
            generatePacket(MAVLINK_MSG_ID_WAYPOINT, req);
#endif

            DateTime start = DateTime.Now;
            int retrys = 6;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("setWP Retry " + retrys);
#if MAVLINK10
            generatePacket(MAVLINK_MSG_ID_MISSION_ITEM, req);
#else
                        generatePacket(MAVLINK_MSG_ID_WAYPOINT, req);
#endif
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - setWP");
                }
                byte[] buffer = readPacket();
                if (buffer.Length > 5)
                {
#if MAVLINK10
                    if (buffer[5] == MAVLINK_MSG_ID_MISSION_ACK)
                    {


                        var ans = buffer.ByteArrayToStructure<__mavlink_mission_ack_t>(6);


                        log.Info("set wp " + index + " ACK 47 : " + buffer[5] + " ans " + Enum.Parse(typeof(MAV_MISSION_RESULT), ans.type.ToString()));
                        break;
                    }
                    else if (buffer[5] == MAVLINK_MSG_ID_MISSION_REQUEST)
                    {
                        var ans = buffer.ByteArrayToStructure<__mavlink_mission_request_t>(6);




                        if (ans.seq == (index + 1))
                        {
                            log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer[5]);
                            MainV2.givecomport = false;
                            break;
                        }
                        else
                        {
                            log.Info("set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " + buffer[5] + " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                            //break;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC setwp " + buffer[5]);
                    }
#else
                    if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_ACK)
                    { //__mavlink_waypoint_request_t
                        log.Info("set wp " + index + " ACK 47 : " + buffer[5]);
                        break;
                    }
                    else if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_REQUEST)
                    {
                        __mavlink_waypoint_request_t ans = buffer.ByteArrayToStructure<__mavlink_waypoint_request_t>(6);

                        if (ans.seq == (index + 1))
                        {
                            log.Info("set wp doing " + index + " req " + ans.seq + " REQ 40 : " + buffer[5]);
                            MainV2.givecomport = false;
                            break;
                        }
                        else
                        {
                            log.InfoFormat("set wp fail doing " + index + " req " + ans.seq + " ACK 47 or REQ 40 : " + buffer[5] + " seq {0} ts {1} tc {2}", req.seq, req.target_system, req.target_component);
                            //break;
                        }
                    }
                    else
                    {
                        //Console.WriteLine(DateTime.Now + " PC setwp " + buffer[5]);
                    }
#endif
                }
            }
        }

        public void setMountConfigure(MAV_MOUNT_MODE mountmode, bool stabroll, bool stabpitch, bool stabyaw)
        {
            __mavlink_mount_configure_t req = new __mavlink_mount_configure_t();

            req.target_system = sysid;
            req.target_component = compid;
            req.mount_mode = (byte)mountmode;
            req.stab_pitch = (stabpitch == true) ? (byte)1 : (byte)0;
            req.stab_roll = (stabroll == true) ? (byte)1 : (byte)0;
            req.stab_yaw = (stabyaw == true) ? (byte)1 : (byte)0;

            generatePacket(MAVLINK_MSG_ID_MOUNT_CONFIGURE, req);
            System.Threading.Thread.Sleep(20);
            generatePacket(MAVLINK_MSG_ID_MOUNT_CONFIGURE, req);
        }

        public void setMountControl(double pa, double pb, double pc, bool islatlng)
        {
            __mavlink_mount_control_t req = new __mavlink_mount_control_t();

            req.target_system = sysid;
            req.target_component = compid;
            if (!islatlng)
            {
                req.input_a = (int)pa;
                req.input_b = (int)pb;
                req.input_c = (int)pc;
            }
            else
            {
                req.input_a = (int)(pa * 10000000.0);
                req.input_b = (int)(pb * 10000000.0);
                req.input_c = (int)(pc * 100.0);
            }

            generatePacket(MAVLINK_MSG_ID_MOUNT_CONTROL, req);
            System.Threading.Thread.Sleep(20);
            generatePacket(MAVLINK_MSG_ID_MOUNT_CONTROL, req);
        }

        public void sendCDnR(__mavlink_cdnr_controller_t req)
        {
            try
            {
                MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_CDNR_CONTROLLER, req);
                //                    System.Threading.Thread.Sleep(10);
                //                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_CDNR_CONTROLLER, req);
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to send new CDnR Message"); }

        }
        public void sendTrafficSimTiming(__mavlink_traffic_sim_timing_t req)
        {
            try
            {
                MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_TRAFFIC_SIM_TIMING, req);
                //                    System.Threading.Thread.Sleep(10);
                //                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_CDNR_CONTROLLER, req);
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to send TrafficSimTiming Message"); }

        }

        public void setMode(string modein)
        {
#if MAVLINK10
            try
            {
                MAVLink.__mavlink_set_mode_t mode = new MAVLink.__mavlink_set_mode_t();

                if (Common.translateMode(modein, ref mode))
                {
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to change Modes"); }
#else
            try
            {
                MAVLink.__mavlink_set_nav_mode_t navmode = new MAVLink.__mavlink_set_nav_mode_t();

                MAVLink.__mavlink_set_mode_t mode = new MAVLink.__mavlink_set_mode_t();

                if (Common.translateMode(modein, ref navmode, ref mode))
                {
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_NAV_MODE, navmode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_NAV_MODE, navmode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                    System.Threading.Thread.Sleep(10);
                    MainV2.comPort.generatePacket((byte)MAVLink.MAVLINK_MSG_ID_SET_MODE, mode);
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to change Modes"); }

#endif
        }

        /// <summary>
        /// used for last bad serial characters
        /// </summary>
        byte[] lastbad = new byte[2];

        /// <summary>
        /// Serial Reader to read mavlink packets. POLL method
        /// </summary>
        /// <returns></returns>
        public byte[] readPacket()
        {
            byte[] temp = new byte[300];
            int count = 0;
            int length = 0;
            int readcount = 0;
            lastbad = new byte[2];

            BaseStream.ReadTimeout = 1200; // 1200 ms between chars - the gps detection requires this.

            DateTime start = DateTime.Now;

            lock (readlock)
            {

                while (BaseStream.IsOpen || logreadmode)
                {
                    try
                    {
                        if (readcount > 300)
                        {
                            log.Info("MAVLink readpacket No valid mavlink packets");
                            break;
                        }
                        readcount++;
                        if (logreadmode)
                        {
                            try
                            {
                                if (logplaybackfile.BaseStream.Position == 0)
                                {
                                    if (logplaybackfile.PeekChar() == '-')
                                    {
                                        oldlogformat = true;
                                    }
                                    else
                                    {
                                        oldlogformat = false;
                                    }
                                }
                            }
                            catch { oldlogformat = false; }

                            if (oldlogformat)
                            {
                                temp = readlogPacket(); //old style log
                            }
                            else
                            {
                                temp = readlogPacketMavlink();
                            }
                        }
                        else
                        {
                            MainV2.cs.datetime = DateTime.Now;

                            DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                            while (BaseStream.BytesToRead <= 0)
                            {
                                if (DateTime.Now > to)
                                {
                                    log.InfoFormat("MAVLINK: S wait time out btr {0} len {1}", BaseStream.BytesToRead, length);
                                    throw new Exception("Timeout");
                                }
                                System.Threading.Thread.Sleep(1);
                            }
                            if (BaseStream.IsOpen)
                                temp[count] = (byte)BaseStream.ReadByte();
                        }
                    }
                    catch (Exception e) { log.Info("MAVLink readpacket read error: " + e.Message); break; }

                    if (temp[0] != 254 && temp[0] != 'U' || lastbad[0] == 'I' && lastbad[1] == 'M' || lastbad[1] == 'G' || lastbad[1] == 'A') // out of sync "AUTO" "GUIDED" "IMU"
                    {
                        if (temp[0] >= 0x20 && temp[0] <= 127 || temp[0] == '\n' || temp[0] == '\r')
                        {
                            TCPConsole.Write(temp[0]);
                            Console.Write((char)temp[0]);
                        }
                        count = 0;
                        lastbad[0] = lastbad[1];
                        lastbad[1] = temp[0];
                        temp[1] = 0;
                        continue;
                    }
                    // reset count on valid packet
                    readcount = 0;

                    if (temp[0] == 'U' || temp[0] == 254)
                    {
                        length = temp[1] + 6 + 2 - 2; // data + header + checksum - U - length
                        if (count >= 5 || logreadmode)
                        {
                            /*
                            if (sysid != 0)
                            {
                                // If (temp[3] == 103) means the message originates from Beagle Board
                                // and it will be accepted
                                //if ((sysid != temp[3] && (byte)103 != temp[3]) || compid != temp[4])
                                if ((temp[3]!=sysid && temp[3]!=(byte)103) || temp[4]!=compid)
                                {
                                    log.InfoFormat("Mavlink Bad Packet (not addressed to this MAV) got {0:x2} {1:x2} {2:x2} [{3:x2}:{4:x2}] {5:x2} vs [{6:x2}|{7:x2}:{8:x2}]", temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], sysid,(byte)103, compid);
                                    return new byte[0];
                                }
                            }
                            */

                            try
                            {
                                if (logreadmode)
                                {

                                }
                                else
                                {
                                    DateTime to = DateTime.Now.AddMilliseconds(BaseStream.ReadTimeout);

                                    while (BaseStream.BytesToRead < (length - 4))
                                    {
                                        if (DateTime.Now > to)
                                        {
                                            log.InfoFormat("MAVLINK: L wait time out btr {0} len {1}", BaseStream.BytesToRead, length);
                                            break;
                                        }
                                        //Console.WriteLine("data " + 0 + " " + length + " aval " + BaseStream.BytesToRead);
                                    }
                                    if (BaseStream.IsOpen)
                                    {
                                        int read = BaseStream.Read(temp, 6, length - 4);
                                    }
                                }
                                //Console.WriteLine("data " + read + " " + length + " aval " + this.BytesToRead);
                                count = length + 2;
                            }
                            catch { break; }
                            break;
                        }
                    }

                    count++;
                    if (count == 299)
                        break;
                }
            }// end readlock

            if (count>0)
                Array.Resize<byte>(ref temp, count);

            if (packetlosttimer.AddSeconds(10) < DateTime.Now)
            {
                packetlosttimer = DateTime.Now;
                packetslost = (int)(packetslost * 0.8f);
                packetsnotlost = (int)(packetsnotlost * 0.8f);
            }

            MainV2.cs.linkqualitygcs = (ushort)((packetsnotlost / (packetsnotlost + packetslost)) * 100);

            if (bpstime.Second != DateTime.Now.Second && !logreadmode)
            {
                //                Console.Write("bps {0} loss {1} left {2} mem {3}      \n", bps1, synclost, BaseStream.BytesToRead, System.GC.GetTotalMemory(false));
                bps2 = bps1; // prev sec
                bps1 = 0; // current sec
                bpstime = DateTime.Now;
            }

            bps1 += temp.Length;

            bps = (bps1 + bps2) / 2;

            // Sniff packets from GroundStation
            if (temp.Length >= 5 && temp[3] == 255 && logreadmode) // gcs packet
            {
                getWPsfromstream(ref temp);
                return temp;// new byte[0];
            }

            if (temp.Length < temp[1] + 8)
            {
                log.InfoFormat("Mavlink Bad Packet (Incomplete Message) Received {0}: msg({1}).len={2}, header says {3}", temp.Length, temp[5], MAVLINK_MESSAGE_LENGTHS[temp[5]], temp[1]);
                return new byte[0];
            }

            if (temp[1] != MAVLINK_MESSAGE_LENGTHS[temp[5]])
            {
                if (MAVLINK_MESSAGE_LENGTHS[temp[5]] == 0) // pass for unknown packets
                {

                }
                else
                {
                    log.InfoFormat("Mavlink Bad Packet (Len Fail) Received {0}: msg({1}).len={2}, header says {3}", temp.Length, temp[5], MAVLINK_MESSAGE_LENGTHS[temp[5]], temp[1]);
#if MAVLINK10
                if (temp.Length == 11 && temp[0] == 'U' && temp[5] == 0)
                    throw new Exception("Mavlink 0.9 Heartbeat, Please upgrade your AP, This planner is for Mavlink 1.0\n\n");
#endif
                    return new byte[0];
                }
            }

            ushort crc = MavlinkCRC.crc_calculate(temp, temp.Length - 2);

            if (temp.Length > 5 && temp[0] == 254)
            {
                crc = MavlinkCRC.crc_accumulate(MAVLINK_MESSAGE_CRCS[temp[5]], crc);
            }

            if (temp.Length < 5 || temp[temp.Length - 1] != (crc >> 8) || temp[temp.Length - 2] != (crc & 0xff))
            {
                //int packetno = 0;
                //if (temp.Length > 5)
                //{
                //    packetno = temp[5];
                //}
                //log.InfoFormat("Mavlink Bad Packet (crc fail) len {0} crc {1} pkno {2}", temp.Length, crc, packetno);
                string mess;
                mess = "MavLink CRC Error:[ ";
                for (int i = 0; i < 6; i++)             mess += string.Format("{0:x2} ", temp[i]);  mess += "] ";
                for (int i = 6; i < temp.Length; i++)   mess += string.Format("{0:x2}" , temp[i]);  mess += string.Format(" : [{0:x4}]", crc);
                //mess = string.Format("Mavlink CRC Error: {0:x2} {1:x2} {2:x2} {3:x2} {4:x2} {5:x2}:[{7:x2}{8:x2}]:[{6:x4}]"
                //                 , temp[0], temp[1], temp[2], temp[3], temp[4], temp[5], crc, temp[temp[1] + 7], temp[temp[1] + 6]);
                log.InfoFormat(mess);
                //DebugPacket(temp);
                return new byte[0];
            }

            try
            {

                if ((temp[0] == 'U' || temp[0] == 254) && temp.Length >= temp[1])
                {
                    if (temp[2] != ((recvpacketcount + 1) % 0x100))
                    {
                        synclost++; // actualy sync loss's

                        if (temp[2] < ((recvpacketcount + 1) % 0x100))
                        {
                            packetslost += 0x100 - recvpacketcount + temp[2];
                        }
                        else
                        {
                            packetslost += temp[2] - recvpacketcount;
                        }

                        log.InfoFormat("lost {0} pkts {1}", temp[2], (int)packetslost);
                    }

                    packetsnotlost++;

                    recvpacketcount = temp[2];

                    //MAVLINK_MSG_ID_GPS_STATUS
                    //if (temp[5] == MAVLINK_MSG_ID_GPS_STATUS)

                    //                    Console.Write(temp[5] + " " + DateTime.Now.Millisecond + " " + packetspersecond[temp[5]] + " " + (DateTime.Now - packetspersecondbuild[temp[5]]).TotalMilliseconds + "     \n");

                    if (double.IsInfinity(packetspersecond[temp[5]]))
                        packetspersecond[temp[5]] = 0;

                    packetspersecond[temp[5]] = (((1000 / ((DateTime.Now - packetspersecondbuild[temp[5]]).TotalMilliseconds) + packetspersecond[temp[5]]) / 2));

                    packetspersecondbuild[temp[5]] = DateTime.Now;

                    //Console.WriteLine("Packet {0}",temp[5]);
                    // store packet history
                    lock (objlock)
                    {
                        if (!GCS_ID_set || (temp[3] == sysid) && (temp[4] == compid)) // message comes from sysid addressed to compid
                            packets[temp[5]] = temp;
                        else
                            Console.WriteLine("GCS Reject: m{0} s{1} c{2}", temp[5],temp[3],temp[4]);
                    }

                    if (debugmavlink)
                        DebugPacket(temp);

#if SLV_ADDED
                    if (temp[5] == MAVLink.MAVLINK_MSG_ID_STATUSTEXT) // status text
                    {
                        string logdata = Encoding.ASCII.GetString(temp, 7, temp.Length - 7);
                        int ind = logdata.IndexOf('\0');
                        if (ind != -1)
                            logdata = logdata.Substring(0, ind);
                        //log.Info(DateTime.Now + " " + logdata);
                        log.Info(logdata);

                        if (logdata.Contains("GIT:"))
                            MainV2.GIT_ID = logdata;

                        if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                        {
                            //Console.WriteLine("SPEAK: {0}", logdata);
                            if (!MainV2.b_traffic_alert)
                                MainV2.talk.SpeakAsync(logdata);
                        }

                    }
#endif
                    try
                    {
                        if (logfile != null)
                        {
                            lock (logwritelock)
                            {
                                byte[] datearray = BitConverter.GetBytes((UInt64)((DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds * 1000)); //ASCIIEncoding.ASCII.GetBytes(DateTime.Now.ToBinary() + ":");
                                Array.Reverse(datearray);
                                logfile.Write(datearray, 0, datearray.Length);
                                logfile.Write(temp, 0, temp.Length);

                                //string textoutput; // Telemetry from AP to Ground Station
                                //textoutput = string.Format("APPacket {0:x2} {1:x2} {2:x2} {3:x2} {4:x2} {5:x2}\r\n"
                                //                          , temp[0], temp[1], temp[2], temp[3], temp[4], temp[5]);
                                //Console.Write(textoutput);
                            }
                        }

                    }
                    catch { }

                    // Sniff packets from Beagle Board
                    if (temp[3] == 103) // Beagle packet
                    {
                        Console.WriteLine("Packet from Beagle Board");
                        getCandidateWPsfromstream(ref temp);
                        return new byte[0];
                    }
                    else
                        getWPsfromstream(ref temp);
                }
            }
            catch { }

            lastvalidpacket = DateTime.Now;

            //            Console.Write((DateTime.Now - start).TotalMilliseconds.ToString("00.000") + "\t" + temp.Length + "     \r");

            return temp;
        }

        /// <summary>
        /// Used to extract candidate mission from Stratway Packet
        /// </summary>
        /// <param name="buffer">packet</param>
        void getCandidateWPsfromstream(ref byte[] buffer)
        {
#if MAVLINK10
            if (buffer[5] == MAVLINK_MSG_ID_MISSION_COUNT)
            {
                // clear old
                candidate_wps = new PointLatLngAlt[candidate_wps.Length];
            }

            if (buffer[5] == MAVLink.MAVLINK_MSG_ID_MISSION_ITEM)
            {
                __mavlink_mission_item_t wp = buffer.ByteArrayToStructure<__mavlink_mission_item_t>(6);
            
                if (candidate_wps == null)
                    candidate_wps = new PointLatLngAlt[candidate_wps.Length];

                if (wp.seq > candidate_wps.Length - 1)
                    Console.WriteLine("Invalid wpno {0} > List Size {1}", wp.seq, candidate_wps.Length);
                else
                {
                    candidate_wps[wp.seq] = new PointLatLngAlt(wp.x, wp.y, wp.z, wp.seq.ToString());
                    Console.WriteLine("Receiving Waypoint {0}", wp.seq);
                }
            }
#else
            switch (buffer[5])
            {
                case MAVLINK_MSG_ID_TRAFFIC_DATA:
                    // Receiving traffic data via MavLink
                    __mavlink_traffic_data_t traf = buffer.ByteArrayToStructure<__mavlink_traffic_data_t>(6);

                    byte[] achar = {0, 0};
                    char n0,n1,n2,n3,n4,n5,n6,n7;
                    achar[0] = traf.ID[0]; achar[1] = 0;
                    n0 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[1]; achar[1] = 0;
                    n1 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[2]; achar[1] = 0;
                    n2 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[3]; achar[1] = 0;
                    n3 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[4]; achar[1] = 0;
                    n4 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[5]; achar[1] = 0;
                    n5 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[6]; achar[1] = 0;
                    n6 = BitConverter.ToChar(achar, 0);
                    achar[0] = traf.ID[7]; achar[1] = 0;
                    n7 = BitConverter.ToChar(achar, 0);
                    string name = n0.ToString() + n1.ToString() + n2.ToString() + n3.ToString() + n4.ToString() + n5.ToString() + n6.ToString() + n7.ToString();
                    //MainV2.ID = traf.ID;
                    //MainV2.Latitude = traf.lat;
                    //MainV2.Longitude = traf.lon;
                    //MainV2.Elevation = traf.alt;
                    //MainV2.Vx = traf.roll;
                    //MainV2.Vy = traf.pitch;
                    //MainV2.Vz = traf.heading;
                    //MainV2.Own = traf.ownship_flag;
                    bool newTraffic = true;
                    int trafficIdx = 0;
                    for (trafficIdx = 0; ((trafficIdx < MainV2.trafficTable.Count) && newTraffic); trafficIdx++)
                    {
                        if (MainV2.trafficTable[trafficIdx].ID.Equals(traf.ID))
                        {
                            newTraffic = false;
                        }
                    }
                    if (newTraffic)
                    {
                        MainV2.trafficTable.Add(new MainV2.Aircraft(name, traf.lat, traf.lon, traf.alt, traf.heading));
                        //Console.Write("                                              Added  : "); Console.WriteLine(aname);
                    }
                    else
                    {
                        MainV2.trafficTable[trafficIdx - 1].Update(traf.lat, traf.lon, traf.alt, traf.heading, traf.ownship_flag);
                        //Console.Write("                                              Updated: "); Console.WriteLine(MainV2.trafficTable[trafficIdx - 1].ID);
                    }

                    break;

                case MAVLINK_MSG_ID_CDNR_CONTROLLER:
                    // Conflict detected, new plan with wc.count points generated
                    __mavlink_cdnr_controller_t cdnr = buffer.ByteArrayToStructure<__mavlink_cdnr_controller_t>(6);

                    if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True" && (cdnr.h_flag == 1 || cdnr.a_flag == 1 || cdnr.s_flag == 1))
                    {
                        if (MainV2.b_traffic_alert)
                        {
                            string msg = " " + cdnr.new_heading;
                            MainV2.talk.SpeakAsync(msg);
                        }
                        else
                        {
                            MainV2.talk.SpeakAsyncCancelAll();
                            string msg = "Alert, Diverted to Head ing " + cdnr.new_heading;
                            MainV2.talk.SpeakAsync(msg);
                            MainV2.b_traffic_alert = true;
                        }
                    }
                    else
                    {
                        string msg = "Resuming Flight Plan";
                        MainV2.talk.SpeakAsync(msg);
                        MainV2.b_traffic_alert = false;
                    }
                    break;

                case MAVLINK_MSG_ID_WAYPOINT_COUNT:
                    // Conflict detected, new plan with wc.count points generated
                    __mavlink_waypoint_count_t wc = buffer.ByteArrayToStructure<__mavlink_waypoint_count_t>(6);

                    cwps = new PointLatLngAlt[wc.count];
                    if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                    {
                        string msg = "Flight Plan Reload";
                        if (!MainV2.b_traffic_alert)
                            MainV2.talk.SpeakAsync(msg);
                    }
                    //Console.WriteLine("Receiving a new flight plan[{0}] from StratWay", wc.count);
                    break;

                case MAVLink.MAVLINK_MSG_ID_WAYPOINT:
                    // Waypoint for new plan received
                    __mavlink_waypoint_t wp = buffer.ByteArrayToStructure<__mavlink_waypoint_t>(6);

                    if (cwps == null)
                        cwps = new PointLatLngAlt[cwps.Length];

                    if (wp.seq > cwps.Length - 1)
                        Console.WriteLine("Invalid wpno {0} > List Size {1}", wp.seq, cwps.Length);
                    else
                    {
                        if (wp.seq == 0)
                            cwps[wp.seq] = new PointLatLngAlt(wp.x, wp.y, wp.z, "Home");
                        else
                            cwps[wp.seq] = new PointLatLngAlt(wp.x, wp.y, wp.z, wp.seq.ToString("c{0}"));

                        Console.WriteLine("Receiving Waypoint [{0}]", wp.seq);
                    }
                    break;

                case MAVLINK_MSG_ID_WAYPOINT_SET_CURRENT:
                    // Sets the current waypoint to start navigating to the new plan
                    __mavlink_waypoint_set_current_t cw = buffer.ByteArrayToStructure<__mavlink_waypoint_set_current_t>(6);
                    if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                    {
                        string msg = "Go to Waypoint " + cw.seq;
                        if (!MainV2.b_traffic_alert)
                            MainV2.talk.SpeakAsync(msg);
                    }
                    break;

                case MAVLINK_MSG_ID_WAYPOINT_ACK:
                    // Acknowledge that last waypoint was received
                    if (MainV2.talk != null && MainV2.config["speechenable"] != null && MainV2.config["speechenable"].ToString() == "True")
                    {
                        if (!MainV2.b_traffic_alert)
                            MainV2.talk.SpeakAsync("Plan Acknowledged");
                    }
                    break;
            }

#endif
        }

        /// <summary>
        /// Used to extract mission from log file
        /// </summary>
        /// <param name="buffer">packet</param>
        void getWPsfromstream(ref byte[] buffer)
        {
#if MAVLINK10
            if (buffer[5] == MAVLINK_MSG_ID_MISSION_COUNT)
            {
                // clear old
                wps = new PointLatLngAlt[wps.Length];
            }

            if (buffer[5] == MAVLink.MAVLINK_MSG_ID_MISSION_ITEM)
            {
                __mavlink_mission_item_t wp = buffer.ByteArrayToStructure<__mavlink_mission_item_t>(6);
#else

            if (buffer[5] == MAVLINK_MSG_ID_WAYPOINT_COUNT)
            {
                // clear old
                wps = new PointLatLngAlt[wps.Length];
            }

            if (buffer[5] == MAVLink.MAVLINK_MSG_ID_WAYPOINT)
            {
                __mavlink_waypoint_t wp = buffer.ByteArrayToStructure<__mavlink_waypoint_t>(6);

#endif
                wps[wp.seq] = new PointLatLngAlt(wp.x, wp.y, wp.z, wp.seq.ToString());
            }
        }

        public PointLatLngAlt getFencePoint(int no, ref int total)
        {
            byte[] buffer;

            MainV2.givecomport = true;

            PointLatLngAlt plla = new PointLatLngAlt();
            __mavlink_fence_fetch_point_t req = new __mavlink_fence_fetch_point_t();

            req.idx = (byte)no;
            req.target_component = compid;
            req.target_system = sysid;

            // request point
            generatePacket(MAVLINK_MSG_ID_FENCE_FETCH_POINT, req);

            DateTime start = DateTime.Now;
            int retrys = 3;

            while (true)
            {
                if (!(start.AddMilliseconds(500) > DateTime.Now))
                {
                    if (retrys > 0)
                    {
                        log.Info("getFencePoint Retry " + retrys + " - giv com " + MainV2.givecomport);
                        generatePacket(MAVLINK_MSG_ID_FENCE_FETCH_POINT, req);
                        start = DateTime.Now;
                        retrys--;
                        continue;
                    }
                    MainV2.givecomport = false;
                    throw new Exception("Timeout on read - getFencePoint");
                }

                buffer = readPacket();
                if (buffer.Length > 5)
                {
                    if (buffer[5] == MAVLINK_MSG_ID_FENCE_POINT)
                    {
                        MainV2.givecomport = false;

                        __mavlink_fence_point_t fp = buffer.ByteArrayToStructure<__mavlink_fence_point_t>(6);

                        plla.Lat = fp.lat;
                        plla.Lng = fp.lng;
                        plla.Tag = fp.idx.ToString();

                        total = fp.count;

                        return plla;
                    }
                }
            }
        }

        public bool setFencePoint(byte index, PointLatLngAlt plla, byte fencepointcount)
        {
            __mavlink_fence_point_t fp = new __mavlink_fence_point_t();

            fp.idx = index;
            fp.count = fencepointcount;
            fp.lat = (float)plla.Lat;
            fp.lng = (float)plla.Lng;
            fp.target_component = compid;
            fp.target_system = sysid;

            int retry = 3;

            while (retry > 0)
            {
                generatePacket(MAVLINK_MSG_ID_FENCE_POINT, fp);
                int counttemp = 0;
                PointLatLngAlt newfp = getFencePoint(fp.idx, ref counttemp);

                if (newfp.Lat == plla.Lat && newfp.Lng == fp.lng)
                    return true;
                retry--;
            }

            return false;
        }

        byte[] readlogPacket()
        {
            byte[] temp = new byte[300];

            sysid = 0;

            int a = 0;
            while (a < temp.Length && logplaybackfile.BaseStream.Position != logplaybackfile.BaseStream.Length)
            {
                temp[a] = (byte)logplaybackfile.BaseStream.ReadByte();
                //Console.Write((char)temp[a]);
                if (temp[a] == ':')
                {
                    break;
                }
                a++;
                if (temp[0] != '-')
                {
                    a = 0;
                }
            }

            //Console.Write('\n');

            //Encoding.ASCII.GetString(temp, 0, a);
            string datestring = Encoding.ASCII.GetString(temp, 0, a);
            //Console.WriteLine(datestring);
            long date = Int64.Parse(datestring);
            DateTime date1 = DateTime.FromBinary(date);

            lastlogread = date1;

            int length = 5;
            a = 0;
            while (a < length)
            {
                temp[a] = (byte)logplaybackfile.BaseStream.ReadByte();
                if (a == 1)
                {
                    length = temp[1] + 6 + 2 + 1;
                }
                a++;
            }

            return temp;
        }

        byte[] readlogPacketMavlink()
        {
            byte[] temp = new byte[300];

            sysid = 0;

            //byte[] datearray = BitConverter.GetBytes((ulong)(DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds);

            byte[] datearray = new byte[8];

            logplaybackfile.BaseStream.Read(datearray, 0, datearray.Length);

            Array.Reverse(datearray);

            DateTime date1 = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);

            UInt64 dateint = BitConverter.ToUInt64(datearray, 0);

            date1 = date1.AddMilliseconds(dateint / 1000);

            lastlogread = date1.ToLocalTime();

            MainV2.cs.datetime = lastlogread;

            int length = 5;
            int a = 0;
            while (a < length)
            {
                temp[a] = (byte)logplaybackfile.ReadByte();
                if (temp[0] != 'U' && temp[0] != 254)
                {
                    log.InfoFormat("lost sync byte {0} pos {1}", temp[0], logplaybackfile.BaseStream.Position);
                    a = 0;
                    continue;
                }
                if (a == 1)
                {
                    length = temp[1] + 6 + 2; // 6 header + 2 checksum
                }
                a++;
            }

            return temp;
        }



    }
}