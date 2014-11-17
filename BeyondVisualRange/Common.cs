
#define SLV_ADDED

using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using AGaugeApp;
using System.IO.Ports;
using System.Threading;
using GMap.NET;
using GMap.NET.WindowsForms;
using GMap.NET.WindowsForms.Markers;

using System.Security.Cryptography.X509Certificates;

using System.Net;
using System.Net.Sockets;
using System.Xml; // config file
using System.Runtime.InteropServices; // dll imports
using log4net;
using ZedGraph; // Graphs
using ArdupilotMega;
using ArdupilotMega.GCSViews;
using System.Reflection;

using System.IO;

using System.Drawing.Drawing2D;

namespace ArdupilotMega
{
    public class VerticalProgressBar : HorizontalProgressBar
    {
        protected override CreateParams CreateParams
        {
            get
            {
                CreateParams cp = base.CreateParams;
                cp.Style |= 0x04;
                return cp;
            }
        }
    }

    /// <summary>
    /// Struct as used in Ardupilot
    /// </summary>
    public struct Locationwp
    {
        public byte id;				// command id
        public byte options;
        public float p1;				// param 1
        public float p2;				// param 2
        public float p3;				// param 3
        public float p4;				// param 4
        public float lat;				// Lattitude * 10**7
        public float lng;				// Longitude * 10**7
        public float alt;				// Altitude in centimeters (meters * 100)
    };


    /// <summary>
    /// used to override the drawing of the waypoint box bounding
    /// </summary>
    public class GMapMarkerRect : GMapMarker
    {
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);
        public Pen Pen = new Pen(Brushes.White, 2);
        public Color Color { get { return Pen.Color; } set { Pen.Color = value; } }
        public GMapMarker InnerMarker;
        public int wprad = 0;
        public GMapControl MainMap;

        public GMapMarkerRect(PointLatLng p)
            : base(p)
        {
            Pen.DashStyle = DashStyle.Dash;

            // do not forget set Size of the marker
            // if so, you shall have no event on it ;}
            Size = new System.Drawing.Size(50, 50);
            Offset = new System.Drawing.Point(-Size.Width / 2, -Size.Height / 2 - 20);
        }

#if SLV_nADDED
        public override void OnRender(Graphics g)
        {
            base.OnRender(g);

            if (wprad == 0 || MainMap == null)
                return;

            Matrix temp = g.Transform;
            g.TranslateTransform(LocalPosition.X, LocalPosition.Y);
            g.RotateTransform(-MainMap.Bearing);

            // undo autochange in mouse over
            if (Pen.Color == Color.Blue)
                Pen.Color = Color.White;

            double width = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Width, 0)) * 1000.0);
            double height = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Height, 0)) * 1000.0);
            double m2pixelwidth = MainMap.Width / width;
            double m2pixelheight = MainMap.Height / height;

    #if SLV_ADDED
            float myR = float.Parse(ArdupilotMega.MainV2.config["TXT_WPRad"].ToString());
            g.DrawArc(Pen, -myR * (float)m2pixelwidth - Offset.X, -myR * (float)m2pixelheight - Offset.Y, 2 * myR * (float)m2pixelwidth, 2 * myR * (float)m2pixelheight, 0, 360);
            //GPoint loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * (int)float.Parse(ArdupilotMega.MainV2.config["TXT_WPRad"].ToString()) * 2)), LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);
            //g.DrawArc(Pen,new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);

            if (int.Parse(Tag.ToString()) == (int)ArdupilotMega.MainV2.cs.wpno)
            {
                //float myA = MainV2.cs.target_bearing + 90;
                float myA = (MainV2.cs.target_bearing) + 90;
                float myTR = (float)(Math.Pow(MainV2.cs.groundspeed, 2) / 9.8);
                Pen myPen = new Pen(Color.Cyan, 4); myPen.DashStyle = DashStyle.Dash;
                GPoint loc = new GPoint((int)(m2pixelwidth * ArdupilotMega.MainV2.cs.notUsed * Math.Cos(myA * deg2rad)), (int)(m2pixelheight * ArdupilotMega.MainV2.cs.notUsed * Math.Sin(myA * deg2rad)));// MainMap.FromLatLngToLocal(wpradposition);
                int fromAngle = (int)ArdupilotMega.MainV2.cs.target_bearing - (int)MainMap.Bearing - 90;
                int spanAngle = 60 * (int)float.Parse(ArdupilotMega.MainV2.config["TXT_WPRad"].ToString()) / wprad;
                g.DrawArc(myPen, new System.Drawing.Rectangle(loc.X - Offset.X - (int)(m2pixelwidth * myTR), loc.Y - Offset.Y - (int)(m2pixelheight * myTR), (int)(2 * myTR * m2pixelwidth), (int)(2 * myTR * m2pixelheight)), fromAngle-spanAngle/2, spanAngle);
            }
            //                else
            //                    g.DrawArc(Pen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);
    #else
            GPoint loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * wprad * 2)), LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);
            g.DrawArc(Pen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);
    #endif

            g.Transform = temp;
        }
#else
        public override void OnRender(Graphics g)
        {
            base.OnRender(g);

            if (wprad == 0 || MainMap == null)
                return;

            // undo autochange in mouse over
            if (Pen.Color == Color.Blue)
                Pen.Color = Color.White;

            double width = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Width, 0)) * 1000.0);
            double height = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Height, 0)) * 1000.0);
            double m2pixelwidth = MainMap.Width / width;
            double m2pixelheight = MainMap.Height / height;

    #if SLV_ADDED
            int TagNo = 0;

            if (Tag != null)
            {
                if (int.Parse(Tag.ToString()) == 0)
                    TagNo = 0;
                else
                    TagNo = int.Parse(InnerMarker.Tag.ToString());
            }
            
            GPoint loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * (int)float.Parse(ArdupilotMega.MainV2.config["TXT_WPRad"].ToString()) * 2))
                                        , LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);
            g.DrawArc(Pen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);

            if ((TagNo > 0) && (TagNo == (int)ArdupilotMega.MainV2.cs.wpno))
            {
                Pen myPen = new Pen(Color.Cyan, 4);
                myPen.DashStyle = DashStyle.Dash;
                loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * (int)ArdupilotMega.MainV2.cs.wp_radius * 2)), LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);
                int fromAngle = 90 + (int)ArdupilotMega.MainV2.cs.target_bearing - (int)MainMap.Bearing;
                int spanAngle = 45; // 60 * (int)float.Parse(ArdupilotMega.MainV2.config["TXT_WPRad"].ToString()) / wprad;
                g.DrawArc(myPen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), fromAngle - spanAngle / 2, spanAngle);
            }
//                else
//                    g.DrawArc(Pen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);
    #else
            GPoint loc = new GPoint((int)(LocalPosition.X - (m2pixelwidth * wprad * 2)), LocalPosition.Y);// MainMap.FromLatLngToLocal(wpradposition);
            g.DrawArc(Pen, new System.Drawing.Rectangle(LocalPosition.X - Offset.X - (Math.Abs(loc.X - LocalPosition.X) / 2), LocalPosition.Y - Offset.Y - Math.Abs(loc.X - LocalPosition.X) / 2, Math.Abs(loc.X - LocalPosition.X), Math.Abs(loc.X - LocalPosition.X)), 0, 360);
    #endif

        }
#endif
    }

    public class GMapMarkerPlane : GMapMarker
    {
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::ArdupilotMega.Properties.Resources.planeicon.Width, global::ArdupilotMega.Properties.Resources.planeicon.Height);
        float heading = 0;
        float cog = -1;
        float target = -1;
        float nav_bearing = -1;
        float alt = 0;

#if SLV_ADDED
        bool ownship = false;
        public GMapControl MainMap;
        public GMapMarkerPlane(PointLatLng p, float heading, float cog, float nav_bearing, float target, GMapControl map)
#else
            public GMapMarkerPlane(PointLatLng p, float heading, float cog, float nav_bearing,float target)
#endif
            : base(p)
        {
            this.heading = heading;
            this.cog = cog;
            this.target = target;
            this.nav_bearing = nav_bearing;
            Size = SizeSt;
            ownship = true;
#if SLV_ADDED
            MainMap = map;
#endif
        }

#if SLV_ADDED
        public GMapMarkerPlane(PointLatLng p, float alt, float heading, GMapControl map)
            : base(p)
        {
            this.heading = heading;
            this.alt = alt;
            Size = SizeSt;
            ownship = false;
            MainMap = map;
        }
#endif
        public Color slvSOC_Color(float soc)
        {
            float WarningThreshold   = 50.0f;
            float EmergencyThreshold = 30.0f;
            int mrColor = soc > WarningThreshold ?   0 : soc > EmergencyThreshold ? 255 : 204;    // (int)(255.0 * (1 - MainV2.soc / 100));
            int mgColor = soc > WarningThreshold ? 128 : soc > EmergencyThreshold ? 255 :   0;    // (int)(255.0 * (MainV2.soc / 100));
            int mbColor = soc > WarningThreshold ?   0 : soc > EmergencyThreshold ?   0 :   0;    // (int)(255.0 * (MainV2.soc / 100));
            return Color.FromArgb(96, mrColor, mgColor, mbColor);
        }
        public void slvDrawPieBattery(Graphics g, Point pt, int outerR, float soc)
        {
            Brush OB = new SolidBrush(Color.FromArgb(128, 255, 255, 255));

            int innerR = outerR - 4;
            Size Osz = new Size(outerR, outerR);
            Size Isz = new Size(innerR, innerR);

            Point Opt = new Point(pt.X - Osz.Width / 2, pt.Y - Osz.Height / 2);
            Point Ipt = new Point(pt.X - Isz.Width / 2, pt.Y - Isz.Height / 2);
            g.FillPie(OB, new Rectangle(Opt, Osz), 0.0f, 360.0f);
            g.DrawPie(new Pen(Color.FromArgb(128, 25, 25, 25), 2), new Rectangle(Opt, Osz), 0.0f, 360.0f);
            Brush IB = new SolidBrush(slvSOC_Color(soc));
            g.FillPie(IB, new Rectangle(Ipt, Isz), 0.0f, 3.6f * soc);
            g.DrawPie(new Pen(Color.FromArgb(128, 25, 25, 25), 1), new Rectangle(Ipt, Isz), 0.0f, 3.6f * soc);
        }

        public void slvDrawBarBattery(Graphics g, Point pt, int outerR, float soc)
        {
            Brush IB = new SolidBrush(slvSOC_Color(soc));
            Brush OB = new SolidBrush(Color.FromArgb(255, 64, 64, 64));
            FillMode fill = FillMode.Winding;
            //int innerR = outerR - 4;
            Size Osz = new Size((int)((5*outerR/8)), outerR);

            Point Opt = new Point(pt.X - Osz.Width / 2, pt.Y - Osz.Height);
            Point[] points = { new Point(pt.X - Osz.Width / 2, pt.Y), 
                               new Point(pt.X + Osz.Width / 2, pt.Y), 
                               new Point(pt.X + Osz.Width / 2, pt.Y - Osz.Height), 
                               new Point(pt.X + Osz.Width / 4, pt.Y - Osz.Height), 
                               new Point(pt.X + Osz.Width / 4, pt.Y - Osz.Height - 10), 
                               new Point(pt.X - Osz.Width / 4, pt.Y - Osz.Height - 10), 
                               new Point(pt.X - Osz.Width / 4, pt.Y - Osz.Height), 
                               new Point(pt.X - Osz.Width / 2, pt.Y - Osz.Height) };
            float tension = 0.15f;
            g.FillClosedCurve(IB, points, fill, tension);
            g.DrawClosedCurve(new Pen(Color.Black), points, tension, fill);
            int reduceBy = 4;
            Opt.X += reduceBy;  Opt.Y += reduceBy;  Osz.Width -= 2*reduceBy;    Osz.Height -= 2*reduceBy;
            g.FillRectangle(OB, new Rectangle(Opt, Osz));
            //g.DrawRectangle(new Pen(Color.FromArgb(128, 25, 25, 25), 2), new Rectangle(Opt, Osz));
            Size Isz = new Size(Osz.Width, (int)(Osz.Height * (soc / 100.0f)));
            Point Ipt = new Point(pt.X - Isz.Width / 2, pt.Y - Isz.Height);
            g.FillRectangle(IB, new Rectangle(Ipt, Isz));
            //g.DrawRectangle(new Pen(Color.FromArgb(128, 25, 25, 25), 1), new Rectangle(Ipt, Isz));
            Point fpt = new Point(pt.X - reduceBy/2 - Osz.Width / 2, pt.Y - 14 - Osz.Height / 2);
            g.DrawString(soc.ToString(" 0.0"), new Font("Helvetica", 14, FontStyle.Bold), Brushes.White, fpt);
        }

        public override void OnRender(Graphics g)
        {
            double width = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Width, 0)) * 1000.0);
            double height = (MainMap.Manager.GetDistance(MainMap.FromLocalToLatLng(0, 0), MainMap.FromLocalToLatLng(MainMap.Height, 0)) * 1000.0);
            double m2pixelwidth = MainMap.Width / width;
            double m2pixelheight = MainMap.Height / height;
            int trafBubbleW = (int)(20 * m2pixelwidth);
            int trafBubbleH = (int)(20 * m2pixelheight);
            int warningBubbleW = (int)(38.1 * m2pixelwidth);
            int warningBubbleH = (int)(38.1 * m2pixelheight);
            int distBubbleW = (int)(42 * m2pixelwidth);
            int distBubbleH = (int)(42 * m2pixelheight);
            int relspeedBubbleW = (int)(50 * m2pixelwidth);
            int relspeedBubbleH = (int)(50 * m2pixelheight);
            float length = MainV2.cs.wp_dist * (float)m2pixelwidth;
            //float desired_lead_dist = (MainV2.cs.wp_radius - global::ArdupilotMega.Properties.Resources.planeicon.Height / 2) * (float)m2pixelwidth;
            float desired_lead_dist = 100 * (float)m2pixelwidth;

            Matrix temp = g.Transform;
            Point mypt1;
            Point mypt2;
            GPoint mygpt;
            mygpt = MainMap.FromLatLngToLocal(new PointLatLng(FlightPlanner.cmds[0].lat, FlightPlanner.cmds[0].lng));
            mypt1 = new Point(mygpt.X, mygpt.Y);
            mygpt = MainMap.FromLatLngToLocal(new PointLatLng(FlightPlanner.cmds[1].lat, FlightPlanner.cmds[1].lng));
            mypt2 = new Point(mygpt.X, mygpt.Y);
            Pen myPen = new Pen(Color.Gray, 5);
            myPen.DashStyle = DashStyle.Dash;
            g.DrawLine(myPen ,mypt1,mypt2);

            mygpt = MainMap.FromLatLngToLocal(MainV2.abortLoc);
            mypt1 = new Point(mygpt.X-10, mygpt.Y-10);
            //g.FillEllipse(Brushes.HotPink, new Rectangle(mypt1, new Size(20, 20)));

            mygpt = MainMap.FromLatLngToLocal(MainV2.maxLoc);
            mypt1 = new Point(mygpt.X-10, mygpt.Y-10);
            //g.FillEllipse(Brushes.OrangeRed, new Rectangle(mypt1, new Size(20, 20)));

            g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

            /*
                        g.RotateTransform(-MainMap.Bearing);

                        // anti NaN
                        try
                        {
                            if (ownship)
                                g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
                        }
                        catch { }
                        if (ownship)
                        {
                            g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
                            g.DrawLine(new Pen(Color.Green, 4), 0.0f, 0.0f, (float)Math.Cos((nav_bearing - 90) * deg2rad) * length, (float)Math.Sin((nav_bearing - 90) * deg2rad) * length);
                            g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
                        }
            
                        try
                        {
                            if (ownship)
                            {
                                float alpha = ((desired_lead_dist) / MainV2.cs.radius) * rad2deg;
                                float myR = Math.Abs(MainV2.cs.radius);
                                float myH;
                                float myXoff;
                                float myYoff;

                                //myH = (int)heading - 90;
                                myXoff = (global::ArdupilotMega.Properties.Resources.planeicon.Height / 2) * (float)Math.Cos((heading - 90) * deg2rad);
                                myYoff = (global::ArdupilotMega.Properties.Resources.planeicon.Height / 2) * (float)Math.Sin((heading - 90) * deg2rad);

                                if (MainV2.cs.radius < 0)
                                {
                                    myH = heading - 180;
                                    myXoff += (myR) * (float)Math.Cos((myH) * deg2rad);
                                    myYoff += (myR) * (float)Math.Sin((myH) * deg2rad);
                                }
                                else
                                {
                                    myH = heading;
                                    myXoff += (myR) * (float)Math.Cos((myH) * deg2rad);
                                    myYoff += (myR) * (float)Math.Sin((myH) * deg2rad);
                                }
                                g.DrawArc(new Pen(Color.HotPink, 2), myXoff - myR, myYoff - myR, 2 * myR, 2 * myR, myH - 180, alpha);
                    
                                // Draw Bands around ownship
                                g.DrawArc(new Pen(Color.LightGreen, 1), -warningBubbleW, -warningBubbleH, 2*warningBubbleW, 2*warningBubbleH, 0 - 180, 360);

                                //g.DrawArc(new Pen(Color.LightGreen, 1), -3 * Size.Width / 4, -3 * Size.Width / 4, 2 * 3 * Size.Width / 4, 2 * 3 * Size.Width / 4, 0 - 180, 360);
                                //g.DrawArc(new Pen(Color.Green, 1), -Size.Width, -Size.Width, 2 * Size.Width, 2 * Size.Width, 0 - 180, 360);
                                PointLatLng currentloc = new PointLatLng(MainV2.cs.lat, MainV2.cs.lng);
                                for (int i = 0; i < MainV2.trafficTable.Count; i++)
                                {
                                    if (MainV2.trafficTable[i].ownship == 0)
                                    {
                                        PointLatLng otherloc = new PointLatLng(MainV2.trafficTable[i].Lat, MainV2.trafficTable[i].Lng);
                                        float dlong = (float)(otherloc.Lng - currentloc.Lng);
                                        float dlat = (float)(otherloc.Lat - currentloc.Lat); // *scaleLongUp;
                                        int ahead = (int)(Math.Atan2(-dlat, dlong) * 57.2957795);
                                        if (ahead < 0) ahead += 360;
                                        float adist = (float)(1e7 * Math.Sqrt(Math.Pow(dlat, 2) + Math.Pow(dlong, 2)) * .01113195);
                                        float dRat = adist / 1500;
                                        if (dRat > 1) dRat = 1;
                                        float aRat = 10 * (MainV2.trafficTable[i].pdist - dRat);
                                        aRat = aRat > 1 ? 1 : aRat < -1 ? -1 : aRat;
                                        MainV2.trafficTable[i].pdist = dRat;

                                        //Console.WriteLine("Distance = " + adist.ToString() + "  Ratio = " + dRat.ToString());

                                        int rColor = (int)(255.0 * Math.Cos(dRat * 1.5708));
                                        int gColor = (int)(255.0 * Math.Sin(dRat * 1.5708));
                                        int bColor = 0;
                                        g.DrawArc(new Pen(Color.FromArgb(255, rColor, gColor, bColor), 16), -warningBubbleW, -warningBubbleH, 2 * warningBubbleW, 2 * warningBubbleH, ahead-10, 20);

                                        rColor = (int)(255 * (aRat > 0 ? aRat : 0));
                                        gColor = (int)((255 * (1 - Math.Abs(aRat))));
                                        bColor = (int)(255 * (aRat < 0 ? -aRat : 0));
                                        g.DrawArc(new Pen(Color.FromArgb(255, rColor, gColor, bColor), 8), -warningBubbleW, -warningBubbleH, 2 * warningBubbleW, 2 * warningBubbleH, ahead-5, 10);
                                    }
                                }
                            }
                        }
                        catch { }
                        */
// anti NaN
            try
            {
            g.RotateTransform(heading);
            } catch{}
            if (ownship)
            {
                //g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.planeicon, global::ArdupilotMega.Properties.Resources.planeicon.Width / -2, global::ArdupilotMega.Properties.Resources.planeicon.Height / -2);
                g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.planeicon, -global::ArdupilotMega.Properties.Resources.planeicon.Width / 2, -3*global::ArdupilotMega.Properties.Resources.planeicon.Height / 8);
            }
            else
            {
                //g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.otherplane, global::ArdupilotMega.Properties.Resources.otherplane.Width / -2, global::ArdupilotMega.Properties.Resources.otherplane.Height / -2);
                float alt_diff = (alt - MainV2.cs.alt)/100;
                alt_diff = (alt_diff>1) ? 1 : (alt_diff<-1) ? -1 : alt_diff;
                int rColor = (int)(255.0 * (1-Math.Abs(alt_diff)));
                int gColor = alt_diff > 0 ? 0 : (int)(255.0 * Math.Abs(alt_diff));
                int bColor = alt_diff < 0 ? 0 : (int)(255.0 * Math.Abs(alt_diff));
                g.DrawArc(new Pen(Color.FromArgb(255, rColor, gColor, bColor), 8), -trafBubbleW,   -trafBubbleH,   2*trafBubbleW,   2*trafBubbleH,   0, 360);
                g.DrawArc(new Pen(Color.LightGreen, 1), -warningBubbleW, -warningBubbleH, 2 * warningBubbleW, 2 * warningBubbleH, 0, 360);
            }
            g.Transform = temp;


            //------------------------
            // Display Trip Statistics
            //

            try
            {
                float tension = 0f;
                FillMode fill = FillMode.Winding;
                foreach (GMapPolygon m in FlightPlanner.Hazards.Polygons)
                {
                    Brush hzBrush = new HatchBrush(HatchStyle.Percent60, Color.FromArgb(64, Color.Red), Color.FromArgb(64, Color.White));
                    List<Point> pl = new List<Point>();
                    foreach (GPoint p in m.LocalPoints)
                    {
                        pl.Add(new Point(p.X,p.Y));
                    }
                    g.FillClosedCurve(hzBrush, pl.ToArray(), fill, tension);

                }

                //float tripOdometer = MainV2.tripOdometer / 1000;
                float tripTimer = MainV2.tripTimer / 60;
                int xc = 10 + MainV2.fpw;
                int yc = MainMap.Height - 610;
                g.FillRectangle(new SolidBrush(Color.FromArgb(128, 0, 0, 0)), new Rectangle(xc, yc, 190, 600));
                String mess =
                    "Mission Distance: " + (MainV2.missionDist / 1000).ToString("0.0") + " (Km)" + Environment.NewLine +
                    "Travel  Distance: " + (MainV2.tripOdometer / 1000).ToString("0.0") + " (Km)" + Environment.NewLine +
                    "Abort   Distance: " + (MainV2.abortDist / 1000).ToString("0.0") + " (Km)" + Environment.NewLine + Environment.NewLine +

                    "Remaining   Dist: " + (MainV2.remDist / 1000).ToString("0.0") + " (Km)" + Environment.NewLine +
                    "EOD         Dist: " + (MainV2.eodDist / 1000).ToString("0.0") + " (Km)" + Environment.NewLine +
                    "Max   Dist  Left: " + (MainV2.maxDistLeft / 1000).ToString("0.0") + " (Km)" + Environment.NewLine + Environment.NewLine +

                    "Travel      Time: " + ((int)tripTimer).ToString("00") + ":" + ((int)((tripTimer - (int)tripTimer) * 60)).ToString("00") + " (m:s)" + Environment.NewLine +
                    "Abort       Time: " + ((int)MainV2.abortTime).ToString("00") + ":" + ((int)((MainV2.abortTime - (int)MainV2.abortTime) * 60)).ToString("00") + " (m:s)" + Environment.NewLine +
                    "Arrival     Time: " + ((int)MainV2.etaTime).ToString("00") + ":" + ((int)((MainV2.etaTime - (int)MainV2.etaTime) * 60)).ToString("00") + " (m:s)" + Environment.NewLine +
                    "EOD         Time: " + ((int)MainV2.eodTime).ToString("00") + ":" + ((int)((MainV2.eodTime - (int)MainV2.eodTime) * 60)).ToString("00") + " (m:s)" + Environment.NewLine +
                    "Abort  Home Time: " + ((int)MainV2.abortHomeTime).ToString("00") + ":" + ((int)((MainV2.abortHomeTime - (int)MainV2.abortHomeTime) * 60)).ToString("00") + " (m:s)" + Environment.NewLine +
                    "Max   Time  Left: " + ((int)MainV2.maxTimeLeft).ToString("00") + ":" + ((int)((MainV2.maxTimeLeft - (int)MainV2.maxTimeLeft) * 60)).ToString("00") + " (m:s)" + Environment.NewLine +
                    "Return Home Time: " + ((int)MainV2.retHomeTime).ToString("00") + ":" + ((int)((MainV2.retHomeTime - (int)MainV2.retHomeTime) * 60)).ToString("00") + " (m:s)" + Environment.NewLine + Environment.NewLine +

                    "Average    Speed: " + MainV2.avgSpeed[1].ToString("0.0") + " (m/s)" + Environment.NewLine +
                    " ";
                Brush fontBrush = Brushes.White;
                g.DrawString(mess, new Font("Courier", 8, FontStyle.Regular), fontBrush, new Point(xc + 10, yc));
                g.FillRectangle(new SolidBrush(Color.FromArgb(128, 0, 0, 0)), new Rectangle(0, 0, MainMap.Width, 50));
                if (MainV2.soc1 < 30)
                    g.DrawString("EMERGENCY:: Land Immediately -- inminent battery failure",
                                 new Font("Courier", 18, FontStyle.Bold), Brushes.Red, new Point(200, 5));
                else if (MainV2.retHomeTime > MainV2.eodTime + 0.5)
                    g.DrawString("ALERT:: Find Alternate Landing Site -- not enough battery charge to return HOME",
                                 new Font("Courier", 18, FontStyle.Bold), Brushes.Orange, new Point(200, 5));
                else if (ArdupilotMega.GCSViews.FlightPlanner.myStatus == ArdupilotMega.GCSViews.FlightPlanner.missionStatus.InsufficientCharge)
                {
                    g.DrawString("WARNING:: Insufficient charge to complete mission. Please replan",
                                 new Font("Courier", 18, FontStyle.Bold), Brushes.Yellow, new Point(200, 5));
                }
                else if (ArdupilotMega.GCSViews.FlightPlanner.myStatus == ArdupilotMega.GCSViews.FlightPlanner.missionStatus.Warning)
                {
                    g.DrawString("WARNING:: Abort Mission and Return HOME -- battery has just enough charge to return HOME",
                                 new Font("Courier", 18, FontStyle.Bold), Brushes.Yellow, new Point(200, 5));
                }
                else
                    g.DrawString("NOTE:: Status normal -- battery system has enough charge to complete mission",
                                 new Font("Courier", 18, FontStyle.Bold), Brushes.White, new Point(200, 5));

                //------------------------
                // Draw Battery Data Here
                //
                float sf = 1.1f;
                int outerR = 110;

                /*
                Point pt = new Point(300, (int)(m2pixelheight * height) - 2 * outerR);
                slvDrawPieBattery(g, pt, outerR, MainV2.soc1);
                pt = new Point(300 + (int)(sf * outerR), (int)(m2pixelheight * height) - 2 * outerR);
                slvDrawPieBattery(g, pt, outerR, MainV2.soc2);
                pt = new Point(300 + (int)(sf * outerR), (int)(m2pixelheight * height) - 2 * outerR + (int)(sf * outerR));
                slvDrawPieBattery(g, pt, outerR, MainV2.soc3);
                pt = new Point(300, (int)(m2pixelheight * height) - 2 * outerR + (int)(sf * outerR));
                slvDrawPieBattery(g, pt, outerR, MainV2.soc4);
                */

                float sfb = 0.85f;
                sf = 1.5f;
                Point pt = new Point(xc + 50, (int)(m2pixelheight * height) - 2 * outerR);
                slvDrawBarBattery(g, pt, outerR, MainV2.soc1);
                pt = new Point(xc + 50 + (int)(sfb * outerR), (int)(m2pixelheight * height) - 2 * outerR);
                slvDrawBarBattery(g, pt, outerR, MainV2.soc2);
                pt = new Point(xc + 50 + (int)(sfb * outerR), (int)(m2pixelheight * height) - 2 * outerR + (int)(sf * outerR));
                slvDrawBarBattery(g, pt, outerR, MainV2.soc3);
                pt = new Point(xc + 50, (int)(m2pixelheight * height) - 2 * outerR + (int)(sf * outerR));
                slvDrawBarBattery(g, pt, outerR, MainV2.soc4);
            }
            catch (Exception ex) { Console.WriteLine("Battery Statistics Overlay Error: " + ex.ToString()); return; }
        }
    }

    public class GMapMarkerQuad : GMapMarker
    {
        const float rad2deg = (float)(180 / Math.PI);
        const float deg2rad = (float)(1.0 / rad2deg);

        static readonly System.Drawing.Size SizeSt = new System.Drawing.Size(global::ArdupilotMega.Properties.Resources.quadicon.Width, global::ArdupilotMega.Properties.Resources.quadicon.Height);
        float heading = 0;
        float cog = -1;
        float target = -1;

        public GMapMarkerQuad(PointLatLng p, float heading, float cog, float target)
            : base(p)
        {
            this.heading = heading;
            this.cog = cog;
            this.target = target;
            Size = SizeSt;
        }

        public override void OnRender(Graphics g)
        {
            Matrix temp = g.Transform;
            g.TranslateTransform(LocalPosition.X, LocalPosition.Y);

            int length = 500;
// anti NaN
            try
            {
                g.DrawLine(new Pen(Color.Red, 2), 0.0f, 0.0f, (float)Math.Cos((heading - 90) * deg2rad) * length, (float)Math.Sin((heading - 90) * deg2rad) * length);
            }
            catch { }
            //g.DrawLine(new Pen(Color.Green, 2), 0.0f, 0.0f, (float)Math.Cos((nav_bearing - 90) * deg2rad) * length, (float)Math.Sin((nav_bearing - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Black, 2), 0.0f, 0.0f, (float)Math.Cos((cog - 90) * deg2rad) * length, (float)Math.Sin((cog - 90) * deg2rad) * length);
            g.DrawLine(new Pen(Color.Orange, 2), 0.0f, 0.0f, (float)Math.Cos((target - 90) * deg2rad) * length, (float)Math.Sin((target - 90) * deg2rad) * length);
// anti NaN
            try
            {
                g.RotateTransform(heading);
            }
            catch { }
            g.DrawImageUnscaled(global::ArdupilotMega.Properties.Resources.quadicon, global::ArdupilotMega.Properties.Resources.quadicon.Width / -2 + 2, global::ArdupilotMega.Properties.Resources.quadicon.Height / -2);

            g.Transform = temp;
        }
    }

    public class PointLatLngAlt
    {
        public double Lat = 0;
        public double Lng = 0;
        public double Alt = 0;
        public string Tag = "";
        public Color color = Color.White;

        public PointLatLngAlt(double lat, double lng, double alt, string tag)
        {
            this.Lat = lat;
            this.Lng = lng;
            this.Alt = alt;
            this.Tag = tag;
        }

        public PointLatLngAlt()
        {

        }

        public PointLatLngAlt(GMap.NET.PointLatLng pll)
        {
            this.Lat = pll.Lat;
            this.Lng = pll.Lng;
        }

        public PointLatLngAlt(Locationwp locwp)
        {
            this.Lat = locwp.lat;
            this.Lng = locwp.lng;
            this.Alt = locwp.alt;
        }

        public PointLatLngAlt(PointLatLngAlt plla)
        {
            this.Lat = plla.Lat;
            this.Lng = plla.Lng;
            this.Alt = plla.Alt;
            this.color = plla.color;
            this.Tag = plla.Tag;
        }

        public PointLatLng Point()
        {
            return new PointLatLng(Lat, Lng);
        }

        public override bool Equals(Object pllaobj)
         {
             PointLatLngAlt plla = (PointLatLngAlt)pllaobj;

             if (plla == null)
                 return false;

             if (this.Lat == plla.Lat &&
             this.Lng == plla.Lng &&
             this.Alt == plla.Alt &&
             this.color == plla.color &&
             this.Tag == plla.Tag)
             {
                 return true;
             }
             return false;
         }

        public override int GetHashCode()
        {
            return (int)((Lat + Lng + Alt) * 100);
        }

        /// <summary>
        /// Calc Distance in M
        /// </summary>
        /// <param name="p2"></param>
        /// <returns>Distance in M</returns>
        public double GetDistance(PointLatLngAlt p2)
        {
            double d = Lat * 0.017453292519943295;
            double num2 = Lng * 0.017453292519943295;
            double num3 = p2.Lat * 0.017453292519943295;
            double num4 = p2.Lng * 0.017453292519943295;
            double num5 = num4 - num2;
            double num6 = num3 - d;
            double num7 = Math.Pow(Math.Sin(num6 / 2.0), 2.0) + ((Math.Cos(d) * Math.Cos(num3)) * Math.Pow(Math.Sin(num5 / 2.0), 2.0));
            double num8 = 2.0 * Math.Atan2(Math.Sqrt(num7), Math.Sqrt(1.0 - num7));
            return (6378.137 * num8) * 1000; // M
        }
    }

    class NoCheckCertificatePolicy : ICertificatePolicy
    {
        public bool CheckValidationResult(ServicePoint srvPoint, X509Certificate certificate, WebRequest request, int certificateProblem)
        {
            return true;
        }
    } 


    public class Common
    {
        private static readonly ILog log = LogManager.GetLogger(MethodBase.GetCurrentMethod().DeclaringType);
        public enum distances
        {
            Meters,
            Feet
        }

        public enum speeds
        {
            ms,
            fps,
            kph,
            mph,
            knots
        }

        public enum apmmodes
        {
            MANUAL = 0,
            CIRCLE = 1,
            STABILIZE = 2,
            FLY_BY_WIRE_A = 5,
            FLY_BY_WIRE_B = 6,
            AUTO = 10,
            RTL = 11,
            LOITER = 12,
            GUIDED = 15
        }

        public enum ac2modes
        {
            STABILIZE = 0,			// hold level position
            ACRO = 1,			// rate control
            ALT_HOLD = 2,		// AUTO control
            AUTO = 3,			// AUTO control
            GUIDED = 4,		// AUTO control
            LOITER = 5,		// Hold a single location
            RTL = 6,				// AUTO control
            CIRCLE = 7,
            POSITION = 8,
            LAND = 9,				// AUTO control
            OF_LOITER = 10
        }

        public enum ac2ch7modes
        {
            CH7_DO_NOTHING = 0,
            CH7_SET_HOVER = 1,
            CH7_FLIP = 2,
            CH7_SIMPLE_MODE = 3,
            CH7_RTL = 4,
            CH7_AUTO_TRIM = 5,
            CH7_ADC_FILTER = 6,
            CH7_SAVE_WP = 7
        }

        public enum ac2ch6modes
        {
            // CH_6 Tuning
            // -----------
            CH6_NONE = 0,
            // Attitude
            CH6_STABILIZE_KP = 1,
            CH6_STABILIZE_KI = 2,
            CH6_YAW_KP = 3,
            // Rate
            CH6_RATE_KP = 4,
            CH6_RATE_KI = 5,
            CH6_RATE_KD = 21,
            CH6_YAW_RATE_KP = 6,
            // Altitude rate controller
            CH6_THROTTLE_KP = 7,
            // Extras
            CH6_TOP_BOTTOM_RATIO = 8,
            CH6_RELAY = 9,
            CH6_TRAVERSE_SPEED = 10,

            CH6_NAV_P = 11,
            CH6_LOITER_P = 12,
            CH6_HELI_EXTERNAL_GYRO = 13,

            // altitude controller
            CH6_THR_HOLD_KP = 14,
            CH6_Z_GAIN = 15,
            CH6_DAMP = 16,

            // optical flow controller
            CH6_OPTFLOW_KP = 17,
            CH6_OPTFLOW_KI = 18,
            CH6_OPTFLOW_KD = 19,

            CH6_NAV_I = 20,

            CH6_LOITER_RATE_P = 22
        }


        public static void linearRegression()
        {
            double[] values = { 4.8, 4.8, 4.5, 3.9, 4.4, 3.6, 3.6, 2.9, 3.5, 3.0, 2.5, 2.2, 2.6, 2.1, 2.2 };
            
            double xAvg = 0;
            double yAvg = 0;

            for (int x = 0; x < values.Length; x++)
            {
                xAvg += x;
                yAvg += values[x];
            }

            xAvg = xAvg / values.Length;
            yAvg = yAvg / values.Length;


            double v1 = 0;
            double v2 = 0;

            for (int x = 0; x < values.Length; x++)
            {
                v1 += (x - xAvg) * (values[x] - yAvg);
                v2 += Math.Pow(x - xAvg, 2);
            }

            double a = v1 / v2;
            double b = yAvg - a * xAvg;

            log.Debug("y = ax + b");
            log.DebugFormat("a = {0}, the slope of the trend line.", Math.Round(a, 2));
            log.DebugFormat("b = {0}, the intercept of the trend line.", Math.Round(b, 2));

            //Console.ReadLine();
        }
       
		#if MAVLINK10
		
        public static bool translateMode(string modein, ref MAVLink.__mavlink_set_mode_t mode)
        {
            //MAVLink.__mavlink_set_mode_t mode = new MAVLink.__mavlink_set_mode_t();
            mode.target_system = MainV2.comPort.sysid;

            try
            {
                if (Common.getModes() == typeof(Common.apmmodes))
                {
                    switch ((int)Enum.Parse(Common.getModes(), modein))
                    {
                        case (int)Common.apmmodes.MANUAL:
                        case (int)Common.apmmodes.CIRCLE:
                        case (int)Common.apmmodes.STABILIZE:
                        case (int)Common.apmmodes.AUTO:
                        case (int)Common.apmmodes.RTL:
                        case (int)Common.apmmodes.LOITER:
                        case (int)Common.apmmodes.FLY_BY_WIRE_A:
                        case (int)Common.apmmodes.FLY_BY_WIRE_B:
                            mode.base_mode = (byte)MAVLink.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                            mode.custom_mode = (uint)(int)Enum.Parse(Common.getModes(), modein);
                            break;
                        default:
                            MessageBox.Show("No Mode Changed " + (int)Enum.Parse(Common.getModes(), modein));
                            return false;
                    }
                }
                else if (Common.getModes() == typeof(Common.ac2modes))
                {
                    switch ((int)Enum.Parse(Common.getModes(), modein))
                    {
                        case (int)Common.ac2modes.STABILIZE:
                        case (int)Common.ac2modes.AUTO:
                        case (int)Common.ac2modes.RTL:
                        case (int)Common.ac2modes.LOITER:
                        case (int)Common.ac2modes.ACRO:
                        case (int)Common.ac2modes.ALT_HOLD:
                        case (int)Common.ac2modes.CIRCLE:
                        case (int)Common.ac2modes.POSITION:
                            mode.base_mode = (byte)MAVLink.MAV_MODE_FLAG.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
                            mode.custom_mode = (uint)(int)Enum.Parse(Common.getModes(), modein);
                            break;
                        default:
                            MessageBox.Show("No Mode Changed " + (int)Enum.Parse(Common.getModes(), modein));
                            return false;
                    }
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to find Mode"); return false; }

            return true;
        }
		
		#else
        public static bool translateMode(string modein, ref  MAVLink.__mavlink_set_nav_mode_t navmode, ref MAVLink.__mavlink_set_mode_t mode)
        {

            //MAVLink.__mavlink_set_nav_mode_t navmode = new MAVLink.__mavlink_set_nav_mode_t();
            navmode.target = MainV2.comPort.sysid;
            navmode.nav_mode = 255;

            //MAVLink.__mavlink_set_mode_t mode = new MAVLink.__mavlink_set_mode_t();
            mode.target = MainV2.comPort.sysid;

            try
            {
                if (Common.getModes() == typeof(Common.apmmodes))
                {
                    switch ((int)Enum.Parse(Common.getModes(), modein))
                    {
                        case (int)Common.apmmodes.MANUAL:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_MANUAL;
                            break;
                        case (int)Common.apmmodes.GUIDED:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_GUIDED;
                            break;
                        case (int)Common.apmmodes.STABILIZE:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST1;
                            break;
                        // AUTO MODES
                        case (int)Common.apmmodes.AUTO:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_WAYPOINT;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.apmmodes.RTL:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_RETURNING;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.apmmodes.LOITER:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_LOITER;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.apmmodes.CIRCLE:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_VECTOR;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST3;
                            break;
                        // FBW
                        case (int)Common.apmmodes.FLY_BY_WIRE_A:
                            navmode.nav_mode = (byte)1;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST2;
                            break;
                        case (int)Common.apmmodes.FLY_BY_WIRE_B:
                            navmode.nav_mode = (byte)2;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_TEST2;
                            break;
                        default:
                            MessageBox.Show("No Mode Changed " + (int)Enum.Parse(Common.getModes(), modein));
                            return false;
                    }
                }
                else if (Common.getModes() == typeof(Common.ac2modes))
                {
                    switch ((int)Enum.Parse(Common.getModes(), modein))
                    {
                        case (int)Common.ac2modes.GUIDED:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_GUIDED;
                            break;
                        case (int)Common.ac2modes.STABILIZE:
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_MANUAL;
                            break;
                        // AUTO MODES
                        case (int)Common.ac2modes.AUTO:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_WAYPOINT;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.ac2modes.RTL:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_RETURNING;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        case (int)Common.ac2modes.LOITER:
                            navmode.nav_mode = (byte)MAVLink.MAV_NAV.MAV_NAV_LOITER;
                            mode.mode = (byte)MAVLink.MAV_MODE.MAV_MODE_AUTO;
                            break;
                        default:
                            MessageBox.Show("No Mode Changed " + (int)Enum.Parse(Common.getModes(), modein));
                            return false;
                    }
                }
            }
            catch { System.Windows.Forms.MessageBox.Show("Failed to find Mode"); return false; }

            return true;
        }		
		#endif


        
        public static bool getFilefromNet(string url,string saveto) {
            try
            {
                // this is for mono to a ssl server
                ServicePointManager.CertificatePolicy = new NoCheckCertificatePolicy(); 

                // Create a request using a URL that can receive a post. 
                WebRequest request = WebRequest.Create(url);
                request.Timeout = 5000;
                // Set the Method property of the request to POST.
                request.Method = "GET";
                // Get the response.
                WebResponse response = request.GetResponse();
                // Display the status.
                log.Info(((HttpWebResponse)response).StatusDescription);
                if (((HttpWebResponse)response).StatusCode != HttpStatusCode.OK)
                    return false;
                // Get the stream containing content returned by the server.
                Stream dataStream = response.GetResponseStream();

                long bytes = response.ContentLength;
                long contlen = bytes;

                byte[] buf1 = new byte[1024];

                FileStream fs = new FileStream(saveto + ".new", FileMode.Create);

                DateTime dt = DateTime.Now;

                while (dataStream.CanRead && bytes > 0)
                {
                    Application.DoEvents();
                    log.Info(saveto + " " + bytes);
                    int len = dataStream.Read(buf1, 0, buf1.Length);
                    bytes -= len;
                    fs.Write(buf1, 0, len);
                }

                fs.Close();
                dataStream.Close();
                response.Close();

                File.Delete(saveto);
                File.Move(saveto + ".new", saveto);

                return true;
            }
            catch (Exception ex) { log.Info("getFilefromNet(): " + ex.ToString()); return false; }
        }

        public static Type getModes()
        {
            if (MainV2.cs.firmware == MainV2.Firmwares.ArduPlane)
            {
                return typeof(apmmodes);
            }
            else if (MainV2.cs.firmware == MainV2.Firmwares.ArduCopter2)
            {
                return typeof(ac2modes);
            }

            return null;
        }

        public static Form LoadingBox(string title, string promptText)
        {
            Form form = new Form();
            System.Windows.Forms.Label label = new System.Windows.Forms.Label();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            form.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            form.Text = title;
            label.Text = promptText;

            label.SetBounds(9, 50, 372, 13);

            label.AutoSize = true;

            form.ClientSize = new Size(396, 107);
            form.Controls.AddRange(new Control[] { label });
            form.ClientSize = new Size(Math.Max(300, label.Right + 10), form.ClientSize.Height);
            form.FormBorderStyle = FormBorderStyle.FixedDialog;
            form.StartPosition = FormStartPosition.CenterScreen;
            form.MinimizeBox = false;
            form.MaximizeBox = false;

            ThemeManager.ApplyThemeTo(form);

            form.Show();
            form.Refresh();
            label.Refresh();
            Application.DoEvents();
            return form;
        }

        public static DialogResult MessageShowAgain(string title, string promptText)
        {
            Form form = new Form();
            System.Windows.Forms.Label label = new System.Windows.Forms.Label();
            CheckBox chk = new CheckBox();
            MyButton buttonOk = new MyButton();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(MainV2));
            form.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));

            form.Text = title;
            label.Text = promptText;

            chk.Tag = ("SHOWAGAIN_" + title.Replace(" ","_"));
            chk.AutoSize = true;
            chk.Text = "Show me again?";
            chk.Checked = true;
            chk.Location = new Point(9,80);

            if (MainV2.config[(string)chk.Tag] != null && (string)MainV2.config[(string)chk.Tag] == "False") // skip it
            {
                form.Dispose();
                chk.Dispose();
                buttonOk.Dispose();
                label.Dispose();
                return DialogResult.OK;
            }

            chk.CheckStateChanged += new EventHandler(chk_CheckStateChanged);

            buttonOk.Text = "OK";
            buttonOk.DialogResult = DialogResult.OK;
            buttonOk.Location = new Point(form.Right - 100 ,80);

            label.SetBounds(9, 40, 372, 13);

            label.AutoSize = true;

            form.ClientSize = new Size(396, 107);
            form.Controls.AddRange(new Control[] { label, chk, buttonOk });
            form.ClientSize = new Size(Math.Max(300, label.Right + 10), form.ClientSize.Height);
            form.FormBorderStyle = FormBorderStyle.FixedDialog;
            form.StartPosition = FormStartPosition.CenterScreen;
            form.MinimizeBox = false;
            form.MaximizeBox = false;

            ThemeManager.ApplyThemeTo(form);

            DialogResult dialogResult =form.ShowDialog();

            form.Dispose();

            return dialogResult;
        }

        static void chk_CheckStateChanged(object sender, EventArgs e)
        {
            MainV2.config[(string)((CheckBox)(sender)).Tag] = ((CheckBox)(sender)).Checked.ToString();
        }

        //from http://www.csharp-examples.net/inputbox/
        public static DialogResult InputBox(string title, string promptText, ref string value)
        {
            Form form = new Form();
            System.Windows.Forms.Label label = new System.Windows.Forms.Label();
            TextBox textBox = new TextBox();
            MyButton buttonOk = new MyButton();
            MyButton buttonCancel = new MyButton();

            form.TopMost = true;

            form.Text = title;
            label.Text = promptText;
            textBox.Text = value;

            buttonOk.Text = "OK";
            buttonCancel.Text = "Cancel";
            buttonOk.DialogResult = DialogResult.OK;
            buttonCancel.DialogResult = DialogResult.Cancel;

            label.SetBounds(9, 20, 372, 13);
            textBox.SetBounds(12, 36, 372, 20);
            buttonOk.SetBounds(228, 72, 75, 23);
            buttonCancel.SetBounds(309, 72, 75, 23);

            label.AutoSize = true;
            textBox.Anchor = textBox.Anchor | AnchorStyles.Right;
            buttonOk.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;
            buttonCancel.Anchor = AnchorStyles.Bottom | AnchorStyles.Right;

            form.ClientSize = new Size(396, 107);
            form.Controls.AddRange(new Control[] { label, textBox, buttonOk, buttonCancel });
            form.ClientSize = new Size(Math.Max(300, label.Right + 10), form.ClientSize.Height);
            form.FormBorderStyle = FormBorderStyle.FixedDialog;
            form.StartPosition = FormStartPosition.CenterScreen;
            form.MinimizeBox = false;
            form.MaximizeBox = false;
            form.AcceptButton = buttonOk;
            form.CancelButton = buttonCancel;

            ThemeManager.ApplyThemeTo(form);

            DialogResult dialogResult = DialogResult.Cancel;

            dialogResult = form.ShowDialog();

            if (dialogResult == DialogResult.OK)
            {
                value = textBox.Text;
            }

            form.Dispose();
            
            return dialogResult;
        }

        public static string speechConversion(string input)
        {
            if (MainV2.cs.wpno == 0)
            {
                input = input.Replace("{wpn}", "Home");
            }
            else
            {
                input = input.Replace("{wpn}", MainV2.cs.wpno.ToString());
            }

            input = input.Replace("{asp}", MainV2.cs.airspeed.ToString("0"));

            input = input.Replace("{alt}", MainV2.cs.alt.ToString("0"));

            input = input.Replace("{wpa}", MainV2.cs.targetalt.ToString("0"));

            input = input.Replace("{gsp}", MainV2.cs.groundspeed.ToString("0"));

            input = input.Replace("{mode}", MainV2.cs.mode.ToString());

            input = input.Replace("{batv}", MainV2.cs.battery_voltage.ToString("0.00"));

            return input;
        }
    }

    public class VerticalProgressBar2 : HorizontalProgressBar2
    {
        protected override void OnPaint(PaintEventArgs e)
        {
            e.Graphics.TranslateTransform(0, e.Graphics.ClipBounds.Height);
            e.Graphics.RotateTransform(270);
            e.Graphics.ScaleTransform((float)this.Height / (float)this.Width, (float)this.Width / (float)this.Height);
            base.OnPaint(e);
        }
    }

    public class HorizontalProgressBar2 : BSE.Windows.Forms.ProgressBar
    {
        private string m_Text;
        int offset = 0;
        int _min = 0;
        int _max = 0;
        int _value = 0;
        System.Windows.Forms.Label lbl1 = new System.Windows.Forms.Label();
        System.Windows.Forms.Label lbl = new System.Windows.Forms.Label();
        public bool reverse = false;
        int displayvalue = 0;

        public HorizontalProgressBar2()
            : base()
        {
        }

        public new int Value
        {
            get { return _value; }
            set
            {
                if (_value == value)
                    return;
                _value = value;
                displayvalue = _value;

                if (reverse)
                {
                    int dif = _value - Minimum;
                    _value = Maximum - dif;
                }

                int ans = _value + offset;
                if (ans <= base.Minimum)
                {
                    ans = base.Minimum + 1; // prevent an exception for the -1 hack
                }
                else if (ans >= base.Maximum)
                {
                    ans = base.Maximum;
                }
                
                base.Value = ans;

                if (this.DesignMode) return;

                if (this.Parent != null)
                {
                    this.Parent.Controls.Add(lbl);
                    this.Parent.Controls.Add(lbl1);
                }
            }
        }

        public new int Minimum
        {
            get { return _min; }
            set
            {
                _min = value;
                if (_min < 0)
                {
                    base.Minimum = 0; offset = (_max - value) / 2; base.Maximum = _max - value;
                }
                else
                {
                    base.Minimum = value;
                }
            }
        }

        public new int Maximum { get { return _max; } set { _max = value; base.Maximum = value; } }

        [System.ComponentModel.Browsable(true),
System.ComponentModel.Category("Mine"),
System.ComponentModel.Description("Text under Bar")]
        public string Label
        {
            get
            {
                return m_Text;
            }
            set
            {
                if (m_Text != value)
                {
                    m_Text = value;
                }
            }
        }

        private void drawlbl(Graphics e)
        {
            lbl.Location = new Point(this.Location.X, this.Location.Y + this.Height + 2);
            lbl.ClientSize = new Size(this.Width, 13);
            lbl.TextAlign = ContentAlignment.MiddleCenter;
            lbl.Text = m_Text;

            lbl1.Location = new Point(this.Location.X, this.Location.Y + this.Height + 15);
            lbl1.ClientSize = new Size(this.Width, 13);
            lbl1.TextAlign = ContentAlignment.MiddleCenter;
            lbl1.Text = displayvalue.ToString();

            if (minline != 0 && maxline != 0)
            {
                float range = this.Maximum - this.Minimum;
                float range2 = this.Width;
                Pen redPen = new Pen(Color.Red, 2);

                SolidBrush mybrush = new SolidBrush(Color.FromArgb(0x40, 0x57, 0x04));

                if ((Type)this.GetType() == typeof(VerticalProgressBar2)) {
                    e.ResetTransform();
                    range2 = this.Height;
                    if (reverse)
                    {
                        e.DrawLine(redPen, 0, (maxline - this.Minimum) / range * range2 + 0, this.Width, (maxline - this.Minimum) / range * range2 + 0);
                        e.DrawLine(redPen, 0, (minline - this.Minimum) / range * range2 + 6, this.Width, (minline - this.Minimum) / range * range2 + 6);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, mybrush, 5, (maxline - this.Minimum) / range * range2 + 2);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, Brushes.White, 5, (minline - this.Minimum) / range * range2 - 10);
                    }
                    else
                    {
                        e.DrawLine(redPen, 0, (this.Maximum - minline) / range * range2 + 0, this.Width, (this.Maximum - minline) / range * range2 + 0);
                        e.DrawLine(redPen, 0, (this.Maximum - maxline) / range * range2 + 6, this.Width, (this.Maximum - maxline) / range * range2 + 6);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, mybrush, 5, (this.Maximum - minline) / range * range2 + 2);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.White, 5, (this.Maximum - maxline) / range * range2 - 10);
                    }
                } else {
                    if (reverse)
                    {
                        e.DrawLine(redPen, (this.Maximum - minline) / range * range2 - 0, 0, (this.Maximum - minline) / range * range2 - 0, this.Height);
                        e.DrawLine(redPen, (this.Maximum - maxline) / range * range2 - 0, 0, (this.Maximum - maxline) / range * range2 - 0, this.Height);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, mybrush, (this.Maximum - minline) / range * range2 - 30, 5);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.White, (this.Maximum - maxline) / range * range2 - 0, 5);
                    }
                    else
                    {
                        e.DrawLine(redPen, (minline - this.Minimum) / range * range2 - 0, 0, (minline - this.Minimum) / range * range2 - 0, this.Height);
                        e.DrawLine(redPen, (maxline - this.Minimum) / range * range2 - 0, 0, (maxline - this.Minimum) / range * range2 - 0, this.Height);
                        e.DrawString(minline.ToString(), SystemFonts.DefaultFont, mybrush, (minline - this.Minimum) / range * range2 - 30, 5);
                        e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.White, (maxline - this.Minimum) / range * range2 - 0, 5);
                    }
                }
            }
        }

        public int minline { get; set; }
        public int maxline { get; set; }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            drawlbl(e.Graphics);
        }

    }

    public class HorizontalProgressBar : ProgressBar
    {
        private string m_Text;
        int offset = 0;
        int _min = 0;
        int _max = 0;
        int _value = 0;
        System.Windows.Forms.Label lbl1 = new System.Windows.Forms.Label();
        System.Windows.Forms.Label lbl = new System.Windows.Forms.Label();


        public HorizontalProgressBar()
            : base()
        {
            drawlbl();
            //this.Parent.Controls.AddRange(new Control[] { lbl, lbl1 });
        }

        public new int Value
        {
            get { return _value; }
            set
            {
                _value = value;
                int ans = value + offset;
                if (ans <= base.Minimum)
                {
                    ans = base.Minimum + 1; // prevent an exception for the -1 hack
                }
                else if (ans >= base.Maximum)
                {
                    ans = base.Maximum;
                }
                base.Value = ans;
                drawlbl();
                base.Value = ans - 1;
                drawlbl();
                base.Value = ans;
                drawlbl();
            }
        }

        public new int Minimum
        {
            get { return _min; }
            set
            {
                _min = value;
                if (_min < 0)
                {
                    base.Minimum = 0; offset = (_max - value) / 2; base.Maximum = _max - value;
                }
                else
                {
                    base.Minimum = value;
                }

                if (this.DesignMode) return;

                if (this.Parent != null)
                {
                    this.Parent.Controls.Add(lbl);
                    this.Parent.Controls.Add(lbl1);
                }
            }
        }

        public new int Maximum { get { return _max; } set { _max = value; base.Maximum = value; } }

        [System.ComponentModel.Browsable(true),
System.ComponentModel.Category("Mine"),
System.ComponentModel.Description("Text under Bar")]
        public string Label
        {
            get
            {
                return m_Text;
            }
            set
            {
                if (m_Text != value)
                {
                    m_Text = value;
                }
            }
        }

        private void drawlbl()
        {
            lbl.Location = new Point(this.Location.X, this.Location.Y + this.Height + 2);
            lbl.ClientSize = new Size(this.Width, 13);
            lbl.TextAlign = ContentAlignment.MiddleCenter;
            lbl.Text = m_Text;

            lbl1.Location = new Point(this.Location.X, this.Location.Y + this.Height + 15);
            lbl1.ClientSize = new Size(this.Width, 13);
            lbl1.TextAlign = ContentAlignment.MiddleCenter;
            lbl1.Text = Value.ToString();

            if (minline != 0 && maxline != 0)
            {
                float range = this.Maximum - this.Minimum;
                float range2 = this.Width;
                Graphics e = this.CreateGraphics();
                Pen redPen = new Pen(Color.Red, 2);

                if ((Type)this.GetType() == typeof(VerticalProgressBar)) {
                    range2 = this.Height;
                    e.DrawLine(redPen, 0, (this.Maximum - minline) / range * range2 + 0, this.Width, (this.Maximum - minline) / range * range2 + 0);
                    e.DrawLine(redPen, 0, (this.Maximum - maxline) / range * range2 + 0, this.Width, (this.Maximum - maxline) / range * range2 + 0);
                    e.DrawString(minline.ToString(), SystemFonts.DefaultFont, Brushes.Black, 5, (this.Maximum - minline) / range * range2 + 2);
                    e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.Black, 5, (this.Maximum - maxline) / range * range2 - 15);
                } else {
                    e.DrawLine(redPen, (minline - this.Minimum) / range * range2 - 3, 0, (minline - this.Minimum) / range * range2 - 3, this.Height);
                    e.DrawLine(redPen, (maxline - this.Minimum) / range * range2 - 3, 0, (maxline - this.Minimum) / range * range2 - 3, this.Height);
                    e.DrawString(minline.ToString(), SystemFonts.DefaultFont, Brushes.Black, (minline - this.Minimum) / range * range2 - 35, 5);
                    e.DrawString(maxline.ToString(), SystemFonts.DefaultFont, Brushes.Black, (maxline - this.Minimum) / range * range2 - 3, 5);
                }
            }
        }

        public int minline { get; set; }
        public int maxline { get; set; }

        protected override void OnPaint(PaintEventArgs e)
        {
            base.OnPaint(e);
            drawlbl();
        }
    }
}
