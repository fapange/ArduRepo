using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;
using ZedGraph;
using GMap.NET;
using System.Xml; // GE xml alt reader
using ArdupilotMega;
using ArdupilotMega.GCSViews;

namespace ArdupilotMega
{
    public partial class BatteryGraph : Form
    {
        GraphPane lastPane = null;
        List<PointLatLngAlt> gelocs = new List<PointLatLngAlt>();
        List<PointLatLngAlt> planlocs = new List<PointLatLngAlt>();
        PointPairList distMarker = new PointPairList();
        PointPairList list1 = new PointPairList();
        PointPairList list2 = new PointPairList();
        float distance = 0;
        //double homealt = 0;

        //public BatteryGraph(List<PointLatLngAlt> locs, double homealt)
        public BatteryGraph(List<PointLatLngAlt> locs)
        {
            InitializeComponent();

            updatePoints(locs);

            //this.homealt = homealt / MainV2.cs.multiplierdist;

            //Form frm = Common.LoadingBox("Loading", "Downloading Google Earth Data");

            //gelocs = getGEAltPath(planlocs);

            //frm.Close();
        }

        private void BatteryGraph_Load(object sender, EventArgs e)
        {
            if (planlocs.Count <= 1)
            {
                this.Close();
                return;
            }
            // GE plot
            double a = 0;
            double increment = (distance / (gelocs.Count - 1));

            foreach (PointLatLngAlt geloc in gelocs)
            {
                list2.Add(a,geloc.Alt);

                Console.WriteLine(geloc.Lng + "," + geloc.Lat + "," + geloc.Alt);

                a+=increment;
            }

            updatePoints(planlocs);
            /*// Planner Plot
                        distance=0;
                        int count = 0;
                        PointLatLngAlt lastloc = null;
                        foreach (PointLatLngAlt planloc in planlocs)
                        {
                            if (lastloc != null)
                            {
                                distance += (float)planloc.GetDistance(lastloc) / 1000;
                            }

                            list1.Add(distance, 30, 0, planloc.Tag); // homealt
                            //list1.Add(a, 30, 0, a.ToString("0")); // homealt

                            lastloc = planloc;
                            count++;
                        }*/
            // draw graph
            CreateChart(zg1);
            timer1.Enabled = true;
            timer1.Start();
        }

        public void updatePoints(List<PointLatLngAlt> locs)
        {
            planlocs = locs;

            if (planlocs.Count <= 1)
            {
                MessageBox.Show("Please plan something first");
                return;
            }

            // Planner Plot
            distance = 0;
            PointLatLngAlt lastloc = null;

            if (distMarker.Count > 0)
                distMarker.Clear();

            distMarker.Add(new PointPair(0, 20));
            distMarker.Add(new PointPair(0, 105));

            if (list1.Count > 0)
                list1.Clear();

            foreach (PointLatLngAlt planloc in planlocs)
            {
                if (lastloc != null)
                {
                    distance += (float)planloc.GetDistance(lastloc) / 1000;
                }

                list1.Add(distance, 30, 0, planloc.Tag); // homealt
                lastloc = planloc;
            }
            if (lastPane != null)
                addWayPointLabels(lastPane);

        }
        
        List<PointLatLngAlt> getGEAltPath(List<PointLatLngAlt> list)
        {
            double alt = 0;
            double lat = 0;
            double lng = 0;

            int pos = 0;

            List<PointLatLngAlt> answer = new List<PointLatLngAlt>();

            //http://code.google.com/apis/maps/documentation/elevation/
            //http://maps.google.com/maps/api/elevation/xml
            string coords = "";

            /*foreach (PointLatLngAlt m in list)
            {
                answer.Add(new PointLatLngAlt(m.Lat, m.Lng, 0.0f,m.Tag));
            }*/
            
            foreach (PointLatLngAlt loc in list)
            {
                coords = coords + loc.Lat.ToString(new System.Globalization.CultureInfo("en-US")) + "," + loc.Lng.ToString(new System.Globalization.CultureInfo("en-US")) + "|";
            }
            coords = coords.Remove(coords.Length - 1);

            if (list.Count <= 2 || coords.Length > (2048 - 256) || distance > 50000)
            {
                MessageBox.Show("To many/few WP's or to Big a Distance " + (distance/1000) + "km");
                return answer;
            }

            try
            {
                using (XmlTextReader xmlreader = new XmlTextReader("http://maps.google.com/maps/api/elevation/xml?path=" + coords + "&samples=" + (distance / 100).ToString(new System.Globalization.CultureInfo("en-US")) + "&sensor=false"))
                {
                    while (xmlreader.Read())
                    {
                        xmlreader.MoveToElement();
                        switch (xmlreader.Name)
                        {
                            case "elevation":
                                alt = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                Console.WriteLine("DO it " + lat + " " + lng + " " + alt);
                                PointLatLngAlt loc = new PointLatLngAlt(lat,lng,alt,"");
                                answer.Add(loc);
                                pos++;
                                break;
                            case "lat":
                                lat = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                break;
                            case "lng":
                                lng = double.Parse(xmlreader.ReadString(), new System.Globalization.CultureInfo("en-US"));
                                break;
                            default:
                                break;
                        }
                    }
                }
            }
            catch { MessageBox.Show("Error getting GE data"); }
            
            return answer;
        }
        public void addWayPointLabels(GraphPane myPane)
        {
            if (myPane.GraphObjList.Count > 0) myPane.GraphObjList.Clear();
            
            foreach (PointPair pp in list1)
            {
                // Add a another text item to to point out a graph feature
                TextObj text = new TextObj((string)pp.Tag, pp.X, 25);
                // rotate the text 90 degrees
                text.FontSpec.Angle = 90;
                text.FontSpec.FontColor = Color.Red;
                // Align the text such that the Right-Center is at (700, 50) in user scale coordinates
                text.Location.AlignH = AlignH.Center;
                text.Location.AlignV = AlignV.Center;
                // Disable the border and background fill options for the text
                text.FontSpec.Fill.IsVisible = true;
                text.FontSpec.Border.IsVisible = true;
                myPane.GraphObjList.Add(text);
            }
        }


        /*public void CreateChart(ZedGraphControl zgc)
        {
            GraphPane myPane = zgc.GraphPane;

            // Set the titles and axis labels
            myPane.Title.Text = "Battery SOC Data";
            myPane.XAxis.Title.Text = "Distance (Km)";
            myPane.YAxis.Title.Text = "Delta SOC (%)";
            myPane.AddY2Axis("%soc"); myPane.Y2Axis.IsVisible = true;
            //myPane.Fill = new Fill(Brushes.Black);
            myPane.Chart.Fill = new Fill(Brushes.Beige);
            LineItem myCurve;

            //myCurve = myPane.AddCurve("Plan", list1, Color.Red, SymbolType.Diamond);
            //myCurve = myPane.AddCurve("GE", list2, Color.Green, SymbolType.None);
            myCurve = myPane.AddCurve("soc1", FlightPlanner.list1, Color.Green, SymbolType.None);       myCurve.Line.Width = 2;
            myCurve = myPane.AddCurve("soc2", FlightPlanner.list2, Color.Aquamarine, SymbolType.None);  myCurve.Line.Width = 2;
            myCurve = myPane.AddCurve("soc3", FlightPlanner.list3, Color.Orange, SymbolType.None);      myCurve.Line.Width = 2;
            myCurve = myPane.AddCurve("soc4", FlightPlanner.list4, Color.Magenta, SymbolType.None);     myCurve.Line.Width = 2;

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;
            myPane.YAxis.MajorGrid.IsVisible = true;

            //myPane.XAxis.Scale.Min = 0;
            //myPane.XAxis.Scale.Max = distance;

            foreach (PointPair pp in list1)
            {
                // Add a another text item to to point out a graph feature
                TextObj text = new TextObj((string)pp.Tag, pp.X, pp.Y);
                // rotate the text 90 degrees
                text.FontSpec.Angle = 90;
                text.FontSpec.FontColor = Color.White;
                // Align the text such that the Right-Center is at (700, 50) in user scale coordinates
                text.Location.AlignH = AlignH.Right;
                text.Location.AlignV = AlignV.Center;
                // Disable the border and background fill options for the text
                text.FontSpec.Fill.IsVisible = false;
                text.FontSpec.Border.IsVisible = false;
                myPane.GraphObjList.Add(text);
            }

            // Make the Y axis scale red
            //myPane.YAxis.Scale.Min = -10;
            //myPane.YAxis.Scale.Max = 10;
            myPane.YAxis.Scale.FontSpec.FontColor = Color.Black;
            myPane.YAxis.Title.FontSpec.FontColor = Color.Black;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Make the Y2 axis scale red
            myPane.Y2Axis.Scale.FontSpec.FontColor = Color.Black;
            myPane.Y2Axis.Title.FontSpec.FontColor = Color.Black;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.Y2Axis.MajorTic.IsOpposite = false;
            myPane.Y2Axis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.Y2Axis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.Y2Axis.Scale.Align = AlignP.Inside;

            // Fill the axis background with a gradient
            //myPane.Chart.Fill = new Fill(Color.White, Color.LightGray, 45.0f);

            // Calculate the Axis Scale Ranges
            try
            {
                zg1.AxisChange();
            }
            catch { }
        }*/
        
        public void CreateChart(ZedGraphControl zgc)
        {
            LineItem myCurve;
            GraphPane myPane;
            MasterPane myMaster = zgc.MasterPane;

            myMaster.PaneList.Clear();

            // Set the margins and the space between panes to 10 points
            myMaster.Margin.All = 0;
            myMaster.InnerPaneGap = 10;
            myMaster.Fill = new Fill(Color.Black);

            //--------------------
            // SOC Gradient Plot
            //
            myPane = new GraphPane();

            // Set the titles and axis labels
            myPane.Title.Text = "SOC Rate";
            myPane.XAxis.Title.Text = "Distance (Km)";
            myPane.YAxis.Title.Text = "SOC Rate (%/Km)";
            myPane.AddY2Axis("%soc"); myPane.Y2Axis.IsVisible = true;
            //myPane.Fill = new Fill(Color.Red);
            //myPane.Chart.Fill = new Fill(Brushes.Beige);
            myPane.BaseDimension = 6.0F;

            //myCurve = myPane.AddCurve("Plan", list1, Color.Red, SymbolType.Diamond);
            //myCurve = myPane.AddCurve("GE", list2, Color.Green, SymbolType.None);
            myCurve = myPane.AddCurve("soc1", FlightPlanner.socd_list1, Color.Green, SymbolType.None); myCurve.Line.Width = 2; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("soc2", FlightPlanner.socd_list2, Color.Orange, SymbolType.None); myCurve.Line.Width = 2; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("soc3", FlightPlanner.socd_list3, Color.Aquamarine, SymbolType.None); myCurve.Line.Width = 2; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("soc4", FlightPlanner.socd_list4, Color.Magenta, SymbolType.None); myCurve.Line.Width = 2; myCurve.Line.IsAntiAlias = true;

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;
            myPane.YAxis.MajorGrid.IsVisible = true;

            //myPane.XAxis.Scale.Min = 0;
            //myPane.XAxis.Scale.Max = distance;

            // Make the Y axis scale red
            //myPane.YAxis.Scale.Min = -10;
            //myPane.YAxis.Scale.Max = 10;
            myPane.YAxis.Scale.FontSpec.FontColor = Color.Black;
            myPane.YAxis.Title.FontSpec.FontColor = Color.Black;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Make the Y2 axis scale red
            myPane.Y2Axis.Scale.FontSpec.FontColor = Color.Black;
            myPane.Y2Axis.Title.FontSpec.FontColor = Color.Black;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.Y2Axis.MajorTic.IsOpposite = false;
            myPane.Y2Axis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.Y2Axis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.Y2Axis.Scale.Align = AlignP.Inside;
            myMaster.Add(myPane);

            //--------------------
            // SOC Plot
            //
            myPane = new GraphPane();

            // Set the titles and axis labels
            myPane.Title.Text = "Battery SOC Data";
            myPane.XAxis.Title.Text = "Distance (Km)";
            myPane.YAxis.Title.Text = "SOC (%)";
            myPane.AddY2Axis("%soc"); myPane.Y2Axis.IsVisible = true;
            //myPane.Fill = new Fill(Brushes.Black);
            //myPane.Chart.Fill = new Fill(Brushes.Beige);
            myPane.BaseDimension = 6.0F;

            //myCurve = myPane.AddCurve("GE", list2, Color.Green, SymbolType.None);
            myCurve = myPane.AddCurve("distMarker", distMarker, Color.Blue, SymbolType.None); 
            myCurve.Label.IsVisible = false; 
            myCurve.Line.Style = System.Drawing.Drawing2D.DashStyle.Custom; 
            myCurve.Line.DashOn = 4;
            myCurve.Line.DashOff = 2;
            myCurve.Line.Width = 2;

            myCurve = myPane.AddCurve("soc1", FlightPlanner.soc_list1, Color.Green, SymbolType.None); myCurve.Line.Width = 4; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("soc2", FlightPlanner.soc_list2, Color.Orange, SymbolType.None); myCurve.Line.Width = 4; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("soc3", FlightPlanner.soc_list3, Color.Aquamarine, SymbolType.None); myCurve.Line.Width = 4; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("soc4", FlightPlanner.soc_list4, Color.Magenta, SymbolType.None); myCurve.Line.Width = 4; myCurve.Line.IsAntiAlias = true;
            myCurve = myPane.AddCurve("project1", FlightPlanner.socp_list1, Color.Green, SymbolType.None); myCurve.Line.Width = 1; myCurve.Label.IsVisible = false; myCurve.Line.IsAntiAlias = true;       //myCurve.Line.DashOff = 0.9f; myCurve.Line.DashOn = 0.1f;
            myCurve.Line.Style = System.Drawing.Drawing2D.DashStyle.Custom;
            myCurve.Line.DashOn = 3;
            myCurve.Line.DashOff = 6;
            myCurve = myPane.AddCurve("project2", FlightPlanner.socp_list2, Color.Orange, SymbolType.None); myCurve.Line.Width = 1; myCurve.Label.IsVisible = false; myCurve.Line.IsAntiAlias = true;    //myCurve.Line.DashOff = 0.9f; myCurve.Line.DashOn = 0.1f;
            myCurve.Line.Style = System.Drawing.Drawing2D.DashStyle.Dash;
            myCurve = myPane.AddCurve("project3", FlightPlanner.socp_list3, Color.Aquamarine, SymbolType.None); myCurve.Line.Width = 1; myCurve.Label.IsVisible = false; myCurve.Line.IsAntiAlias = true;        //myCurve.Line.DashOff = 0.9f; myCurve.Line.DashOn = 0.1f;
            myCurve.Line.Style = System.Drawing.Drawing2D.DashStyle.Dash;
            myCurve = myPane.AddCurve("project4", FlightPlanner.socp_list4, Color.Magenta, SymbolType.None); myCurve.Line.Width = 1; myCurve.Label.IsVisible = false; myCurve.Line.IsAntiAlias = true;        //myCurve.Line.DashOff = 0.9f; myCurve.Line.DashOn = 0.1f;
            myCurve.Line.Style = System.Drawing.Drawing2D.DashStyle.Dash;

            myCurve = myPane.AddCurve("Plan", list1, Color.Red, SymbolType.Diamond); myCurve.Label.IsVisible = false; myCurve.Line.Width = 1; myCurve.Line.IsVisible = false; myCurve.Symbol.Size = 14; myCurve.Symbol.Fill = new Fill(Color.Gray);

            // Show the x axis grid
            myPane.XAxis.MajorGrid.IsVisible = true;
            myPane.YAxis.MajorGrid.IsVisible = true;

            //myPane.XAxis.Scale.Min = 0;
            //myPane.XAxis.Scale.Max = distance;

            addWayPointLabels(myPane);
            /*foreach (PointPair pp in list1)
            {
                // Add a another text item to to point out a graph feature
                TextObj text = new TextObj((string)pp.Tag, pp.X, 25);
                // rotate the text 90 degrees
                text.FontSpec.Angle = 90;
                text.FontSpec.FontColor = Color.Red;
                // Align the text such that the Right-Center is at (700, 50) in user scale coordinates
                text.Location.AlignH = AlignH.Center;
                text.Location.AlignV = AlignV.Center;
                // Disable the border and background fill options for the text
                text.FontSpec.Fill.IsVisible = true;
                text.FontSpec.Border.IsVisible = true;
                myPane.GraphObjList.Add(text);
            }*/

            // Make the Y axis scale red
            //myPane.YAxis.Scale.Min = -10;
            //myPane.YAxis.Scale.Max = 10;
            myPane.YAxis.Scale.FontSpec.FontColor = Color.Black;
            myPane.YAxis.Title.FontSpec.FontColor = Color.Black;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.YAxis.MajorTic.IsOpposite = false;
            myPane.YAxis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.YAxis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.YAxis.Scale.Align = AlignP.Inside;
            // Manually set the axis range
            //myPane.YAxis.Scale.Min = -1;
            //myPane.YAxis.Scale.Max = 1;

            // Make the Y2 axis scale red
            myPane.Y2Axis.Scale.FontSpec.FontColor = Color.Black;
            myPane.Y2Axis.Title.FontSpec.FontColor = Color.Black;
            // turn off the opposite tics so the Y tics don't show up on the Y2 axis
            myPane.Y2Axis.MajorTic.IsOpposite = false;
            myPane.Y2Axis.MinorTic.IsOpposite = false;
            // Don't display the Y zero line
            myPane.Y2Axis.MajorGrid.IsZeroLine = true;
            // Align the Y axis labels so they are flush to the axis
            myPane.Y2Axis.Scale.Align = AlignP.Inside;
            lastPane = myPane;
            myMaster.Add(myPane);

            // Fill the axis background with a gradient
            //myPane.Chart.Fill = new Fill(Color.White, Color.LightGray, 45.0f);

            // Calculate the Axis Scale Ranges
            using (Graphics g = CreateGraphics())
            {
                myMaster.SetLayout(g, PaneLayout.SquareColPreferred);
                zgc.AxisChange();
            }
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            //CreateChart(zg1);
            //zg1.Invalidate();
            try
            {
                double odometer = 0;
                foreach (GraphPane m in zg1.MasterPane.PaneList)
                {

                    if (m.Equals(lastPane))
                    {
                        m.XAxis.Scale.Min = 0;
                        m.XAxis.Scale.Max = distance;
                        m.YAxis.Scale.Max = 105;
                        m.YAxis.Scale.Min = 20;
                        m.Y2Axis.Scale.Max = 105;
                        m.Y2Axis.Scale.Min = 20;
                        distMarker[0].X = odometer;
                        distMarker[1].X = odometer;
                    }
                    else
                    {
                        LineItem myCurve = m.CurveList[0] as LineItem;
                        m.XAxis.Scale.Min = myCurve.Points[0].X;
                        //myCurve = m.CurveList[m.CurveList.Count - 1] as LineItem;
                        m.XAxis.Scale.Max = myCurve.Points[myCurve.Points.Count - 1].X;
                    }
                    odometer = m.XAxis.Scale.Max;
                }
                zg1.AxisChange();
                zg1.Invalidate();
            }
            catch { }

        }
    }
}
