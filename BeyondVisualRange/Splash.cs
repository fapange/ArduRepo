using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Text;
using System.Windows.Forms;

namespace ArdupilotMega
{
    public partial class Splash : Form
    {
        public Splash()
        {
            InitializeComponent();

            TXT_version.Text = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString();
            //TXT_repo.Text = ((System.Reflection.AssemblyTrademarkAttribute)System.Reflection.Assembly.GetExecutingAssembly().GetCustomAttributes(true)[4]).Trademark;
            TXT_repo.Text = "null";
            TXT_date.Text = Application.ProductVersion;
        }
    }
}
