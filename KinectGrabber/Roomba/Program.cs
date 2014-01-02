using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace RobotController
{
    static class Program
    {

        [STAThread]
        static void Main()
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            FormMain form = new FormMain();
            Application.Run(form);

        }

    }
}