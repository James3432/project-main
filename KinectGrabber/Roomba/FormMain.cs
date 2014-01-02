using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Net.Sockets;
using System.Diagnostics;
using System.Threading;
using System.Windows.Threading;


namespace RobotController
{
    public partial class FormMain : Form
    {

        Controller c;
        int seq;
        TcpClient client;
        NetworkStream stream;
        Process win2;
        bool moving;
        bool robot_connected;

        //----------------------------------
        // RobotController form constructor
        //----------------------------------
        public FormMain()
        {
            InitializeComponent();

            LogHelp();

            btn_tcpcon.Focus();

            seq = 1;
            moving = false;

            win2 = Process.Start("C:\\Users\\James\\Documents\\Visual Studio 2010\\Projects\\RobotController\\Release\\KinectGrabber.exe");       

        }

        //---------------------
        // Robot connect
        //---------------------
        private void btn_connect_Click(object sender, EventArgs e)
        {
            try
            {
                c = new Controller(this);
                Log("Controller connected");
                btn_stop.Enabled = true;
                robot_connected = true;
            }
            catch (System.IO.IOException)
            {
                Log("Error connecting to controller");
            }
        }

        //--------------------
        // Robot disconnect
        //--------------------
        private void btn_disconn_Click(object sender, EventArgs e)
        {
            try
            {
                c.Kill();
                Log("Controller disconnected");
                btn_stop.Enabled = false;
                robot_connected = false;
            }
            catch (System.NullReferenceException)
            {
                Log("Controller not connected");
            }
        }

        //----------------------
        // Show initial log help
        //----------------------
        private void LogHelp()
        {
            txt_log.Items.Insert(0, "RobotController v1.0 for Roomba");
        }


        //---------------------------------
        // Write string to on-screen log
        //---------------------------------
        public void Log(String txt)
        {
            txt_log.Items.Insert(0,"---------------------");
            txt_log.Items.Insert(0,txt);
        }


        //----------------------
        // Connect TCP button
        //----------------------
        private void btn_tcpcon_Click(object sender, EventArgs e)
        {
            Dispatcher.CurrentDispatcher.Invoke(new Action(delegate
            {
                btn_tcptest.Enabled = false;
                btn_tcpcon.Enabled = false;
                btn_tcpdis.Enabled = false;
                btn_scan.Enabled = false;
            }));
            
            Connect("localhost");

        }

        //-------------------
        // Disconnect TCP
        //-------------------
        private void btn_tcpdis_Click(object sender, EventArgs e)
        {
            client.Close();
            btn_tcpcon.Enabled = true;
            btn_tcpdis.Enabled = false;
            btn_tcptest.Enabled = false;
            btn_scan.Enabled = false;
            Log("Disconnected from server");
        }

        //---------------
        // Test TCP link
        //---------------
        private void btn_tcptest_Click(object sender, EventArgs e)
        {
            TCPsend("Ping " + seq++);

        }

        //---------------------
        // On FormClose
        //---------------------
        private void FormMain_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                
                win2.Kill();
                c.Kill();

            }
            catch (Exception) { }

        }


        //-----------------------
        // Send string via TCP
        //-----------------------
        void TCPsend(String message)
        {
            try
            {
                Log("Sending message: " + message);
                // If we don't have a high resolution timer then Stopwatch will fall back
                // to DateTime, which is much less reliable
                //if (Stopwatch.IsHighResolution)
                //    Console.WriteLine("We have a high resolution timer available");

                //long frequency = Stopwatch.Frequency;
                //label2.Text=" Timer frequency in ticks per second = "+ frequency;

                // Start the stopwatch
                Stopwatch sw = Stopwatch.StartNew();




                Byte[] data = System.Text.Encoding.ASCII.GetBytes(message);
                // Send the message to the connected TcpServer. 
                stream.Write(data, 0, data.Length);

                // Buffer to store the response bytes.
                data = new Byte[256];

                // String to store the response ASCII representation.
                String responseData = String.Empty;

                // Read the first batch of the TcpServer response bytes.
                Int32 bytes = stream.Read(data, 0, data.Length);
                responseData = System.Text.Encoding.ASCII.GetString(data, 0, bytes);
                //Console.WriteLine("Received: {0}", responseData);
                if (responseData != "")
                {
                    //label1.Text = "Connected: " + responseData;
                    //timer1.Stop();
                    //label1.Text += " time: " + timer1.
                }
                else
                    //label1.Text = "failed";
                    Log("Message send error");

                // Stop the stopwatch
                sw.Stop();

                // Report the results
                Log("Latency (ms): " + sw.Elapsed.TotalMilliseconds);
                Log("Message sent,");
                //Console.WriteLine("Time used (rounded): {0} ms", sw.ElapsedMilliseconds);
            }catch(Exception e){
                Log("TCP error: "+e.Message);
            }

        }

        //--------------------------------
        // Start TCP connection to server
        //--------------------------------
        void Connect(String server)
        {
            try
            {
                Dispatcher.CurrentDispatcher.Invoke(new Action(delegate
                {
                    Log("Connecting to server...");
                }));
                // Create a TcpClient.
                // Note, for this client to work you need to have a TcpServer 
                // connected to the same address as specified by the server, port
                // combination.
                Int32 port = 13000;
                client = new TcpClient(server, port);

                Dispatcher.CurrentDispatcher.Invoke(new Action(delegate
                {
                   Log("Connected");
                   btn_tcpdis.Enabled = true;
                   btn_tcptest.Enabled = true;
                   btn_scan.Enabled = true;
                }));

                String message = "Ping " + seq++;

                // Translate the passed message into ASCII and store it as a Byte array.
               

                // Get a client stream for reading and writing.
                //  Stream stream = client.GetStream();

                stream = client.GetStream();

                TCPsend(message);

                

                //Console.WriteLine("Sent: {0}", message);

                // Receive the TcpServer.response.

                
                // Close everything.
                //client.Close();
            }
            catch (ArgumentNullException e)
            {
                //Console.WriteLine("ArgumentNullException: {0}", e);
                Log("Argument error");
                btn_tcpcon.Enabled = true;
                btn_tcpdis.Enabled = false;
                btn_tcptest.Enabled = false;
                btn_scan.Enabled = false;
            }
            catch (SocketException e)
            {
                //Console.WriteLine("SocketException: {0}", e);
                Log("Socket error");
                btn_tcpcon.Enabled = true;
                btn_tcpdis.Enabled = false;
                btn_tcptest.Enabled = false;
                btn_scan.Enabled = false;
            }
            catch(Exception e)
            {
                Log(e.Message);
                btn_tcpcon.Enabled = true;
                btn_tcpdis.Enabled = false;
                btn_tcptest.Enabled = false;
                btn_scan.Enabled = false;
            }

            //Console.WriteLine("\n Press Enter to continue...");
            //Console.Read();
        }
               

        //---------------------
        // Start scanning
        //---------------------
        private void btn_scan_Click(object sender, EventArgs e)
        {
            btn_stopscan.Enabled = true;
            btn_scan.Enabled = false;
            TCPsend("startGrab");
        }

        //---------------------
        // Stop scanning
        //---------------------
        private void btn_stopscan_Click(object sender, EventArgs e)
        {
            btn_stopscan.Enabled = false;
            btn_scan.Enabled = true;
            TCPsend("stopGrab");
        }

        //-------------
        // Stop robot
        //-------------
        private void btn_stop_Click(object sender, EventArgs e)
        {
            if (robot_connected)
            {
                moving = false;
                c.Stop();
                TCPsend("stop");
                img_ctrl.Image = RobotController.Properties.Resources.stop;
            }
        }       

        //-----------------
        // Allows KeyDown to capture arrow keys
        //-----------------
        private void FormMain_PreviewKeyDown(object sender, PreviewKeyDownEventArgs e)
        {
            //Log("key previewing");
            switch (e.KeyCode)
            {
                case Keys.Down:
                    //Log("previewed");
                    e.IsInputKey = true;
                    break;
                case Keys.Up:
                    e.IsInputKey = true;
                    break;
                case Keys.Left:
                    e.IsInputKey = true;
                    break;
                case Keys.Right:
                    e.IsInputKey = true;
                    break;
            }
        }

        //-------------
        // KeyDown
        //-------------
        private void FormMain_KeyDown(object sender, KeyEventArgs e)
        {
            if (robot_connected)
            {
                switch (e.KeyCode)
                {
                    case Keys.Down:
                        img_ctrl.Image = RobotController.Properties.Resources.back;
                        if (moving)
                            c.DriveStraight(-300);
                        else
                        {
                            moving = true;
                            c.SlowStart(-300);

                        }
                        //Log("Driving straight");
                        TCPsend("back");
                        break;
                    case Keys.Up:
                        img_ctrl.Image = RobotController.Properties.Resources.fwd;
                        if (moving)
                            c.DriveStraight(300);
                        else
                        {
                            moving = true;
                            c.SlowStart(300);

                        }
                        TCPsend("forward");
                        break;
                    case Keys.Left:
                        img_ctrl.Image = RobotController.Properties.Resources.left;
                        c.Rotate(1, 100);
                        TCPsend("left");
                        break;
                    case Keys.Right:
                        img_ctrl.Image = RobotController.Properties.Resources.right;
                        c.Rotate(-1, 100);
                        TCPsend("right");
                        break;
                    default:
                        return;
                }
                e.Handled = true;
            }
        }

        //---------------
        // KeyUp
        //---------------
        private void FormMain_KeyUp(object sender, KeyEventArgs e)
        {
            if (robot_connected)
            {
                switch (e.KeyCode)
                {
                    case Keys.Down:
                        c.SlowStop(-300);
                        //Log("Stopped");
                        moving = false;
                        break;
                    case Keys.Up:
                        //c.Stop();
                        c.SlowStop(300);
                        moving = false;
                        break;
                    case Keys.Left:
                        c.Stop();
                        break;
                    case Keys.Right:
                        c.Stop();
                        break;
                    default:
                        return;
                }
                TCPsend("stop");
                img_ctrl.Image = RobotController.Properties.Resources.stop;
                e.Handled = true;

            }
        }

    }
}
