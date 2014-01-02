using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

using System.IO.Ports;
using System.Threading;
using System.Windows.Threading;

namespace RobotController
{
    class Controller
    {
        //-----------
        // Variables
        //-----------
        SerialPort serial = new SerialPort("COM4", 57600);
        int totalAngle;
        int totalDistance;
        FormMain form;

        //-----------------------
        // Controller constructor
        //-----------------------
        public Controller(FormMain frm)
        {

            totalAngle = 0;
            totalDistance = 0;

            form = frm;

            serial.Open();

            lock (serial)
            {
                SerialRequest(0, 128);
                SerialRequest(0, 131);
            }

            //new Thread(Move).Start();
            new Thread(PollRobot).Start();
        }

        //------------
        // Destructor
        //------------
        ~Controller()
        {
            Kill();
        }

        //--------------
        // Test movement (not used)
        //--------------
        public void Move()
        {
            Thread.CurrentThread.IsBackground = true;
            Thread.Sleep(1500);
 
                while (true)
                {
                    DriveStraight(500);
                    Stop();
                    Turn(180, 150);
                }
            //Stop();
        }
 
        //------------------
        // Issue command over serial port
        //------------------
        private byte[] SerialRequest(int responseLength, params byte[] request)
        {
            lock (serial)
            {
                try
                {
                    serial.Write(request, 0, request.Length);
                }
                catch (InvalidOperationException) { }

                if (responseLength == 0)
                    return null;
 
                while (serial.BytesToRead < responseLength)
                    Thread.Sleep(1);
 
                byte[] buff = new byte[responseLength];
                serial.Read(buff, 0, responseLength);
 
                return buff;
            }
        }

        //------------------
        // Check robot status
        //------------------
        public void PollRobot()
        {
            Thread.CurrentThread.IsBackground = true;
 
 
            while (true)
            {
 
                lock (serial)
                {
                    byte[] angle = SerialRequest(2, 142, 20);
                    totalAngle += BitConverter.ToInt16(angle.Reverse().ToArray(), 0);
 
                    byte[] dist = SerialRequest(2, 142, 19);
                    totalDistance += BitConverter.ToInt16(dist.Reverse().ToArray(), 0);
 
                    byte[] bumper = SerialRequest(1, 142, 7);
 
                    byte[] overcurrent = SerialRequest(1, 142, 14);
 
                    EnsureSafety(bumper[0], overcurrent[0]);
 
                    // TODO: output dist/angle information somehow
                    //Dispatcher.CurrentDispatcher.Invoke(new Action(delegate{
                    //    form.Log("Distance: " + totalDistance + ", Angle: " + totalAngle);
                    //}));

                }
                Thread.Sleep(200);
 
            }
        }
 
        //---------------
        // Prevent overcurrent
        //---------------
        private static int overcurrentCount = 0;
        private void EnsureSafety(byte bumper, byte overcurrent)
        {
            if (bumper > 0)
                Kill();
 
            if (overcurrent > 0)
                overcurrentCount++;
            else
                overcurrentCount = 0;
 
            if (overcurrentCount > 5)
                Kill();
           
        }
 
        //----------
        // Disconnect
        //----------
        public void Kill()
        {
 
            Stop();
            SerialRequest(0, 128);
 
        }

        //---------------
        // Stop robot
        //---------------
        public void Stop()
        {
            SerialRequest(0, 137, 0, 0, 0, 0);
        }
 
        //----------------
        // Drive (fwds or back)
        //----------------
        public void DriveStraight(int speed)
        {
            byte[] b1 = BitConverter.GetBytes((short)speed);
            SerialRequest(0, 137, b1[1], b1[0], 0x7f, 0xff);
        }

        //---------------------
        // Rotate by 'angle'
        //---------------------
        public void Rotate(int angle, int speed)
        {
            byte[] speedBytes = BitConverter.GetBytes((short)speed);
            if (angle > 0)
            {
                SerialRequest(0, 137, speedBytes[1], speedBytes[0], 0x00, 0x01);
            }
            else
            {
                SerialRequest(0, 137, speedBytes[1], speedBytes[0], 0xff, 0xff);
            }
        }

        //-------------------------
        // Stop smoothly
        //-------------------------
        public void SlowStop(int speed)
        {
            if (speed > 0)
            {

                for (int i = speed; i > 0; i--)
                {
                    DriveStraight(i);
                    Thread.Sleep(1);
                }
            }
            else if (speed < 0)
            {
                for (int i = speed; i < 0; i++)
                {
                    DriveStraight(i);
                    Thread.Sleep(1);
                }
            }

            Stop();
        }

        //-----------------
        // Start smoothly
        //-----------------
        public void SlowStart(int speed)
        {
            if (speed > 0)
            {

                for (int i = 0; i < speed; i++)
                {
                    DriveStraight(i);
                    Thread.Sleep(1);
                }
            }
            else if (speed < 0)
            {
                for (int i = 0; i > speed; i--)
                {
                    DriveStraight(i);
                    Thread.Sleep(1);
                }
            }

        }

        //--------------------
        // Turn by fixed amount
        // (not suitable for direct control use)
        //--------------------
        public void Turn(int angle, int speed)
        {
 
 
            byte[] speedBytes = BitConverter.GetBytes((short)speed);
            int startAngle = totalAngle;
            int targetAngle = (startAngle + angle);
 
            if (angle > 0)
            {
                SerialRequest(0, 137, speedBytes[1], speedBytes[0], 0x00, 0x01);
 
                while (totalAngle < targetAngle)
                    Thread.Sleep(1);
 
                Stop();
                Thread.Sleep(500);
 
                if (totalAngle > targetAngle + 1)
                    Turn(-(totalAngle - targetAngle) + 2, 40);
            }
            else
            {
                SerialRequest(0, 137, speedBytes[1], speedBytes[0], 0xff, 0xff);
 
                while (totalAngle > targetAngle)
                    Thread.Sleep(1);
 
                Stop();
                Thread.Sleep(500);
 
                if (totalAngle < targetAngle - 1)
                    Turn(-(totalAngle - targetAngle) - 2, 40);
            }
            Stop();
 
        }

    }
}
