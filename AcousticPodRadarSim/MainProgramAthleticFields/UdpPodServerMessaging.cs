using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Diagnostics;
using System.IO.Ports;
using System.IO;
using System.Threading.Tasks;

namespace AcousticPodSimRadarAsterix
{
    public class PodServerUdpMessaging
    {
        readonly object _locker = new object();
        public Queue<byte[]> msgQueue = new Queue<byte[]>();

        public int IPport;
        UdpClient mUdpClient;
        bool mRunning = true;
        Mutex parseMutex = new Mutex();


        public void StartUdpPodServer(int ipPort)
        {
            this.IPport = ipPort;
            mUdpClient = new UdpClient(this.IPport);
            while (mRunning)
            {
                IAsyncResult result = mUdpClient.BeginReceive(AsyncRecv, this);
                //Always time out. Never block. 
                result.AsyncWaitHandle.WaitOne(1000);
            }
        }

        //delegate that is invoked when packet is received
        public void AsyncRecv(System.IAsyncResult result)
        {
            PodServerUdpMessaging localProgram = (PodServerUdpMessaging)result.AsyncState;
            IPEndPoint ipeEndpoint = new IPEndPoint(IPAddress.Any, localProgram.IPport);

            localProgram.parseMutex.WaitOne();
            if (result.IsCompleted)
            {
                byte[] buffer = localProgram.mUdpClient.EndReceive(result, ref ipeEndpoint);

                {
                    if (buffer[0] == 0xe8 && buffer[1] == 0xa1)  // verify message ID
                    {
                        int len = BitConverter.ToUInt16(buffer, 2);
                        {
                            //queue full message to forward to cadets until they have separate server working
                            byte[] msg = new byte[len];
                            Array.Copy(buffer, 0, msg, 0, len);  // strip off message ID and size

                            // place messages in queue for main thread processing
                            lock (_locker)
                                msgQueue.Enqueue(msg);
                        }
                    }
                }
            }
            localProgram.parseMutex.ReleaseMutex();
        }
        //stop running and exit
        void Stop()
        {
            mRunning = false;
        }
    }
}
