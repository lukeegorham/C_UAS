using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Diagnostics;

/*
 * This example shows how to :
 *  1) Connect to Mavlink UDP stream
 *  2) Decode UDP packets into messages
 *  3) Decode messages into their component pieces
 * 
 *  For a complete list of messages see:
 *  https://pixhawk.ethz.ch/mavlink/
 *  Please note the description of the messages fields above are NOT in the order they are transmitted.
 *  To see the actual message structure you need to download the Mavlink 1.0 C library and peruse the headers there:
 *  https://github.com/mavlink/mavlink/
 *  I've converted two of the structures -- mavlink_global_position_int_t and mavlink_param_value_t -- and two of the messages types
 *  -- MAVLINK_MSG_ID_GLOBAL_POSITION_INT and MAVLINK_MSG_ID_PARAM_VALUE -- as an example. If you search for those strings in that library you can see what I did.
 *  
 */
namespace MavlinkExampleCSharp
{
    public enum MAV_PARAM_TYPE
    {
        MAV_PARAM_TYPE_UINT8 = 1, /* 8-bit unsigned integer | */
        MAV_PARAM_TYPE_INT8 = 2, /* 8-bit signed integer | */
        MAV_PARAM_TYPE_UINT16 = 3, /* 16-bit unsigned integer | */
        MAV_PARAM_TYPE_INT16 = 4, /* 16-bit signed integer | */
        MAV_PARAM_TYPE_UINT32 = 5, /* 32-bit unsigned integer | */
        MAV_PARAM_TYPE_INT32 = 6, /* 32-bit signed integer | */
        MAV_PARAM_TYPE_UINT64 = 7, /* 64-bit unsigned integer | */
        MAV_PARAM_TYPE_INT64 = 8, /* 64-bit signed integer | */
        MAV_PARAM_TYPE_REAL32 = 9, /* 32-bit floating-point | */
        MAV_PARAM_TYPE_REAL64 = 10, /* 64-bit floating-point | */
        MAV_PARAM_TYPE_ENUM_END = 11, /*  | */
    };
    public struct mavlink_global_position_int_t
    {
        public uint time_boot_ms; ///< Timestamp (milliseconds since system boot)
        public float lat; ///< Latitude, expressed as int * 1E7
        public float lon; ///< Longitude, expressed as int * 1E7
        public float alt; ///< Altitude in meters, expressed as int * 1000 (millimeters), above MSL
        public float relative_alt; ///< Altitude above ground in meters, expressed as int * 1000 (millimeters)
        public float vx; ///< Ground X Speed (Latitude), expressed as m/s, short * 100
        public float vy; ///< Ground Y Speed (Longitude), expressed as m/s, short * 100
        public float vz; ///< Ground Z Speed (Altitude), expressed as m/s, short * 100
        public float hdg; ///< Compass heading in degrees, ushort * 100, 0.0..359.99 degrees. If unknown, set to: 65535
    } ;

    public struct mavlink_param_value_t
    {
        public object param_value; ///< Onboard parameter value
        public ushort param_count; ///< Total number of onboard parameters
        public ushort param_index; ///< Index of this onboard parameter
        public string param_id; ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        public MAV_PARAM_TYPE param_type; ///< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    } ;

    class MavlinkMessages
    {
        public const int MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33;
        public const int MAVLINK_MSG_ID_PARAM_VALUE = 22;
        public const int MAVLINK_HEADER_SIZE = 6;
        public const int MAVLINK_CHECKSUM_SIZE = 2;

        UdpClient mUdpClientUav1;
        IPEndPoint mEndPointUav1;
        bool mRunning = true;
        Mutex parseMutex = new Mutex();

        //not in use
        public void CheckForInterrupt()
        {
                //blocks until keyhit:
                Console.ReadKey();
                Stop();
        }
        //delegate that is invoked when packet is received
        public static void AsyncRecv( System.IAsyncResult result )
        {
            MavlinkMessages localProgram = (MavlinkMessages)result.AsyncState;
            localProgram.parseMutex.WaitOne();
            if (result.IsCompleted)
            {
                byte[] buffer = localProgram.mUdpClientUav1.EndReceive(result, ref localProgram.mEndPointUav1);
                //Debug.WriteLine("Size = " + buffer.Length);
                localProgram.parseMessage(ref buffer);
            }
            localProgram.parseMutex.ReleaseMutex();
        }
        
        //stop running and exit
        void Stop()
        {
            mRunning = false;
        }
        //example for how to break apart messages
        void parseMessage(ref byte[] msg)
        {
            mavlink_global_position_int_t pos;
            mavlink_param_value_t param_value;

            //We need to be able to read at least the length of the payload
            for (int offset = 0; offset < msg.Length -1; )
            {
                //look for magic header
                if (msg[offset] == 0xfe)
                {
                    int msg_offset = MAVLINK_HEADER_SIZE; 
                    int len = msg[offset + 1]; //length of payload, not including header and checksum
                    //sanity check -- ditch this packet if it fails
                    if (offset + len + MAVLINK_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE > msg.Length)
                        break;
                   
                    int sequence = msg[offset + 2];
                    int msg_id = msg[offset + 5];
                    //I've done 2 message types as an example
                    //Look at 
                    switch (msg_id)
                    {
                            //This is an example of a "fixed" message
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
                            {
                                //floating point values have to be scaled from the integer format they're transmitted as
                                //sorry, I don't know of an elegant way to calculate offsets in C# like I do in C++ :( 
                                pos.time_boot_ms = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof( int );
                                pos.lat = BitConverter.ToInt32(msg, offset + msg_offset)/(float)1e7; msg_offset += sizeof(int);
                                pos.lon = BitConverter.ToInt32(msg, offset + msg_offset)/(float)1e7; msg_offset += sizeof(int);
                                pos.alt = BitConverter.ToInt32(msg,offset + msg_offset)/(float)1000.0; msg_offset += sizeof(int);
                                pos.relative_alt = BitConverter.ToInt32( msg, offset + msg_offset)/(float)1000; msg_offset += sizeof(int);
                                pos.vx = BitConverter.ToInt16( msg, offset + msg_offset)/(float)100.0; msg_offset += sizeof( short );
                                pos.vy = BitConverter.ToInt16(msg, offset + msg_offset) / (float)100.0; msg_offset += sizeof(short);
                                pos.vz = BitConverter.ToInt16(msg, offset + msg_offset) / (float)100.0; msg_offset += sizeof(short);
                                pos.hdg = BitConverter.ToUInt16(msg, offset + msg_offset) / (float)100.0; msg_offset += sizeof(ushort);
                                Console.WriteLine("time="+ pos.time_boot_ms + ", lat=" + pos.lat + ", lon=" + pos.lon + ", alt=" + pos.alt);
                            }
                            break;
                            //This is an example of the more abstract "param-value" format
                        case MAVLINK_MSG_ID_PARAM_VALUE:
                            {
                                msg_offset += sizeof(float); //skip the param_value for now
                                param_value.param_count = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                param_value.param_index = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                param_value.param_id = System.Text.Encoding.ASCII.GetString(msg, offset + msg_offset, 16); msg_offset += 16;
                                param_value.param_type = (MAV_PARAM_TYPE)msg[offset + msg_offset];
                                switch (param_value.param_type)
                                {
                                        //I leave the other types as an exercise to the user:
                                    case MAV_PARAM_TYPE.MAV_PARAM_TYPE_INT32:
                                        param_value.param_value = BitConverter.ToInt32(msg, offset + MAVLINK_HEADER_SIZE);
                                        Console.WriteLine(param_value.param_id + " = " + param_value.param_value.ToString());
                                        break;
                                    case MAV_PARAM_TYPE.MAV_PARAM_TYPE_REAL32:
                                        param_value.param_value = BitConverter.ToSingle(msg, offset + MAVLINK_HEADER_SIZE);
                                        Console.WriteLine(param_value.param_id + " = " + param_value.param_value.ToString());
                                        break;
                                }
                            }
                            break;
                    }
                    offset += len + 8; //header + checksum = 8
                }
                else
                    //ditch this packet -- not sure what we're reading
                    break;
            }
        }
        void DoExample()
        {
            mUdpClientUav1 = new UdpClient(14550);
            mEndPointUav1 = new IPEndPoint(0, 14550); ///0 address translates as INADDR_ANY, for people with C++ background
            while( mRunning )
            {
                IAsyncResult result = mUdpClientUav1.BeginReceive(AsyncRecv, this);
                //Always time out. Never block. 
                result.AsyncWaitHandle.WaitOne(1000);
                if (Console.KeyAvailable)
                    break;
            }
        }
        static void Main(string[] args)
        {
            MavlinkMessages mavlinkExample = new MavlinkMessages();
            //You don't really need this because you have AsyncWaitHandle so there's a "pause" that you can use to check the console
            //but I've left it in if you do want thread later for other reasons
           // Thread interruptThread = new Thread(new ThreadStart(mavlinkExample.CheckForInterrupt));
          //  interruptThread.Start();
          //  while (!interruptThread.IsAlive) ;
            Console.WriteLine("Press any key to quit.");
            mavlinkExample.DoExample();
            Console.WriteLine("Program exit.");
        }
    }
}
