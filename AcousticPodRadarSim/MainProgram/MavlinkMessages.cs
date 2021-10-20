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

    public enum MAV_MSG_PROTOCOL
    {
        MAVLINK1 = 1,
        MAVLINK2 = 2,
    }
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
        public double time_boot_sec; ///< Timestamp (seconds since system boot)
        public double lat_d; ///< Latitude, expressed as degrees * 1E7
        public double lon_d; ///< Longitude, expressed as degrees * 1E7
        public double alt_m; ///< Altitude in meters, expressed as int * 1000 (millimeters), above MSL
        public double relative_alt; ///< Altitude above ground in meters, expressed as int * 1000 (millimeters)
        public double vx; ///< Ground X Speed (Latitude), expressed as m/s, short * 100
        public double vy; ///< Ground Y Speed (Longitude), expressed as m/s, short * 100
        public double vz; ///< Ground Z Speed (Altitude), expressed as m/s, short * 100
        public double hdg; ///< Compass heading in degrees, ushort * 100, 0.0..359.99 degrees. If unknown, set to: 65535
    } ;

    public struct mavlink_param_value_t
    {
        public object param_value; ///< Onboard parameter value
        public ushort param_count; ///< Total number of onboard parameters
        public ushort param_index; ///< Index of this onboard parameter
        public string param_id; ///< Onboard parameter id, terminated by NULL if the length is less than 16 human-readable chars and WITHOUT null termination (NULL) byte if the length is exactly 16 chars - applications have to provide 16+1 bytes storage if the ID is stored as string
        public MAV_PARAM_TYPE param_type; ///< Onboard parameter type: see the MAV_PARAM_TYPE enum for supported data types.
    } ;

    public struct mavlink_battery_status_t
    {
        public uint id;	                ///<uint8_t	Battery ID
        public uint battery_function;   ///<	uint8_t	Function of the battery
        public uint type;	            ///<uint8_t	Type (chemistry) of the battery
        public float temperature;	    ///<int16_t	Temperature of the battery in centi-degrees celsius. INT16_MAX for unknown temperature.
        public float voltages;	        ///<uint16_t[10]	Battery voltage of cells, in millivolts (1 = 1 millivolt). Cells above the valid cell count for this battery should have the UINT16_MAX value.
        public float current_battery;   ///<	int16_t	Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        public float current_consumed;	///<int32_t	Consumed charge, in milliampere hours (1 = 1 mAh), -1: autopilot does not provide mAh consumption estimate
        public float energy_consumed;	///<int32_t	Consumed energy, in HectoJoules (intergrated U*I*dt) (1 = 100 Joule), -1: autopilot does not provide energy consumption estimate
        public int battery_remaining;	///<int8_t	Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot does not estimate the remaining battery
    }

    public struct mavlink_sys_status_t
    {
        public UInt32 onboard_control_sensors_present;  //  uint32_t	Bitmask showing which onboard controllers and sensors are present. Value of 0: not present. Value of 1: present. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        public UInt32 onboard_control_sensors_enabled;  //	uint32_t	Bitmask showing which onboard controllers and sensors are enabled: Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        public UInt32 onboard_control_sensors_health;   //	uint32_t	Bitmask showing which onboard controllers and sensors are operational or have an error: Value of 0: not enabled. Value of 1: enabled. Indices defined by ENUM MAV_SYS_STATUS_SENSOR
        public double load;                             //	uint16_t	Maximum usage in percent of the mainloop time, (0%: 0, 100%: 1000) should be always below 1000
        public double voltage_battery;                  //	uint16_t	Battery voltage, in millivolts (1 = 1 millivolt)
        public double current_battery;                   //	int16_t	Battery current, in 10*milliamperes (1 = 10 milliampere), -1: autopilot does not measure the current
        public int battery_remaining;                  //	int8_t	Remaining battery energy: (0%: 0, 100%: 100), -1: autopilot estimate the remaining battery
        public double drop_rate_comm;                   //	uint16_t	Communication drops in percent, (0%: 0, 100%: 10'000), (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        public double errors_comm;                      //	uint16_t	Communication errors (UART, I2C, SPI, CAN), dropped packets on all links (packets that were corrupted on reception on the MAV)
        public UInt16 errors_count1;                    //	uint16_t	Autopilot-specific errors
        public UInt16 errors_count2;                    //	uint16_t	Autopilot-specific errors
        public UInt16 errors_count3;                    //	uint16_t	Autopilot-specific errors
        public UInt16 errors_count4;                    //	uint16_t	Autopilot-specific errors
    }
    public struct mavlink_system_time_t
    {
        public double time_unix_sec;               // Timestamp of the master clock in seconds since UNIX epoch
        public double time_boot_sec;                 // Timestamp of the component clock since boot time in seconds
        public double del_mav_cpu_sec;              // time difference between CPU UTC time and Mavlink message
    }

    public enum COPTER_MODE
    {
        COPTER_MODE_STABILIZE = 0,
        COPTER_MODE_ACRO = 1,
        COPTER_MODE_ALT_HOLD = 2,
        COPTER_MODE_AUTO = 3,
        COPTER_MODE_GUIDED = 4,
        COPTER_MODE_LOITER = 5,
        COPTER_MODE_RTL = 6,
        COPTER_MODE_CIRCLE = 7,
        COPTER_MODE_LAND = 9,
        COPTER_MODE_DRIFT = 11,
        COPTER_MODE_SPORT = 13,
        COPTER_MODE_FLIP = 14,
        COPTER_MODE_AUTOTUNE = 15,
        COPTER_MODE_POSHOLD = 16,
        COPTER_MODE_BRAKE = 17,
        COPTER_MODE_THROW = 18,
        COPTER_MODE_AVOID_ADSB = 19,
        COPTER_MODE_GUIDED_NOGPS = 20,
        COPTER_MODE_SMART_RTL = 21,
    }

    public class MavlinkMessages
    {
        public const int MAVLINK_MSG_ID_GLOBAL_POSITION_INT = 33;
        public const int MAVLINK_MSG_ID_SYS_STATUS = 1;
        public const int MAVLINK_MSG_ID_SYSTEM_TIME = 2;

        public const int MAVLINK1_HEADER_SIZE = 6;
        public const int MAVLINK2_HEADER_SIZE = 10;
        public const int MAVLINK_CHECKSUM_SIZE = 2;

        public const byte MAVLINK1_MAGIC_HDR = 0xfe;
        public const byte MAVLINK2_MAGIC_HDR = 0xfd;

        public MAV_MSG_PROTOCOL protocol = MAV_MSG_PROTOCOL.MAVLINK1;

        public int IPport;
        public string IPaddress;
        UdpClient mUdpClient;
        bool mRunning = true;
        Mutex parseMutex = new Mutex();
        byte[] buffer = new byte[2048];

        // add code for serial interface of 3DR radio
        public SerialPort _serialPort;
        public bool streamsStarted = false;

        public mavlink_battery_status_t uavBattery;
        public mavlink_global_position_int_t uavPos;
        public mavlink_sys_status_t uavSysStatus;
        public mavlink_system_time_t uavSysTime;
        protected MAVLink.MavlinkParse mavlinkParser;
        private IPEndPoint remoteEndPoint = new IPEndPoint(IPAddress.Any, 0);

        // define buffer to carry over message
        private byte[] previousData = new byte[0];

        private byte messageSequence = 0;  //track command message sequence

        object locker = new object();

        public void StartSerialPort(string portNum)
        {
            try
            {
                _serialPort = new SerialPort(portNum);
                _serialPort.BaudRate = 57600;
                _serialPort.Parity = Parity.None;
                _serialPort.DataBits = 8;
                _serialPort.StopBits = StopBits.One;
                _serialPort.Handshake = Handshake.None;
                _serialPort.DataReceived += new SerialDataReceivedEventHandler(_serialPort_DataReceived);

                _serialPort.ReadTimeout = 1000;
                _serialPort.WriteTimeout = 500;
                _serialPort.Open();
                _serialPort.DiscardOutBuffer();
                _serialPort.DiscardInBuffer();

            }
            catch
            {

            }
            try { _serialPort.DtrEnable = true; }
            catch { }
            try { _serialPort.RtsEnable = true; }
            catch { }
        }

        private void _serialPort_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            SerialPort sp = (SerialPort)sender;

            byte[] temp = new byte[sp.ReadBufferSize];
            try
            {
                int bytesRead = sp.Read(temp, 0, temp.Length);
                byte[] indata = new byte[bytesRead];
                Buffer.BlockCopy(temp, 0, indata, 0, bytesRead);

                parseMessage(ref indata);
            }
            catch { }
        }

        public void StopSerialCom()
        {
            if (_serialPort.IsOpen)
            {
                _serialPort.Close();
            }
        }

        public void StartUdpServer(int UavNum)
        {
            this.IPport = 14550 + UavNum;
            this.IPaddress = "192.168.1.50" + UavNum.ToString();

            mUdpClient = new UdpClient(this.IPport);
            mavlinkParser = new MAVLink.MavlinkParse();

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
            MavlinkMessages localProgram = (MavlinkMessages)result.AsyncState;
            IPEndPoint ipeEndpoint = new IPEndPoint(IPAddress.Any, localProgram.IPport);

            localProgram.parseMutex.WaitOne();
            if (result.IsCompleted)
            {
                buffer = localProgram.mUdpClient.EndReceive(result, ref ipeEndpoint);
                remoteEndPoint = ipeEndpoint;

                localProgram.parseMessage(ref buffer);
            }
            localProgram.parseMutex.ReleaseMutex();
        }

        public void SendUavHome(mavlinkInterface target)
        {
            // request return to launch
            byte[] msgPacket = new byte[0];
            ushort checksum;
            byte ck_a, ck_b;
            byte payloadLen = 0;
            int pointer = 0;
            byte tarSys = 1;
            byte tarComp = 1;
            float mode = (float)COPTER_MODE.COPTER_MODE_RTL;

            if (protocol == MAV_MSG_PROTOCOL.MAVLINK1)
            {
                payloadLen = 33;
                msgPacket = new byte[payloadLen + 8];
                msgPacket[pointer] = 0xfe; pointer++;
                msgPacket[pointer] = payloadLen; pointer++;
                msgPacket[pointer] = messageSequence; pointer++;
                msgPacket[pointer] = 0xff; pointer++;
                msgPacket[pointer] = 0xbe; pointer++;
                msgPacket[pointer] = (byte)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG; pointer++;

                (BitConverter.GetBytes((float)1.0)).CopyTo(msgPacket, pointer); pointer += sizeof(float);
                (BitConverter.GetBytes(mode)).CopyTo(msgPacket, pointer); pointer += sizeof(float);
                // skip next five floats
                pointer += 5 * sizeof(float);
                (BitConverter.GetBytes((ushort)MAVLink.MAV_CMD.DO_SET_MODE)).CopyTo(msgPacket, pointer); pointer += sizeof(short);
                msgPacket[pointer] = tarSys; pointer++;
                msgPacket[pointer] = tarComp; pointer++;
                msgPacket[pointer] = 0;


                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.COMMAND_LONG, checksum);  //50 is crc for heartbeat message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }
            else
            {
                payloadLen = 32;
                msgPacket = new byte[payloadLen + 12];
                msgPacket[pointer] = 0xfd; pointer++;
                msgPacket[pointer] = payloadLen; pointer++;
                pointer += 2; //skip flag bytes
                msgPacket[pointer] = messageSequence; pointer++;
                msgPacket[pointer] = 0xff; pointer++;
                msgPacket[pointer] = 0xbe; pointer++;
                msgPacket[pointer] = (byte)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG; pointer++;
                pointer += 2;  //next two bytes are zero

                (BitConverter.GetBytes((float)1.0)).CopyTo(msgPacket, pointer); pointer += sizeof(float);
                (BitConverter.GetBytes(mode)).CopyTo(msgPacket, pointer); pointer += sizeof(float);
                // skip next five floats
                pointer += 5 * sizeof(float);
                (BitConverter.GetBytes((ushort)MAVLink.MAV_CMD.DO_SET_MODE)).CopyTo(msgPacket, pointer); pointer += sizeof(short);
                msgPacket[pointer] = tarSys; pointer++;
                msgPacket[pointer] = tarComp;


                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.COMMAND_LONG, checksum);  //50 is crc for heartbeat message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }
            if (target == mavlinkInterface.Udp)
                SendCommandUdp(msgPacket);
            else
                SendCommandSerial(msgPacket);
            messageSequence++;
        }

        public void SetModeAuto(mavlinkInterface target)
        {
            // set mode of UAV to auto
            byte[] msgPacket = new byte[0];
            ushort checksum;
            byte ck_a, ck_b;
            byte payloadLen = 0;
            int pointer = 0;
            byte tarSys = 1;
            byte tarComp = 1;
            float mode = (float)COPTER_MODE.COPTER_MODE_AUTO;

            if (protocol == MAV_MSG_PROTOCOL.MAVLINK1)
            {
                payloadLen = 33;
                msgPacket = new byte[payloadLen + 8]; 
                msgPacket[pointer]=0xfe; pointer++;
                msgPacket[pointer]=payloadLen; pointer++;
                msgPacket[pointer]=messageSequence; pointer++;
                msgPacket[pointer]=0xff; pointer++;
                msgPacket[pointer]=0xbe; pointer++;
                msgPacket[pointer]=(byte)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG; pointer++;

                (BitConverter.GetBytes((float)1.0)).CopyTo(msgPacket,pointer); pointer+=sizeof(float);
                (BitConverter.GetBytes(mode)).CopyTo(msgPacket,pointer); pointer+=sizeof(float);
                // skip next five floats
                pointer += 5 * sizeof(float);
                (BitConverter.GetBytes((ushort)MAVLink.MAV_CMD.DO_SET_MODE)).CopyTo(msgPacket, pointer); pointer += sizeof(short);
                msgPacket[pointer] = tarSys; pointer++;
                msgPacket[pointer] = tarComp; pointer++;
                msgPacket[pointer] = 0; 


                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.COMMAND_LONG, checksum);  //50 is crc for heartbeat message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }
            else
            {
                payloadLen = 32;
                msgPacket = new byte[payloadLen + 12];
                msgPacket[pointer] = 0xfd; pointer++;
                msgPacket[pointer] = payloadLen; pointer++;
                pointer += 2; //skip flag bytes
                msgPacket[pointer] = messageSequence; pointer++;
                msgPacket[pointer] = 0xff; pointer++;
                msgPacket[pointer] = 0xbe; pointer++;
                msgPacket[pointer] = (byte)MAVLink.MAVLINK_MSG_ID.COMMAND_LONG; pointer++;
                pointer += 2;  //next two bytes are zero

                (BitConverter.GetBytes((float)1.0)).CopyTo(msgPacket, pointer); pointer += sizeof(float);
                (BitConverter.GetBytes(mode)).CopyTo(msgPacket, pointer); pointer += sizeof(float);
                // skip next five floats
                pointer += 5 * sizeof(float);
                (BitConverter.GetBytes((ushort)MAVLink.MAV_CMD.DO_SET_MODE)).CopyTo(msgPacket, pointer); pointer += sizeof(short);
                msgPacket[pointer] = tarSys; pointer++;
                msgPacket[pointer] = tarComp;


                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.COMMAND_LONG, checksum);  //50 is crc for heartbeat message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }
            if (target == mavlinkInterface.Udp)
                SendCommandUdp(msgPacket);
            else
                SendCommandSerial(msgPacket);
            messageSequence++;
        }

        public void SendHeartBeat(mavlinkInterface target)
        {
            byte[] msgPacket = new byte[0];
            ushort checksum;
            byte ck_a, ck_b;

            if (protocol == MAV_MSG_PROTOCOL.MAVLINK1)
            {
                msgPacket = new byte[17] { 0xfe, 9, messageSequence, 0xff, 0xbe, 0, 0, 0, 0, 0, 6, 8, 0, 0, 3, 0, 0 };
                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.HEARTBEAT, checksum);  //50 is crc for heartbeat message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }
            else
            {
                msgPacket = new byte[21] { 0xfd, 9, 0, 0, messageSequence, 0xff, 0xbe, 0, 0, 0, 0, 0, 0, 0, 6, 8, 0, 0, 3, 0, 0 };
                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.HEARTBEAT, checksum);  //50 is crc for heartbeat message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }

            if (target == mavlinkInterface.Udp)
                SendCommandUdp(msgPacket);
            else
                SendCommandSerial(msgPacket);
            messageSequence++;
        }


        public void RequestDataStream(MAVLink.MAV_DATA_STREAM streamId, int msgFreq, int start, mavlinkInterface target)
        {
            byte[] msgPacket = new byte[0];
            ushort checksum;
            byte ck_a, ck_b;
            byte tarSys = 1;
            byte tarComp = 1;
            byte[] freq = BitConverter.GetBytes((short)msgFreq);

            if (protocol == MAV_MSG_PROTOCOL.MAVLINK1)
            {
                msgPacket = new byte[14] { 0xfe, 6, messageSequence, 0xff, 0xbe, 0x42, freq[0], freq[1], tarSys, tarComp, (byte)streamId, (byte)start, 0, 0 };
                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.REQUEST_DATA_STREAM, checksum);  //148 is crc for request stream message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }
            else
            {
                msgPacket = new byte[18] { 0xfd, 6, 0, 0, messageSequence, 0xff, 0xbe, 0x42, 0, 0, freq[0], freq[1], tarSys, tarComp, (byte)streamId, (byte)start, 0, 0 };
                checksum = MavlinkCRC.crc_calculate(msgPacket, msgPacket.Length - 2);
                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.REQUEST_DATA_STREAM, checksum);  //148 is crc for request stream message
                ck_a = (byte)(checksum & 0xFF); ///< High byte
                ck_b = (byte)(checksum >> 8); ///< Low byte

                msgPacket[msgPacket.Length - 2] = ck_a;
                msgPacket[msgPacket.Length - 1] = ck_b;
            }

            if (target == mavlinkInterface.Udp)
                SendCommandUdp(msgPacket);
            else
                SendCommandSerial(msgPacket);
            messageSequence++;
        }

        public void SendCommandUdp(byte[] message)
        {
            try
            {
                mUdpClient.Send(message, message.Length, remoteEndPoint);
                Thread.Sleep(10);   //Mission Planner waits ~10 ms and then repeats a commands to ensure receipt
                mUdpClient.Send(message, message.Length, remoteEndPoint);
            }
            catch (Exception ex)
            {
                Console.WriteLine(ex.ToString());
            }
        }

        public void SendCommandSerial(byte[] message)
        {
            try
            {
                if (_serialPort.IsOpen)
                    _serialPort.Write(message, 0, message.Length);
            }
            catch { }
        }
        //stop running and exit
        void Stop()
        {
            mRunning = false;
        }
        //example for how to break apart messages
        void parseMessage(ref byte[] msgIn)
        {
            byte[] msg = new byte[msgIn.Length];
            List<byte> tempList = new List<byte>();
            byte[] temp2 = new byte[0];
            bool previousDataFlag = false;

            lock (locker)
            {
                msgIn.CopyTo(msg, 0);  // work with a copy of the msg
            }

            // if carry over from previous message, include
            if (previousData.Length != 0)
            {
                byte[] tmsg = new byte[previousData.Length + msg.Length];
                Buffer.BlockCopy(previousData, 0, tmsg, 0, previousData.Length);
                Buffer.BlockCopy(msg, 0, tmsg, previousData.Length, msg.Length);
                msg = tmsg;
                previousData = new byte[0];
                previousDataFlag = true;
            }
            //We need to be able to read at least the length of the payload
            for (int offset = 0; offset < msg.Length; )
            {
                //look for magic header
                if (msg[offset] == MAVLINK1_MAGIC_HDR)
                {
                    protocol = MAV_MSG_PROTOCOL.MAVLINK1;
                    if ((msg.Length - offset) == 1)
                    {
                        previousData = new byte[1];
                        previousData[0] = msg[offset];
                        break;
                    }
                    int msg_offset = MAVLINK1_HEADER_SIZE;
                    int len = msg[offset + 1]; //length of payload, not including header and checksum

                    // need to handle messages that carries over multiple transmissions
                    if (offset + len + MAVLINK1_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE > msg.Length)
                    {
                        //save message to be combined with next message
                        previousData = new byte[msg.Length - offset];
                        Buffer.BlockCopy(msg, offset, previousData, 0, msg.Length - offset);
                        break;
                    }

                    int sequence = msg[offset + 2];
                    int msg_id = msg[offset + 5];
                    ushort msgChecksum = BitConverter.ToUInt16(msg, offset + len + MAVLINK1_HEADER_SIZE);
                    ushort checksum = 0;
                    byte[] datain = new byte[len + MAVLINK1_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE];
                    Buffer.BlockCopy(msg, offset, datain, 0, len + MAVLINK1_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE);
                    //I've done 2 message types as an example

                    switch (msg_id)
                    {
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // stream SR0_Position
                            {
                                // verify checksum
                                checksum = MavlinkCRC.crc_calculate(datain, len + MAVLINK1_HEADER_SIZE);
                                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.GLOBAL_POSITION_INT, checksum);  //104 is crc for global position message
                                if (checksum != msgChecksum)
                                    break;

                                uavPos.time_boot_sec = BitConverter.ToUInt32(msg, offset + msg_offset) / (double)1e3; msg_offset += sizeof(int);
                                uavPos.lat_d = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1e7; msg_offset += sizeof(int);
                                uavPos.lon_d = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1e7; msg_offset += sizeof(int);
                                uavPos.alt_m = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1000.0; msg_offset += sizeof(int);
                                uavPos.relative_alt = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1000; msg_offset += sizeof(int);
                                uavPos.vx = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavPos.vy = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavPos.vz = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavPos.hdg = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(ushort);

                                // if first time through, streams are now started
                                if (!streamsStarted)
                                    streamsStarted = true;
                            }
                            break;


                        case MAVLINK_MSG_ID_SYS_STATUS:  // stream SR0_EXT_STAT
                            {
                                // verify checksum
                                checksum = MavlinkCRC.crc_calculate(datain, len + MAVLINK1_HEADER_SIZE);
                                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.SYS_STATUS, checksum);  //124 is crc for system status message
                                if (checksum != msgChecksum)
                                    break;
                                uavSysStatus.onboard_control_sensors_present = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                uavSysStatus.onboard_control_sensors_enabled = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                uavSysStatus.onboard_control_sensors_health = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                uavSysStatus.load = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)10.0; msg_offset += sizeof(ushort);
                                uavSysStatus.voltage_battery = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)1000.0; msg_offset += sizeof(ushort);
                                uavSysStatus.current_battery = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavSysStatus.drop_rate_comm = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(ushort);
                                uavSysStatus.errors_comm = (double)BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count1 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count2 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count3 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count4 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.battery_remaining = (int)msg[offset + msg_offset]; msg_offset += sizeof(Char);
                            }
                            break;
                        case MAVLINK_MSG_ID_SYSTEM_TIME: //stream SR0_EXTRA3
                            {
                                double utcTimeSec = (DateTime.UtcNow -
                                    new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalMilliseconds / 1000.0;
                                // synchronize with local CPU
                                uavSysTime.time_unix_sec = BitConverter.ToUInt64(msg, offset + msg_offset) / (double)1e6; msg_offset += sizeof(UInt64);
                                uavSysTime.del_mav_cpu_sec = utcTimeSec - uavSysTime.time_unix_sec;

                                uavSysTime.time_boot_sec = BitConverter.ToUInt32(msg, offset + msg_offset) / (double)1e3; msg_offset += sizeof(UInt32);

                            }
                            break;
                    }
                    offset += len + MAVLINK1_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE; //header + checksum = 8
                }
                else if (msg[offset] == MAVLINK2_MAGIC_HDR)
                {
                    protocol = MAV_MSG_PROTOCOL.MAVLINK2;
                    if ((msg.Length - offset) == 1)
                    {
                        previousData = new byte[1];
                        previousData[0] = msg[offset];
                        break;
                    }

                    int msg_offset = MAVLINK2_HEADER_SIZE;

                    int len = msg[offset + 1]; //length of payload, not including header and checksum

                    // need to handle messages that carries over multiple transmissions
                    if (offset + len + MAVLINK2_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE > msg.Length)
                    {
                        //save message to be combined with next message
                        previousData = new byte[msg.Length - offset];
                        Buffer.BlockCopy(msg, offset, previousData, 0, msg.Length - offset);
                        break;
                    }
                    byte incompat_flags = msg[offset + 2];
                    byte compat_flags = msg[offset + 3];
                    byte sequence = msg[offset + 4];

                    byte[] temp = new byte[4];  //used for getting msg id
                    Buffer.BlockCopy(msg, offset + 7, temp, 0, 3);
                    int msg_id = BitConverter.ToInt32(temp, 0);

                    ushort msgChecksum = BitConverter.ToUInt16(msg, offset + len + MAVLINK2_HEADER_SIZE);
                    ushort checksum = 0;
                    byte[] datain = new byte[len + MAVLINK2_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE];
                    Buffer.BlockCopy(msg, offset, datain, 0, len + MAVLINK2_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE);

                    switch (msg_id)
                    {
                        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:  // stream SR0_Position
                            {
                                // verify checksum
                                checksum = MavlinkCRC.crc_calculate(datain, len + MAVLINK2_HEADER_SIZE);
                                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.GLOBAL_POSITION_INT, checksum);  //104 is crc for global position message
                                if (checksum != msgChecksum)
                                    break;
                                uavPos.time_boot_sec = BitConverter.ToUInt32(msg, offset + msg_offset) / (double)1e3; msg_offset += sizeof(int);
                                uavPos.lat_d = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1e7; msg_offset += sizeof(int);
                                uavPos.lon_d = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1e7; msg_offset += sizeof(int);
                                uavPos.alt_m = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1000.0; msg_offset += sizeof(int);
                                uavPos.relative_alt = BitConverter.ToInt32(msg, offset + msg_offset) / (double)1000; msg_offset += sizeof(int);
                                uavPos.vx = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavPos.vy = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavPos.vz = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavPos.hdg = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(ushort);

                                // if first time through, streams are now started
                                if (!streamsStarted)
                                    streamsStarted = true;
                            }
                            break;

                        case MAVLINK_MSG_ID_SYS_STATUS:  // stream SR0_EXT_STAT
                            {
                                // verify checksum
                                checksum = MavlinkCRC.crc_calculate(datain, len + MAVLINK2_HEADER_SIZE);
                                checksum = MavlinkCRC.crc_accumulate((byte)MSG_CRC.SYS_STATUS, checksum);  //124 is crc for system status message
                                if (checksum != msgChecksum)
                                    break;
                                uavSysStatus.onboard_control_sensors_present = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                uavSysStatus.onboard_control_sensors_enabled = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                uavSysStatus.onboard_control_sensors_health = BitConverter.ToUInt32(msg, offset + msg_offset); msg_offset += sizeof(int);
                                uavSysStatus.load = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)10.0; msg_offset += sizeof(ushort);
                                uavSysStatus.voltage_battery = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)1000.0; msg_offset += sizeof(ushort);
                                uavSysStatus.current_battery = BitConverter.ToInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(short);
                                uavSysStatus.drop_rate_comm = BitConverter.ToUInt16(msg, offset + msg_offset) / (double)100.0; msg_offset += sizeof(ushort);
                                uavSysStatus.errors_comm = (double)BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count1 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count2 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count3 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.errors_count4 = BitConverter.ToUInt16(msg, offset + msg_offset); msg_offset += sizeof(ushort);
                                uavSysStatus.battery_remaining = (int)msg[offset + msg_offset]; msg_offset += sizeof(Char);
                            }
                            break;
                        case MAVLINK_MSG_ID_SYSTEM_TIME: //stream SR0_EXTRA3
                            {
                                double utcTimeSec = (DateTime.UtcNow -
                                    new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalMilliseconds / 1000.0;
                                int fullPayloadSize = sizeof(UInt64) + sizeof(UInt32);
                                // synchronize with local CPU
                                uavSysTime.time_unix_sec = BitConverter.ToUInt64(msg, offset + msg_offset) / (double)1e6; msg_offset += sizeof(UInt64);
                                uavSysTime.del_mav_cpu_sec = utcTimeSec - uavSysTime.time_unix_sec;

                                // account for three byte messages
                                if (len < fullPayloadSize)
                                {
                                    temp = new byte[4];  //used for fill
                                    Buffer.BlockCopy(msg, offset + msg_offset, temp, 0, 4 + len - fullPayloadSize);
                                    uavSysTime.time_boot_sec = BitConverter.ToUInt32(temp, 0) / (double)1e3; msg_offset += sizeof(UInt32);
                                }
                                else
                                    uavSysTime.time_boot_sec = BitConverter.ToUInt32(msg, offset + msg_offset) / (double)1e3; msg_offset += sizeof(UInt32);
                            }
                            break;
                    }
                    offset += len + MAVLINK2_HEADER_SIZE + MAVLINK_CHECKSUM_SIZE; //checksum = 2

                }
                else
                    if (previousDataFlag)  // if message is stretched across multiple packets, need to save data
                    {
                        tempList.Add(msg[offset]);  // use list to make growing buffer
                        offset++;
                    }
                    else
                        //skip this message. sync up with data by waiting for message that starts with fd or fe
                        //break;
                        offset++;
            }
            if (tempList.Count > 0)
            {
                temp2 = new byte[previousData.Length + tempList.Count];
                Buffer.BlockCopy(tempList.ToArray(), 0, temp2, previousData.Length, tempList.Count);
                previousData = temp2;
            }
        }
    }
    public enum MSG_CRC
    {
		HEARTBEAT = 50,
		SYS_STATUS = 124,
		SYSTEM_TIME = 137,
		SET_MODE = 89,
		PARAM_REQUEST_READ = 214,
		PARAM_REQUEST_LIST = 159,
		PARAM_VALUE = 220,
		PARAM_SET = 168,
		GPS_RAW_INT = 24,
		GPS_STATUS = 23,
		SCALED_IMU = 170,
		RAW_IMU = 144,
		RAW_PRESSURE = 67,
		SCALED_PRESSURE = 115,
		ATTITUDE = 39,
		ATTITUDE_QUATERNION = 246,
		LOCAL_POSITION_NED = 185,
		GLOBAL_POSITION_INT = 104,
		RC_CHANNELS_SCALED = 237,
		RC_CHANNELS_RAW = 244,
		SERVO_OUTPUT_RAW = 222,
		MISSION_REQUEST_PARTIAL_LIST = 212,
		MISSION_WRITE_PARTIAL_LIST = 9,
		MISSION_ITEM = 254,
		MISSION_REQUEST = 230,
		MISSION_SET_CURRENT = 28,
		MISSION_CURRENT = 28,
		MISSION_REQUEST_LIST = 132,
		MISSION_COUNT = 221,
		MISSION_CLEAR_ALL = 232,
		MISSION_ITEM_REACHED = 11,
		MISSION_ACK = 153,
		SET_GPS_GLOBAL_ORIGIN = 41,
		GPS_GLOBAL_ORIGIN = 39,
		PARAM_MAP_RC = 78,
		MISSION_REQUEST_INT = 196,
		REQUEST_DATA_STREAM = 148,
		DATA_STREAM = 21,
		MANUAL_CONTROL = 243,
		RC_CHANNELS_OVERRIDE = 124,
		MISSION_ITEM_INT = 38,
		COMMAND_INT = 158,
		COMMAND_LONG = 152,
		COMMAND_ACK = 143,
		POWER_STATUS = 203,
		DATA_TRANSMISSION_HANDSHAKE = 29,
		ALTITUDE = 47,
		AUTOPILOT_VERSION = 178,
		FENCE_POINT = 78,
		FENCE_FETCH_POINT = 68,
		FENCE_STATUS = 189,
		AHRS = 127,
		AUTOPILOT_VERSION_REQUEST = 85,
		HOME_POSITION = 104,
		SET_HOME_POSITION = 85,

    }
}