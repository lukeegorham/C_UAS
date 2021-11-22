using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Net.Sockets;
using System.Net;
using System.Threading;
using System.Diagnostics;
using GeoCoordinateSystemNS;

namespace AcousticPodSimRadarAsterix
{
    public struct radar_estimate_struct
    {
        public double range_m;  //range estimate in meters
        public double az_deg; //azimuth estiamte in radians
        public double el_deg; //elevation estimate in radians
        public bool uavActive;  // track this UAV is in active use
    }

    public struct target_position_struct
    {
        public double targetId;  // target tracking
        public double time_unix_epoch_sec; // timestamp in seconds
        public double tar_lat_deg;  //true latitude of target in degrees for debug purposes
        public double tar_long_deg;  //true longitude of target in degrees for debug purposes
        public double tar_alt_m;  //true altitude of target in meters for debug purposes
    }
    public struct sensor_position_struct
    {
        public double sourceId; // source of sensor data (see enum SensorSource)
        public double time_unix_epoch_sec; // timestamp in seconds
        public double pos_lat_deg; //position of acoustics mast, latitude in degrees
        public double pos_long_deg; //position of acoustics mast, longitude in degrees
        public double pos_elev_m; //position of acoustics mast, land elevation in meters - MSL
        public double numMeasurements;  // number of measurements to be transmitted
        public double numTargets; // number of targets to be transmitted
    }
    
    public class UdpClientCl
    {
        public int IPport;
        UdpClient mUdpClient;
        Mutex parseMutex = new Mutex();
        byte[] buffer = new byte[2048];

        public void StartUdpClient(int ipPort)
        {
            this.IPport = ipPort;
            mUdpClient = new UdpClient(this.IPport);

        }

        public void Send(byte[] pckMsg, IPEndPoint currentUdpClient)
        {
            //send the byte array...
            mUdpClient.Send(pckMsg, pckMsg.Length, currentUdpClient);
        }
        
        //stop running and exit
        void Stop()
        {
        }
    }

    public class OutgoingMessage
    {
        readonly object _locker = new object();

        public byte[] packTrueUavLocationMsg(FormAcousticPodSimRadarAsterix.asset_status_t[] targetIn, bool isBigEndian)
        {
            FormAcousticPodSimRadarAsterix.asset_status_t[] target = new FormAcousticPodSimRadarAsterix.asset_status_t[targetIn.Length];
            lock (_locker)
                target=targetIn;

            int MaxBytes = 3 * sizeof(UInt16) + (sizeof(UInt16) * 2 + sizeof(double) * 4) * 4;

            byte[] tmpMsg = new byte[MaxBytes];

            //determine order of bytes based on Endianess of send format and local BitConverter
            bool reverseBytes = false;

            if (isBigEndian)
                if (BitConverter.IsLittleEndian)
                    reverseBytes = true;
                else
                    if (!BitConverter.IsLittleEndian)
                        reverseBytes = true;


            // find number of active targets
            int numtarget = 0;
            for (int i = 0; i < 4; i++)
            {
                if (target[i].EnableTrack)  // target is enabled in simulator
                    numtarget++;
            }

            // pack message
            int idx = 0;

            byte[] tmpAr = BitConverter.GetBytes((UInt16)MessageIdentifier.TrueUavPosition);
            if (BitConverter.IsLittleEndian)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            idx += sizeof(UInt16);  // jump over length location

            tmpAr = BitConverter.GetBytes((UInt16)numtarget);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            for (int k = 0; k < 4; k++)
            {
                if (target[k].EnableTrack)
                {
                    tmpAr = BitConverter.GetBytes((UInt16)k);
                    if (reverseBytes)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                    tmpAr = BitConverter.GetBytes((UInt16)target[k].uavAssociation);
                    if (reverseBytes)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                    tmpAr = BitConverter.GetBytes(target[k].unix_epoch_time_sec);
                    if (reverseBytes)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                    tmpAr = BitConverter.GetBytes(target[k].latitude_deg);
                    if (reverseBytes)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                    tmpAr = BitConverter.GetBytes(target[k].longitude_deg);
                    if (reverseBytes)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                    tmpAr = BitConverter.GetBytes(target[k].altitude_m);
                    if (reverseBytes)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                }
            }
            // fill in length
            tmpAr = BitConverter.GetBytes((UInt16)idx);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, tmpMsg, 2, 2);

            byte[] msgOut = new byte[idx];

            Array.Copy(tmpMsg, 0, msgOut, 0, idx);
            return msgOut;
        }

        public byte[] packPodTargetEstimateMsg(pod_target_estimate_struct podEstimateIn, bool isBigEndian)
        {
            pod_target_estimate_struct podEstimate = new pod_target_estimate_struct();
            lock (_locker)
                podEstimate = podEstimateIn;

            int MsgLength = 5 * sizeof(UInt16)  + 8 * sizeof(double);

            byte[] msgOut = new byte[MsgLength];

            //determine order of bytes based on Endianess of send format and local BitConverter
            bool reverseBytes = false;

            if (isBigEndian)
                if (BitConverter.IsLittleEndian)
                    reverseBytes = true;
                else
                    if (!BitConverter.IsLittleEndian)
                        reverseBytes = true;

            // pack message
            int idx = 0;

            byte[] tmpAr = BitConverter.GetBytes((UInt16)MessageIdentifier.PodTargetEstimate);
            if (BitConverter.IsLittleEndian)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes((UInt16)MsgLength);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.timestamp_sec);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.targetLatitude_deg);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.targetLongitude_deg);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.targetAltitude_mMSL);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.estimateSigma_m);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes((UInt16)podEstimate.pod1Id);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.pod1Aoa_deg);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes((UInt16)podEstimate.pod2Id);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.pod2Aoa_deg);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes((UInt16)podEstimate.pod3Id);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podEstimate.pod3Aoa_deg);
            if (reverseBytes)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, msgOut, idx, tmpAr.Length); idx += tmpAr.Length;

            return msgOut;
        }

        public byte[] packPodSimMsg(pod_asset_struct podAssetIn, bool isBigEndian)
        {
            pod_asset_struct podAsset = new pod_asset_struct();
            lock (_locker)
                podAsset = podAssetIn;

            int MaxBytes = 50;
            byte[] tmpMsg = new byte[MaxBytes];

            int idx = 0;
            //determine order of bytes based on Endianess of send format and local BitConverter
            // bytes are always little Endian to match actual pod data

            // pack message
            byte[] tmpAr = BitConverter.GetBytes((UInt16)MessageIdentifier.SimPodMessage);
            if (BitConverter.IsLittleEndian)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            idx += sizeof(UInt16);  // jump over length location

            tmpAr = BitConverter.GetBytes((UInt16)podAsset.receivedPodMsg.msgType);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes((UInt16)podAsset.receivedPodMsg.podId);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podAsset.receivedPodMsg.timestamp_sec);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podAsset.receivedPodMsg.lat_deg);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podAsset.receivedPodMsg.long_deg);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            tmpAr = BitConverter.GetBytes(podAsset.receivedPodMsg.elevation_mMSL);
            Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

            if (podAsset.receivedPodMsg.msgType == PodMsgType.PodHealth)
            {
                tmpAr = BitConverter.GetBytes(podAsset.receivedPodMsg.battery_volt);
                Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;
            }
            else //target message
            {
                tmpAr = BitConverter.GetBytes((UInt16)podAsset.receivedPodMsg.targetClassification);
                Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;

                tmpAr = BitConverter.GetBytes(podAsset.receivedPodMsg.aoa_deg);
                Array.Copy(tmpAr, 0, tmpMsg, idx, tmpAr.Length); idx += tmpAr.Length;
            }

            // fill in length
            tmpAr = BitConverter.GetBytes((UInt16)idx);
            Array.Copy(tmpAr, 0, tmpMsg, 2, 2);

            byte[] msgOut = new byte[idx];

            Array.Copy(tmpMsg, 0, msgOut, 0, idx);

            return msgOut;
        }
    }
}
