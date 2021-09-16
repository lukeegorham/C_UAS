using System;
using System.Drawing;
using System.Collections;
using System.ComponentModel;
using System.Windows.Forms;
using System.Data;
using System.Runtime.InteropServices;
using System.Diagnostics;
using System.Configuration;
using System.Threading;
using System.IO;
using System.Linq;

namespace ConvertRawData
{
    public partial class ConvertRawData : Form
    {
        private BinaryReader br;
        private LogType logType;

        public ConvertRawData()
        {
            InitializeComponent();
        }

        private void StartProcessing_Click(object sender, EventArgs e)
        {
            MessageBox.Text = "";
            try
            {
                br = new BinaryReader(new FileStream(InputFileNameBox.Text, FileMode.Open));
                StartProcessing.Enabled = false;
                StartProcessing.ForeColor = Color.Gray;
                bool testStr = "Acoustic".Equals(InputFileNameBox.Text.Substring(0, 8));
                bool testStr2 = "Aircraft".Equals(InputFileNameBox.Text.Substring(0, 8));
                if (testStr)
                    logType = LogType.PodLog;
                else if (testStr2)
                    logType = LogType.AircraftLog;
                else
                    logType = LogType.AoALog;

                ProcessMessage();

            }
            catch (IOException ex)
            {
                MessageBox.Text = ex.Message + "\n Cannot create file.";
                StartProcessing.Enabled = true;
                StartProcessing.ForeColor = Color.Black;
            }

        }
        private void ProcessMessage()
        {
            // Make terminator
            byte[] term = new byte[] { 9, 8, 9, 8, 9, 8 };
            int termLength = term.Length;
            byte[] readTerm = new byte[termLength];
            bool termNotFound = true;
            double systemTimeStamp = 0;
            double factor = Math.Pow(2.0, 16) / 2.0;  // mccdaq using 16 bits for numbers
            string lineOut = "";
            UInt16 msgType;
            int aircraftId;
            UInt16 podId;
            double cpuTimestamp;
            double timestamp;
            double posLat_deg;
            double posLong_deg;
            double posElev_mMSL;
            double posRelAlt_m;
            double posVelX_mps;
            double posVelY_mps;
            double posVelZ_mps;
            double posHdg_deg;
            UInt16 targetId=0;
            double AoA_deg=0.0;
            double batt_volt=0.0;
            bool currentSoloEstimate;
            bool currentRacerEstimate;
            double targetThreshold;
            double targetDegradation;
            double PodAoa_0_pod_pos_lat_deg;
            double PodAoa_0_pod_pos_lon_deg;
            double PodAoa_0_pod_pos_alt_mMSL;
            double PodAoa_0_pod_aoa;
            double PodAoa_1_pod_pos_lat_deg;
            double PodAoa_1_pod_pos_lon_deg;
            double PodAoa_1_pod_pos_alt_mMSL;
            double PodAoa_1_pod_aoa;
            double PodAoa_2_pod_pos_lat_deg;
            double PodAoa_2_pod_pos_lon_deg;
            double PodAoa_2_pod_pos_alt_mMSL;
            double PodAoa_2_pod_aoa;
            double aoaTargetEstimate_0_lat_deg;
            double aoaTargetEstimate_0_long_deg;
            double aoaTargetEstimate_0_elev_m;
            double trueTarget0_lat_deg = 0;
            double trueTarget0_long_deg = 0;
            double trueTarget0_elev_m = 0;
            bool targetDataPresent;


            // open output
            using (var w = new StreamWriter(OutputFileNameBox.Text, true))
            {

                while (br.BaseStream.Position < br.BaseStream.Length)
                {
                    if (br.BaseStream.Position == 249620)
                        PodAoa_0_pod_aoa = 0;
                    // find terminator sequence
                    while (termNotFound)
                    {
                        readTerm = br.ReadBytes(termLength);
                        if (term.SequenceEqual(readTerm))
                        {
                            termNotFound = false;
                        }
                        else
                        {
                            br.BaseStream.Position = br.BaseStream.Position - termLength + 1;  // position to next byte
                        }
                    }
                    switch (logType)
                    {
                        case LogType.PodLog:
                            {

                                // read data (timestamp,sampRate,
                                cpuTimestamp = br.ReadDouble();
                                systemTimeStamp = br.ReadDouble();
                                currentSoloEstimate = br.ReadBoolean();
                                currentRacerEstimate = br.ReadBoolean();
                                targetThreshold = br.ReadDouble();
                                targetDegradation = br.ReadDouble();
                                msgType = br.ReadUInt16();

                                if (msgType == 0 || msgType == 1)
                                {
                                    podId = br.ReadUInt16();
                                    timestamp = br.ReadDouble();
                                    posLat_deg = br.ReadDouble();
                                    posLong_deg = br.ReadDouble();
                                    posElev_mMSL = br.ReadDouble();
                                    if ((int)msgType == 1)
                                    {
                                        targetId = br.ReadUInt16();
                                        AoA_deg = br.ReadDouble();
                                    }
                                    else
                                    {
                                        batt_volt = br.ReadDouble();
                                    }
                                    // write line
                                    lineOut = cpuTimestamp.ToString() + "," +
                                        systemTimeStamp.ToString() + "," +
                                        currentSoloEstimate.ToString() + "," +
                                        currentRacerEstimate.ToString() + "," +
                                        targetThreshold.ToString() + "," +
                                        targetDegradation.ToString() + "," +
                                        msgType.ToString() + "," +
                                        podId.ToString() + "," +
                                        timestamp.ToString() + "," +
                                        posLat_deg.ToString() + "," +
                                        posLong_deg.ToString() + "," +
                                        posElev_mMSL.ToString() + ",";
                                    if ((int)msgType == 1)
                                    {
                                        lineOut += targetId.ToString() + "," +
                                            AoA_deg.ToString();
                                    }
                                    else
                                    {
                                        lineOut += batt_volt.ToString() + ",0";
                                    }
                                }
                                break;
                            }
                        case LogType.AircraftLog:
                            {

                                aircraftId = br.ReadInt32();
                                cpuTimestamp = br.ReadDouble();
                                systemTimeStamp = br.ReadDouble();
                                posLat_deg = br.ReadDouble();
                                posLong_deg = br.ReadDouble();
                                posElev_mMSL = br.ReadDouble();
                                posRelAlt_m = br.ReadDouble();
                                posVelX_mps = br.ReadDouble();
                                posVelY_mps = br.ReadDouble();
                                posVelZ_mps = br.ReadDouble();
                                posHdg_deg = br.ReadDouble();
                                // write line
                                lineOut = aircraftId.ToString() + "," +
                                    cpuTimestamp.ToString() + "," +
                                    systemTimeStamp.ToString() + "," +
                                    posLat_deg.ToString() + "," +
                                    posLong_deg.ToString() + "," +
                                    posElev_mMSL.ToString() + "," +
                                    posRelAlt_m.ToString() + "," +
                                    posVelX_mps.ToString() + "," +
                                    posVelY_mps.ToString() + "," +
                                    posVelZ_mps.ToString() + "," +
                                    posHdg_deg.ToString();
                                break;
                            }
                        case LogType.AoALog:
                            {
                                cpuTimestamp = br.ReadDouble();
                                PodAoa_0_pod_pos_lat_deg = br.ReadDouble();
                                PodAoa_0_pod_pos_lon_deg = br.ReadDouble();
                                PodAoa_0_pod_pos_alt_mMSL = br.ReadDouble();
                                PodAoa_0_pod_aoa = br.ReadDouble();
                                PodAoa_1_pod_pos_lat_deg = br.ReadDouble();
                                PodAoa_1_pod_pos_lon_deg = br.ReadDouble();
                                PodAoa_1_pod_pos_alt_mMSL = br.ReadDouble();
                                PodAoa_1_pod_aoa = br.ReadDouble();
                                PodAoa_2_pod_pos_lat_deg = br.ReadDouble();
                                PodAoa_2_pod_pos_lon_deg = br.ReadDouble();
                                PodAoa_2_pod_pos_alt_mMSL = br.ReadDouble();
                                PodAoa_2_pod_aoa = br.ReadDouble();
                                aoaTargetEstimate_0_lat_deg = br.ReadDouble();
                                aoaTargetEstimate_0_long_deg = br.ReadDouble();
                                aoaTargetEstimate_0_elev_m = br.ReadDouble();
                                targetDataPresent = br.ReadBoolean();
                                if (targetDataPresent)
                                {
                                    trueTarget0_lat_deg = br.ReadDouble();
                                    trueTarget0_long_deg = br.ReadDouble();
                                    trueTarget0_elev_m = br.ReadDouble();
                                }

                                // write line
                                lineOut =
                                    cpuTimestamp.ToString() + "," +
                                    PodAoa_0_pod_pos_lat_deg.ToString() + "," +
                                    PodAoa_0_pod_pos_lon_deg.ToString() + "," +
                                    PodAoa_0_pod_pos_alt_mMSL.ToString() + "," +
                                    PodAoa_0_pod_aoa.ToString() + "," +
                                    PodAoa_1_pod_pos_lat_deg.ToString() + "," +
                                    PodAoa_1_pod_pos_lon_deg.ToString() + "," +
                                    PodAoa_1_pod_pos_alt_mMSL.ToString() + "," +
                                    PodAoa_1_pod_aoa.ToString() + "," +
                                    PodAoa_2_pod_pos_lat_deg.ToString() + "," +
                                    PodAoa_2_pod_pos_lon_deg.ToString() + "," +
                                    PodAoa_2_pod_pos_alt_mMSL.ToString() + "," +
                                    PodAoa_2_pod_aoa.ToString() + "," +
                                    aoaTargetEstimate_0_lat_deg.ToString() + "," +
                                    aoaTargetEstimate_0_long_deg.ToString() + "," +
                                    aoaTargetEstimate_0_elev_m.ToString();
                                if (targetDataPresent)
                                {
                                    lineOut += "," +
                                    trueTarget0_lat_deg.ToString() + "," +
                                    trueTarget0_long_deg.ToString() + "," +
                                    trueTarget0_elev_m.ToString();
                                }

                                break;
                            }
                    }
                    // read end terminator
                    readTerm = br.ReadBytes(termLength);

                    termNotFound = true;

                    w.WriteLine(lineOut);
                    w.Flush();


                    progressBar1.Value = (int)(((double)br.BaseStream.Position /
                        (double)br.BaseStream.Length) * 100.0);
                }
                w.Close();
            }
            MessageBox.Text = "File conversion complete.";
            StartProcessing.Enabled = true;
            StartProcessing.ForeColor = Color.Black;

        }

        private void Clear_button_Click(object sender, EventArgs e)
        {
            InputFileNameBox.Text = "";
            OutputFileNameBox.Text = "";
        }
    }
    public enum LogType
    {
        PodLog = 0,
        AircraftLog = 1,
        AoALog=2,
    };
}
