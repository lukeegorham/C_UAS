// Version 6 flashes the table names to help identify pod numbers
// Version 7 adds initial direction finding capability
// Version 8 added serial radio support
// Version 9 updated AoA algorithm

// New versions with AoA target estimates:
// Version 6: forward received pod messages to ensure one receive port for the cadets
// Version 7: Change AoA intersect algorithm to new version that addresses special cases such as target equal distance between pods 
// Version 8: Added false targets - only impacts radar messages to simulate L-star found targets
// Version 9: Corrected disconnects with radar parameters inputs from GUI

/*
 * Comm Strategy
 * Multiple communications are taking place with this program.  To generate simulated radar and acoustic signals, MAVLINK messages are
 * received.  MAVLINK messages of RTL and AUTO can also be sent.  The MAVLINK messages can either be received via UDP or Serial bus (USB).
 * Four messages are outputted to users.  All of these messages are UDP:
 * ASTERIX simulated radar messages: originated from port:
 * Acoustic pod messages: originated from port:
 * Acoustic position messages: originated from port:
 * UAV true position messages (for debug): originated from port:
 */

using System;
using System.Collections;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.IO;
using System.IO.Ports;
using System.Net;
using System.Threading;
using System.Runtime.InteropServices;
using System.Configuration;
using System.Diagnostics;
using System.Numerics;
using GeoCoordinateSystemNS;
using MathNet.Numerics.Random;




namespace AcousticPodSimRadarAsterix

{

    public partial class FormAcousticPodSimRadarAsterix : Form
    {
        // setup global variables
        readonly object _locker = new object();  // used to lock pod message queue when reading

        public int currColorCycle = 1; // used to track color cycles for pod detections (ex: Solo detection, flashing red and white circles)
        public static Dictionary<int, pod_asset_struct> podAssets = new Dictionary<int, pod_asset_struct>();  // dictionary to track acoustic pod assets
        public static Dictionary<int, int> PodAssociations = new Dictionary<int, int>();  // dictionary to associate pod Id with table location on GUI
        public int simPodMsgCntr = 0;  // want to stagger simulated Pod messages to be more representative of normal pods so counter used to only one message update per pod msg timer event

        // define threads used for messaging 
        public Thread udpPodServerMsgThread;  // Incoming messages from acoustic pod server
        public Thread uav1MsgThread;    //comms with UAV1
        public Thread uav2MsgThread;    //comms with UAV2
        public Thread uav3MsgThread;    //comms with UAV3
        public Thread uav4MsgThread;    //comms with UAV4
        public Thread udpOutgoingMsgThread;  // outgoing messages broadcast to all users

        // define message handling
        PodServerUdpMessaging incomingPodMsg = new PodServerUdpMessaging();  // Incoming message handling from acoustic pod server

        MavlinkMessages uavStatus1 = new MavlinkMessages();  // Message handling with UAV1
        MavlinkMessages uavStatus2 = new MavlinkMessages();  // Message handling with UAV2
        MavlinkMessages uavStatus3 = new MavlinkMessages();  // Message handling with UAV3
        MavlinkMessages uavStatus4 = new MavlinkMessages();  // Message handling with UAV4
        UdpClientCl udpClient = new UdpClientCl();  // Message handling to broadcast to users
        public string serialCommPort1;  //serial com port used by UAV1
        public string serialCommPort2;  //serial com port used by UAV2
        public string serialCommPort3;  //serial com port used by UAV3
        public string serialCommPort4;  //serial com port used by UAV4

        // IP ports can either be set up unique for message types or a common IP port may be used.  IP port numbers are defined
        // in the configuration file and assigned to variables here.
        public bool useCommonCommPorts = Convert.ToBoolean(ConfigurationManager.AppSettings["UseCommonCommPorts"]);
        public int commonReceivePort = Convert.ToInt32(ConfigurationManager.AppSettings["CommonReceivePort"]);
        public int commonTransmitPort = Convert.ToInt32(ConfigurationManager.AppSettings["CommonTransmitPort"]);
        public int podServerReceivePort = Convert.ToInt32(ConfigurationManager.AppSettings["PodServerReceivePort"]);
        public int podMsgOutputPort = Convert.ToInt32(ConfigurationManager.AppSettings["PodSimMsgOutputPort"]);
        public int podEstimateMsgOutputPort = Convert.ToInt32(ConfigurationManager.AppSettings["PodEstimateMsgOutputPort"]);
        public int radarEstimateMsgOutputPort = Convert.ToInt32(ConfigurationManager.AppSettings["RadarEstimateMsgOutputPort"]);
        public int trueUavMsgOutputPort = Convert.ToInt32(ConfigurationManager.AppSettings["TrueUavMsgOutputPort"]);
        public bool isUdpMessageFormatBigEndian = Convert.ToBoolean(ConfigurationManager.AppSettings["UdpMessageFormatBigEndian"]);
        public bool isIncomingNetworkLoopback = Convert.ToBoolean(ConfigurationManager.AppSettings["ReadLoopbackNetwork"]);

        // allocate IP end points for different message types
        public IPEndPoint podSimMsgEndpoint;
        public IPEndPoint podEstimateMsgEndpoint;
        public IPEndPoint radarEstimateEndpoint;
        public IPEndPoint trueUavMsgEndpoint;

        public OutgoingMessage outgoingMessage;  //properly pack outgoing messages 

        // allocate arrays for asterix messages
        public byte[] Asterix048Message;
        public byte[] Asterix034Message;
        public byte[] AsterixSpFieldMessage;

        private double gpsTimeOffset = 0.0; //time difference between CPU time and GPS time so log data is synced

        public PodDisplayColors displayColors = new PodDisplayColors();  //allocate colors for GUI

        // set up log names, log binary writers, and termination byte array
        private string rawDataFilenameBase = "AcousticPodDataLog";
        private string rawDataFilename;
        private BinaryWriter bw;
        private BinaryWriter bwAir;
        private BinaryWriter bwAoa;
        private byte[] term = new byte[] { 9, 8, 9, 8, 9, 8 };

        private string aircraftDataFilenameBase = "AircraftDataLog";
        private string aircraftDataFilename;

        private string aoaDataFilenameBase = "AoaDataLog";
        private string aoaDataFilename;

        private int trackNumber = 1;  // used as a counter as tracks are seen by the radar.  Track id used for Asterix messages

        private bool LogSimUav0 = Convert.ToBoolean(ConfigurationManager.AppSettings["AoaLogUavSimPos0"]);  // use to save simulated UAV0 position info for debug

        private const int maxQueueSize = 20;  // define the length of breadcrumbs for UAV

        // assign logicals from configuration file
        private bool simulateTracks = Convert.ToBoolean(ConfigurationManager.AppSettings["SimulationTrack"]);  // are UAV simulated or real UAVs used
        private bool simulatePods = Convert.ToBoolean(ConfigurationManager.AppSettings["SimulatePods"]);    //are acoustic pods simulated or real pods used
        private bool usePodLocationData = Convert.ToBoolean(ConfigurationManager.AppSettings["ArePodsSelfPositioning"]);  // do pods provide their own location
        private double podThresholdVoltage = Convert.ToDouble(ConfigurationManager.AppSettings["PodThresholdVoltage"]);  // pod battery voltage threshold for reporting poor health
        private double podLostCommTimeout_sec = Convert.ToDouble(ConfigurationManager.AppSettings["PodLostCommTimeout_sec"]);  // time for no pod messages before graying out pod on GUI
        private bool usePodServerGpsLocation = Convert.ToBoolean(ConfigurationManager.AppSettings["UsePodServerGpsPosition"]);  // use reported position of pod server to locate sim radar on GUI

        // used to determine display of acoustic pod targets on GUI.  Want to minimize reporting of momentary, false targets
        private double reportTargetThreshold = Convert.ToDouble(ConfigurationManager.AppSettings["ReportTargetThreshold"]);
        private double oldTargetDegradeRate = Convert.ToDouble(ConfigurationManager.AppSettings["OldTargetDegradeRate"]);
        private double sampleTime_sec = Convert.ToDouble(ConfigurationManager.AppSettings["SampleTime_sec"]);

        // define UAV associations - used for display accountability and identifying Solo IP port
        // Since Solo uses UDP messages, separate IP sockets must be set up for each Solo.  By our convention, these sockets tie
        // to the UAV number.  So the config file must contain the Solo UAV numbers that are being used for this program to open
        // the correct IP socket.
        private string uav1Association = Convert.ToString(ConfigurationManager.AppSettings["UAV1association"]);
        private string uav2Association = Convert.ToString(ConfigurationManager.AppSettings["UAV2association"]);
        private string uav3Association = Convert.ToString(ConfigurationManager.AppSettings["UAV3association"]);
        private string uav4Association = Convert.ToString(ConfigurationManager.AppSettings["UAV4association"]);
        private int uav1Number;
        private int uav2Number;
        private int uav3Number;
        private int uav4Number;

        // define the minimum altitude agl before radar messages are sent so no messages are sent when sitting on ground but powered on
        private double minAltToRegisterTarget_m_agl = Convert.ToDouble(ConfigurationManager.AppSettings["MinAltToRegisterRadar_m_agl"]);

        //Simulation of tracks variables
        public double uav1LatTemp;
        public double uav1LonTemp;
        public double uav1AltTemp;
        public double uav1VelXtemp;
        public double uav1VelYtemp;
        public double uav1VelZtemp;
        public double uav2LatTemp;
        public double uav2LonTemp;
        public double uav2AltTemp;
        public double uav2VelXtemp;
        public double uav2VelYtemp;
        public double uav2VelZtemp;
        public double uav3LatTemp;
        public double uav3LonTemp;
        public double uav3AltTemp;
        public double uav3VelXtemp;
        public double uav3VelYtemp;
        public double uav3VelZtemp;
        public double uav4LatTemp;
        public double uav4LonTemp;
        public double uav4AltTemp;
        public double uav4VelXtemp;
        public double uav4VelYtemp;
        public double uav4VelZtemp;

        public double simSpeedUav1;
        public double simSpeedUav2;
        public double simSpeedUav3;
        public double simSpeedUav4;
        public double[,] wayPointsUav1;
        public double[,] wayPointsUav2;
        public double[,] wayPointsUav3;
        public double[,] wayPointsUav4;
        public int wpIndexUav1;
        public int wpIndexUav2;
        public int wpIndexUav3;
        public int wpIndexUav4;
        public double[,] stepVectorsUav1;
        public double[,] stepVectorsUav2;
        public double[,] stepVectorsUav3;
        public double[,] stepVectorsUav4;
        public double turnToleranceUav1;
        public double turnToleranceUav2;
        public double turnToleranceUav3;
        public double turnToleranceUav4;

        // Simulate non targets
        private int numberOfNonTargets = Convert.ToInt32(ConfigurationManager.AppSettings["NumberOfNonTargets"]);
        private non_target_status_t[] non_target_List;
        private bool simulateNonTargets = false;


        internal static readonly DateTime Ref1970_Utc = new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc);  //baseline reference
        private Image MapImage;

        // radar and rod radius parameters
        public radar_parameters_t RadarParameter = new radar_parameters_t();
        public double rcs_dBsm = 0.1;  // default rcs to use in Asterix messages

        // map image coordinates - corners defined for proper pixel to lat/long conversion
        private double NWlat = Convert.ToDouble(ConfigurationManager.AppSettings["MissionMapNWLatitudeDeg"]);
        private double NWlon = Convert.ToDouble(ConfigurationManager.AppSettings["MissionMapNWLongitudeDeg"]);
        private double SElat = Convert.ToDouble(ConfigurationManager.AppSettings["MissionMapSELatitudeDeg"]);
        private double SElon = Convert.ToDouble(ConfigurationManager.AppSettings["MissionMapSELongitudeDeg"]);
        private int mouseDetectDistance_pixels = Convert.ToInt32(ConfigurationManager.AppSettings["MouseDetectDistance_pixels"]);

        private double delLon;
        private double delLat;
        private double deg2meterN;
        private double deg2meterE;

        // AoA tartget estimates
        private map_coordinate aoaTargetEstimate = new map_coordinate();


        private Point corner = new Point(0, 0);
        
        // Initialize assets 
        private asset_status_t[] asset_status_List = new asset_status_t[4];


        public FormAcousticPodSimRadarAsterix()
        {

            //don't worry about updating UI from another thread
            //you can remove if you have a better way of updating the UI thread
            TextBox.CheckForIllegalCrossThreadCalls = false;
            
            InitializeComponent();
            initializeServers();
            initializePodForm();

            // set up initial serial comm ports from configuration file.  May be changed in GUI.
            serialCommPort1 = ConfigurationManager.AppSettings["SerialCommPort1"];
            serCommPort1TextBox.Text = serialCommPort1;
            serialCommPort2 = ConfigurationManager.AppSettings["SerialCommPort2"];
            serCommPort2TextBox.Text = serialCommPort2;
            serialCommPort3 = ConfigurationManager.AppSettings["SerialCommPort3"];
            serCommPort3TextBox.Text = serialCommPort3;
            serialCommPort4 = ConfigurationManager.AppSettings["SerialCommPort4"];
            serCommPort4TextBox.Text = serialCommPort4;

            // set up UAV associations to no UAV (9999) if string is hyphen
            if (uav1Association != "-")
                uav1Number = Convert.ToInt32(uav1Association);
            else
                uav1Number = 9999;
            if (uav2Association != "-")
                uav2Number = Convert.ToInt32(uav2Association);
            else
                uav2Number = 9999;
            if (uav3Association != "-")
                uav3Number = Convert.ToInt32(uav3Association);
            else
                uav3Number = 9999;
            if (uav4Association != "-")
                uav4Number = Convert.ToInt32(uav4Association);
            else
                uav4Number = 9999;

            mainLoopTimer.Start();
            targetDecayTimer.Interval=(int)(sampleTime_sec*1000); // need milliseconds for timer
            targetDecayTimer.Start();
            mavlinkHeartbeat_timer.Start();
            asterix034_timer.Start();

            if(!simulatePods)
                checkPodMsgTimer.Start();

            // show associated UAV column labels
            Uav1label.Text = "UAV" + uav1Association;
            Uav2label.Text = "UAV" + uav2Association;
            Uav3label.Text = "UAV" + uav3Association;
            Uav4label.Text = "UAV" + uav4Association;

            // Config File initial pod server parameters
            RadarParameter.podRadius_m = Convert.ToDouble(ConfigurationManager.AppSettings["PodRadius_m"]);
            PodRadius_m.Text = RadarParameter.podRadius_m.ToString("f2");

            // Config File initial radar parameters
            RadarParameter.minRadarRange_m = Convert.ToDouble(ConfigurationManager.AppSettings["RadarMinDetRng_m"]);
            RadarParameter.maxRadarRange_m = Convert.ToDouble(ConfigurationManager.AppSettings["RadarMaxDetRng_m"]);
            RadarParameter.range_sigma_m = Convert.ToDouble(ConfigurationManager.AppSettings["RadarRangeSigma_m"]);
            RadarParameter.azimuth_beam_width_deg = Convert.ToDouble(ConfigurationManager.AppSettings["RadarAzBeamW_deg"]);
            RadarParameter.elevation_beam_width_deg = Convert.ToDouble(ConfigurationManager.AppSettings["RadarElBeamW_deg"]);
            RadarParameter.elevation_mMSL = Convert.ToDouble(ConfigurationManager.AppSettings["RadarElevation_mMSL"]);
            RadarParameter.latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["RadarLat_deg"]);
            RadarParameter.longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["RadarLong_deg"]);

            RadarMinDetRng_m.Text = RadarParameter.minRadarRange_m.ToString("f1");
            RadarMaxDetRng_m.Text = RadarParameter.maxRadarRange_m.ToString("f1");
            RangeSigma_m.Text = RadarParameter.range_sigma_m.ToString("f2");
            AzimuthBeamW_deg.Text = RadarParameter.azimuth_beam_width_deg.ToString("f2");
            ElevationBeamW_deg.Text = RadarParameter.elevation_beam_width_deg.ToString("f2");
            RadarElev_m.Text = RadarParameter.elevation_mMSL.ToString("f2");
            RadarLat_deg.Text = RadarParameter.latitude_deg.ToString("f6");
            RadarLong_deg.Text = RadarParameter.longitude_deg.ToString("f6");



            // generate references from radar position
            RadarParameter.posUtmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(RadarParameter.latitude_deg, RadarParameter.longitude_deg);
            // define Asterix set radar parameters for cat 034 messages
            RadarParameter.radarParam.S_N = 55;
            RadarParameter.radarParam.SAC = 0x19;  // per example captures
            RadarParameter.radarParam.SIC = 0xC4;  // per example captures
            RadarParameter.radarParam.rot_period_s = 2.5;  // per example captures
            RadarParameter.utcTime_sec = (DateTime.UtcNow -
                    Ref1970_Utc).TotalMilliseconds / 1000.0;
            RadarParameter.radarParam.PSRamplitude_dBm = 27;  //27dBm

            // set up simulated non-targets
            if (numberOfNonTargets > 0)
            {
                non_target_List = new non_target_status_t[numberOfNonTargets];
                initializeNonTargets();
            }

            // set up simulation
            // simulated pods (max of five)
            if (simulatePods)
            {
                pod_asset_struct podAssetPlaceholder;
                int formMappingId;  // map items on form to this pod
                double currentTime_sec = (DateTime.UtcNow -
                    Ref1970_Utc).TotalMilliseconds / 1000.0;

                podAssetPlaceholder=new pod_asset_struct();
                podAssetPlaceholder.timestamp_sec = currentTime_sec;
                podAssetPlaceholder.podHealthParameters.battery_volt = 10.0;
                podAssetPlaceholder.podHealthy = true;
                podAssetPlaceholder.lastSystemMsgTime_sec = currentTime_sec;
                podAssetPlaceholder.targetColorsSolo = Color.Gray;  //Solo
                podAssetPlaceholder.targetColorsRacer = Color.Gray;  //Racer
                podAssetPlaceholder.textColorsSolo = Color.Black;
                podAssetPlaceholder.textColorsRacer = Color.Black;
                podAssetPlaceholder.targetIdSolo = false;
                podAssetPlaceholder.targetIdRacer = false;
                podAssetPlaceholder.targetRacerRunningResult = 0.0;
                podAssetPlaceholder.targetSoloRunningResult = 0.0;

                int numPods = Convert.ToInt32(ConfigurationManager.AppSettings["NumberOfPods"]);
                formMappingId = 1;
                if (formMappingId < numPods)
                {
                    podAssetPlaceholder.podId =
                        Convert.ToInt32(ConfigurationManager.AppSettings["Pod1PodId"]);
                    podAssetPlaceholder.pos_lat_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod1Lat_deg"]);
                    podAssetPlaceholder.pos_long_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod1Long_deg"]);
                    podAssetPlaceholder.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(podAssetPlaceholder.pos_lat_deg, podAssetPlaceholder.pos_long_deg);
                    podAssetPlaceholder.pos_elev_m =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod1Elev_mMSL"]);
                    podAssetPlaceholder.isPodStereo = Convert.ToBoolean(ConfigurationManager.AppSettings["Pod1AoACapable"]);
                    podAssets.Add(formMappingId, podAssetPlaceholder);
                    PodAssociations.Add(podAssetPlaceholder.podId, formMappingId);
                    pod1Label.Text = "Pod " + podAssetPlaceholder.podId.ToString("d1");
                    pod1Lat_tbox.Text = podAssets[1].pos_lat_deg.ToString("f6");
                    pod1Long_tbox.Text = podAssets[1].pos_long_deg.ToString("f6");
                    pod1Label.Visible = true;
                    pod1Lat_tbox.Visible = true;
                    pod1Long_tbox.Visible = true;
                }
                if (formMappingId < numPods)
                {
                    formMappingId = 2;
                    podAssetPlaceholder.podId =
                        Convert.ToInt32(ConfigurationManager.AppSettings["Pod2PodId"]);
                    podAssetPlaceholder.pos_lat_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod2Lat_deg"]);
                    podAssetPlaceholder.pos_long_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod2Long_deg"]);
                    podAssetPlaceholder.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(podAssetPlaceholder.pos_lat_deg, podAssetPlaceholder.pos_long_deg);
                    podAssetPlaceholder.pos_elev_m =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod2Elev_mMSL"]);
                    podAssetPlaceholder.isPodStereo = Convert.ToBoolean(ConfigurationManager.AppSettings["Pod2AoACapable"]);
                    podAssets.Add(formMappingId, podAssetPlaceholder);
                    PodAssociations.Add(podAssetPlaceholder.podId, formMappingId);
                    pod2Label.Text = "Pod " + podAssetPlaceholder.podId.ToString("d1");
                    pod2Lat_tbox.Text = podAssets[2].pos_lat_deg.ToString("f6");
                    pod2Long_tbox.Text = podAssets[2].pos_long_deg.ToString("f6");
                    pod2Label.Visible = true;
                    pod2Lat_tbox.Visible = true;
                    pod2Long_tbox.Visible = true;
                }
                if (formMappingId < numPods)
                {
                    formMappingId = 3;
                    podAssetPlaceholder.podId =
                        Convert.ToInt32(ConfigurationManager.AppSettings["Pod3PodId"]);
                    podAssetPlaceholder.pos_lat_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod3Lat_deg"]);
                    podAssetPlaceholder.pos_long_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod3Long_deg"]);
                    podAssetPlaceholder.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(podAssetPlaceholder.pos_lat_deg, podAssetPlaceholder.pos_long_deg);
                    podAssetPlaceholder.pos_elev_m =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod3Elev_mMSL"]);
                    podAssetPlaceholder.isPodStereo = Convert.ToBoolean(ConfigurationManager.AppSettings["Pod3AoACapable"]);
                    podAssets.Add(formMappingId, podAssetPlaceholder);
                    PodAssociations.Add(podAssetPlaceholder.podId, formMappingId);
                    pod3Label.Text = "Pod " + podAssetPlaceholder.podId.ToString("d1");
                    pod3Lat_tbox.Text = podAssets[3].pos_lat_deg.ToString("f6");
                    pod3Long_tbox.Text = podAssets[3].pos_long_deg.ToString("f6");
                    pod3Label.Visible = true;
                    pod3Lat_tbox.Visible = true;
                    pod3Long_tbox.Visible = true;
                }

                if (formMappingId < numPods)
                {
                    formMappingId = 4;
                    podAssetPlaceholder.podId =
                        Convert.ToInt32(ConfigurationManager.AppSettings["Pod4PodId"]);
                    podAssetPlaceholder.pos_lat_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod4Lat_deg"]);
                    podAssetPlaceholder.pos_long_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod4Long_deg"]);
                    podAssetPlaceholder.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(podAssetPlaceholder.pos_lat_deg, podAssetPlaceholder.pos_long_deg);
                    podAssetPlaceholder.pos_elev_m =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod4Elev_mMSL"]);
                    podAssetPlaceholder.isPodStereo = Convert.ToBoolean(ConfigurationManager.AppSettings["Pod4AoACapable"]);
                    podAssets.Add(formMappingId, podAssetPlaceholder);
                    PodAssociations.Add(podAssetPlaceholder.podId, formMappingId);
                    pod4Label.Text = "Pod " + podAssetPlaceholder.podId.ToString("d1");
                    pod4Lat_tbox.Text = podAssets[4].pos_lat_deg.ToString("f6");
                    pod4Long_tbox.Text = podAssets[4].pos_long_deg.ToString("f6");
                    pod4Label.Visible = true;
                    pod4Lat_tbox.Visible = true;
                    pod4Long_tbox.Visible = true;
                }

                if (formMappingId < numPods)
                {
                    formMappingId = 5;
                    podAssetPlaceholder.podId =
                        Convert.ToInt32(ConfigurationManager.AppSettings["Pod5PodId"]);
                    podAssetPlaceholder.pos_lat_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod5Lat_deg"]);
                    podAssetPlaceholder.pos_long_deg =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod5Long_deg"]);
                    podAssetPlaceholder.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(podAssetPlaceholder.pos_lat_deg, podAssetPlaceholder.pos_long_deg);
                    podAssetPlaceholder.pos_elev_m =
                        Convert.ToDouble(ConfigurationManager.AppSettings["Pod5Elev_mMSL"]);
                    podAssetPlaceholder.isPodStereo = Convert.ToBoolean(ConfigurationManager.AppSettings["Pod5AoACapable"]);
                    podAssets.Add(formMappingId, podAssetPlaceholder);
                    PodAssociations.Add(podAssetPlaceholder.podId, formMappingId);
                    pod4Label.Text = "Pod " + podAssetPlaceholder.podId.ToString("d1");
                    pod4Lat_tbox.Text = podAssets[5].pos_lat_deg.ToString("f6");
                    pod4Long_tbox.Text = podAssets[5].pos_long_deg.ToString("f6");
                    pod4Label.Visible = true;
                    pod4Lat_tbox.Visible = true;
                    pod4Long_tbox.Visible = true;
                }

                // Define target types from config file
                if (Convert.ToBoolean(ConfigurationManager.AppSettings["Uav1IsTargetSolo"]))
                    asset_status_List[0].targetType = TargetType.Solo;
                else
                    asset_status_List[0].targetType = TargetType.Racer;

                if (Convert.ToBoolean(ConfigurationManager.AppSettings["Uav2IsTargetSolo"]))
                    asset_status_List[1].targetType = TargetType.Solo;
                else
                    asset_status_List[1].targetType = TargetType.Racer;

                if (Convert.ToBoolean(ConfigurationManager.AppSettings["Uav3IsTargetSolo"]))
                    asset_status_List[2].targetType = TargetType.Solo;
                else
                    asset_status_List[2].targetType = TargetType.Racer;

                if (Convert.ToBoolean(ConfigurationManager.AppSettings["Uav4IsTargetSolo"]))
                    asset_status_List[3].targetType = TargetType.Solo;
                else
                    asset_status_List[3].targetType = TargetType.Racer;

            }



 
            //Define UAV colors
            asset_status_List[0].ellipse = new Ellipse();
            asset_status_List[1].ellipse = new Ellipse();
            asset_status_List[2].ellipse = new Ellipse();
            asset_status_List[3].ellipse = new Ellipse();

            asset_status_List[0].ellipse.color = Color.Orange;
            asset_status_List[1].ellipse.color = Color.Aqua;
            asset_status_List[2].ellipse.color = Color.Magenta;
            asset_status_List[3].ellipse.color = Color.Yellow;


            // set up UAV tracks
            for (int kk=0; kk < 4; kk++)
            {
                asset_status_List[kk].EnableTrack = false;
                asset_status_List[kk].trackQueue = new Queue(maxQueueSize);
            }
            utmCoord_t tempLocationNW = new utmCoord_t();
            utmCoord_t tempLocationSE = new utmCoord_t();
            tempLocationNW = LatLonUtmWgs84Conv.LLtoUTM_Degrees(
                NWlat, NWlon);
            tempLocationSE = LatLonUtmWgs84Conv.LLtoUTM_Degrees(
                SElat, SElon);
            deg2meterN = (NWlat - SElat) / (tempLocationNW.UTMNorthing - tempLocationSE.UTMNorthing);
            deg2meterE = (SElon - NWlon) / (tempLocationSE.UTMEasting - tempLocationNW.UTMEasting);
        }

        private void setupIpEndpoints()
        {
            string endPoint;
            if(isIncomingNetworkLoopback)
                endPoint = "127.0.0.1";
            else
                endPoint = "192.168.1.255";
            // may be configured to send to individual IP ports or a common port
            if(useCommonCommPorts)
            {
                podSimMsgEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), commonTransmitPort);  // For sending data;
                podEstimateMsgEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), commonTransmitPort); ;
                radarEstimateEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), commonTransmitPort); ;
                trueUavMsgEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), commonTransmitPort); ;
            }
            else
            {
                podSimMsgEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), podMsgOutputPort);  // For sending data;
                podEstimateMsgEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), podEstimateMsgOutputPort);
                radarEstimateEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), radarEstimateMsgOutputPort);
                trueUavMsgEndpoint = new IPEndPoint(IPAddress.Parse(endPoint), trueUavMsgOutputPort);
            }
        }


        private void initializePodForm()
        {
            pod1Label.Visible = false;
            pod2Label.Visible = false;
            pod3Label.Visible = false;
            pod4Label.Visible = false;
            pod5Label.Visible = false;
            pod6Label.Visible = false;
            pod7Label.Visible = false;
            pod8Label.Visible = false;
            pod9Label.Visible = false;
            pod1Lat_tbox.Visible = false;
            pod2Lat_tbox.Visible = false;
            pod3Lat_tbox.Visible = false;
            pod4Lat_tbox.Visible = false;
            pod5Lat_tbox.Visible = false;
            pod6Lat_tbox.Visible = false;
            pod7Lat_tbox.Visible = false;
            pod8Lat_tbox.Visible = false;
            pod9Lat_tbox.Visible = false;
            pod1Long_tbox.Visible = false;
            pod2Long_tbox.Visible = false;
            pod3Long_tbox.Visible = false;
            pod4Long_tbox.Visible = false;
            pod5Long_tbox.Visible = false;
            pod6Long_tbox.Visible = false;
            pod7Long_tbox.Visible = false;
            pod8Long_tbox.Visible = false;
            pod9Long_tbox.Visible = false;

        }

        private void initializeServers()
        {
            // define IPendpoints for sending messages
            setupIpEndpoints();
            outgoingMessage = new OutgoingMessage();

            // start pod message thread
            if (!simulatePods)
            {
                try
                {
                    udpPodServerMsgThread = new Thread(() => incomingPodMsg.StartUdpPodServer(podServerReceivePort));
                    udpPodServerMsgThread.Start();
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error pod udp thread:" + ex.Message);
                }

            }
            // start UDP servers for any IP UAVs (Solo)
            if (uav1Association != "-" & !enableSerialComm1Checkbox.Checked)
            {
                try
                {
                    uav1MsgThread = new Thread(() => uavStatus1.StartUdpServer(Convert.ToInt32(uav1Association)));  //format used to pass parameters
                    uav1MsgThread.Start();
                    Console.WriteLine("Started Uav1 UDP listener thread");
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error with Uav1 thread:" + ex.Message);
                }
            }
            if (uav2Association != "-" & !enableSerialComm2Checkbox.Checked)
            {
                try
                {
                    uav2MsgThread = new Thread(() => uavStatus2.StartUdpServer(Convert.ToInt32(uav2Association)));  //format used to pass parameters
                    uav2MsgThread.Start();
                    Console.WriteLine("Started Uav2 UDP listener thread");
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error with Uav2 thread" + ex.Message);
                }
            }
            if (uav3Association != "-" & !enableSerialComm3Checkbox.Checked)
            {
                try
                {
                    uav3MsgThread = new Thread(() => uavStatus3.StartUdpServer(Convert.ToInt32(uav3Association)));  //format used to pass parameters
                    uav3MsgThread.Start();
                    Console.WriteLine("Started Uav3 UDP listener thread");
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error with Uav3 thread" + ex.Message);
                }
            }
            if (uav4Association != "-" & !enableSerialComm4Checkbox.Checked)
            {
                try
                {
                    uav4MsgThread = new Thread(() => uavStatus4.StartUdpServer(Convert.ToInt32(uav4Association)));  //format used to pass parameters
                    uav4MsgThread.Start();
                    Console.WriteLine("Started Uav4 UDP listener thread");
                }
                catch (Exception ex)
                {
                    Console.WriteLine("Error with Uav4 thread" + ex.Message);
                }
            }
            // set up UDP client for outgoing messages to users
            try
            {
                udpOutgoingMsgThread = new Thread(() => udpClient.StartUdpClient(commonReceivePort));
                udpOutgoingMsgThread.Start();
                Console.WriteLine("Started SimRadar UDP send thread");
            }
            catch (Exception ex)
            {
                Console.WriteLine("Error with simRadar thread" + ex.Message);
            }
             
        }

        // updateStates updates the UAV asset list with current information
        private void updateStates()
        {

            if (uavStatus1 != null && uav1Association != "-")
            {
                asset_status_List[0].unix_epoch_time_sec = uavStatus1.uavPos.time_boot_sec - uavStatus1.uavSysTime.time_boot_sec
                    + uavStatus1.uavSysTime.time_unix_sec + uavStatus1.uavSysTime.del_mav_cpu_sec + gpsTimeOffset;
                asset_status_List[0].uavAssociation = uav1Number;
                asset_status_List[0].latitude_deg = uavStatus1.uavPos.lat_d;
                asset_status_List[0].longitude_deg = uavStatus1.uavPos.lon_d;
                asset_status_List[0].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(asset_status_List[0].latitude_deg, asset_status_List[0].longitude_deg);
                asset_status_List[0].altitude_m = uavStatus1.uavPos.alt_m;
                asset_status_List[0].heading = uavStatus1.uavPos.hdg;
                asset_status_List[0].battery_voltage = uavStatus1.uavSysStatus.voltage_battery;
                asset_status_List[0].velX_mps = uavStatus1.uavPos.vx;
                asset_status_List[0].velY_mps = uavStatus1.uavPos.vy;
                asset_status_List[0].velZ_mps = uavStatus1.uavPos.vz;
                asset_status_List[0].radar_report.rcs_dBsm = rcs_dBsm;
                
                // log UAV position data
                if (logReceivedDataCheckBox.Checked)
                {
                    double currentTimeSub_sec = (DateTime.UtcNow -
                        Ref1970_Utc).TotalMilliseconds / 1000.0;
                    bwAir.Write(term);
                    bwAir.Write(System.Convert.ToInt32(uav1Association));
                    bwAir.Write(currentTimeSub_sec);
                    bwAir.Write(uavStatus1.uavPos.time_boot_sec);
                    bwAir.Write(uavStatus1.uavPos.lat_d);
                    bwAir.Write(uavStatus1.uavPos.lon_d);
                    bwAir.Write(uavStatus1.uavPos.alt_m);
                    bwAir.Write(uavStatus1.uavPos.relative_alt);
                    bwAir.Write(uavStatus1.uavPos.vx);
                    bwAir.Write(uavStatus1.uavPos.vy);
                    bwAir.Write(uavStatus1.uavPos.vz);
                    bwAir.Write(uavStatus1.uavPos.hdg);
                    bwAir.Write(term);
                }

            }
            if (uavStatus2 != null && uav2Association != "-")
            {
                asset_status_List[1].unix_epoch_time_sec = uavStatus2.uavPos.time_boot_sec - uavStatus2.uavSysTime.time_boot_sec
                    + uavStatus2.uavSysTime.time_unix_sec + uavStatus2.uavSysTime.del_mav_cpu_sec + gpsTimeOffset;
                asset_status_List[1].uavAssociation = uav2Number;
                asset_status_List[1].latitude_deg = uavStatus2.uavPos.lat_d;
                asset_status_List[1].longitude_deg = uavStatus2.uavPos.lon_d;
                asset_status_List[1].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(asset_status_List[1].latitude_deg, asset_status_List[1].longitude_deg);
                asset_status_List[1].altitude_m = uavStatus2.uavPos.alt_m;
                asset_status_List[1].heading = uavStatus2.uavPos.hdg;
                asset_status_List[1].battery_voltage = uavStatus2.uavSysStatus.voltage_battery;
                asset_status_List[1].velX_mps = uavStatus2.uavPos.vx;
                asset_status_List[1].velY_mps = uavStatus2.uavPos.vy;
                asset_status_List[1].velZ_mps = uavStatus2.uavPos.vz;
                asset_status_List[1].radar_report.rcs_dBsm = rcs_dBsm;

                if (logReceivedDataCheckBox.Checked)
                {
                    double currentTimeSub_sec = (DateTime.UtcNow -
                        Ref1970_Utc).TotalMilliseconds / 1000.0;
                    bwAir.Write(term);
                    bwAir.Write(System.Convert.ToInt32(uav2Association));
                    bwAir.Write(currentTimeSub_sec);
                    bwAir.Write(uavStatus2.uavPos.time_boot_sec);
                    bwAir.Write(uavStatus2.uavPos.lat_d);
                    bwAir.Write(uavStatus2.uavPos.lon_d);
                    bwAir.Write(uavStatus2.uavPos.alt_m);
                    bwAir.Write(uavStatus2.uavPos.relative_alt);
                    bwAir.Write(uavStatus2.uavPos.vx);
                    bwAir.Write(uavStatus2.uavPos.vy);
                    bwAir.Write(uavStatus2.uavPos.vz);
                    bwAir.Write(uavStatus2.uavPos.hdg);
                    bwAir.Write(term);
                }
            }

            if (uavStatus3 != null && uav3Association != "-")
            {
                asset_status_List[2].unix_epoch_time_sec = uavStatus3.uavPos.time_boot_sec - uavStatus3.uavSysTime.time_boot_sec
                    + uavStatus3.uavSysTime.time_unix_sec + uavStatus3.uavSysTime.del_mav_cpu_sec + gpsTimeOffset;
                asset_status_List[2].uavAssociation = uav3Number;
                asset_status_List[2].latitude_deg = uavStatus3.uavPos.lat_d;
                asset_status_List[2].longitude_deg = uavStatus3.uavPos.lon_d;
                asset_status_List[2].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(asset_status_List[2].latitude_deg, asset_status_List[2].longitude_deg);
                asset_status_List[2].altitude_m = uavStatus3.uavPos.alt_m;
                asset_status_List[2].heading = uavStatus3.uavPos.hdg;
                asset_status_List[2].battery_voltage = uavStatus3.uavSysStatus.voltage_battery;
                asset_status_List[2].velX_mps = uavStatus3.uavPos.vx;
                asset_status_List[2].velY_mps = uavStatus3.uavPos.vy;
                asset_status_List[2].velZ_mps = uavStatus3.uavPos.vz;
                asset_status_List[2].radar_report.rcs_dBsm = rcs_dBsm;

                if (logReceivedDataCheckBox.Checked)
                {
                    double currentTimeSub_sec = (DateTime.UtcNow -
                        Ref1970_Utc).TotalMilliseconds / 1000.0;
                    bwAir.Write(term);
                    bwAir.Write(System.Convert.ToInt32(uav3Association));
                    bwAir.Write(currentTimeSub_sec);
                    bwAir.Write(uavStatus3.uavPos.time_boot_sec);
                    bwAir.Write(uavStatus3.uavPos.lat_d);
                    bwAir.Write(uavStatus3.uavPos.lon_d);
                    bwAir.Write(uavStatus3.uavPos.alt_m);
                    bwAir.Write(uavStatus3.uavPos.relative_alt);
                    bwAir.Write(uavStatus3.uavPos.vx);
                    bwAir.Write(uavStatus3.uavPos.vy);
                    bwAir.Write(uavStatus3.uavPos.vz);
                    bwAir.Write(uavStatus3.uavPos.hdg);
                    bwAir.Write(term);
                }
            }

            if (uavStatus4 != null && uav4Association != "-")
            {
                asset_status_List[3].unix_epoch_time_sec = uavStatus4.uavPos.time_boot_sec - uavStatus4.uavSysTime.time_boot_sec
                    + uavStatus4.uavSysTime.time_unix_sec + uavStatus4.uavSysTime.del_mav_cpu_sec + gpsTimeOffset;
                asset_status_List[3].uavAssociation = uav4Number;
                asset_status_List[3].latitude_deg = uavStatus4.uavPos.lat_d;
                asset_status_List[3].longitude_deg = uavStatus4.uavPos.lon_d;
                asset_status_List[3].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(asset_status_List[3].latitude_deg, asset_status_List[3].longitude_deg);
                asset_status_List[3].altitude_m = uavStatus4.uavPos.alt_m;
                asset_status_List[3].heading = uavStatus4.uavPos.hdg;
                asset_status_List[3].battery_voltage = uavStatus4.uavSysStatus.voltage_battery;
                asset_status_List[3].velX_mps = uavStatus4.uavPos.vx;
                asset_status_List[3].velY_mps = uavStatus4.uavPos.vy;
                asset_status_List[3].velZ_mps = uavStatus4.uavPos.vz;
                asset_status_List[3].radar_report.rcs_dBsm = rcs_dBsm;

                if (logReceivedDataCheckBox.Checked)
                {
                    double currentTimeSub_sec = (DateTime.UtcNow -
                        Ref1970_Utc).TotalMilliseconds / 1000.0;
                    bwAir.Write(term);
                    bwAir.Write(System.Convert.ToInt32(uav4Association));
                    bwAir.Write(currentTimeSub_sec);
                    bwAir.Write(uavStatus4.uavPos.time_boot_sec);
                    bwAir.Write(uavStatus4.uavPos.lat_d);
                    bwAir.Write(uavStatus4.uavPos.lon_d);
                    bwAir.Write(uavStatus4.uavPos.alt_m);
                    bwAir.Write(uavStatus4.uavPos.relative_alt);
                    bwAir.Write(uavStatus4.uavPos.vx);
                    bwAir.Write(uavStatus4.uavPos.vy);
                    bwAir.Write(uavStatus4.uavPos.vz);
                    bwAir.Write(uavStatus4.uavPos.hdg);
                    bwAir.Write(term);
                }

            }
            // send target positions to users over UDP
            if (sendUavTruthMsgCheckbox.Checked)
            {
                byte[] msgArray = outgoingMessage.packTrueUavLocationMsg(asset_status_List,isUdpMessageFormatBigEndian);
                if(msgArray.Length>(3*sizeof(UInt16)))  // only send if target info is available)
                    udpClient.Send(msgArray, trueUavMsgEndpoint);
            }
        }

        // updateStatus updates the GUI UAV status table
        private void updateStatus()   
        {
            {
                asset_1_Lat.Text = asset_status_List[0].latitude_deg.ToString("f6");
                asset_1_Lon.Text = asset_status_List[0].longitude_deg.ToString("f6");
                asset_1_Alt.Text = asset_status_List[0].altitude_m.ToString("f1");
                asset_1_Hdg.Text = asset_status_List[0].heading.ToString("f1");
                asset_1_Pow.Text = asset_status_List[0].battery_voltage.ToString("f1");
            }
            {
                asset_2_Lat.Text = asset_status_List[1].latitude_deg.ToString("f6");
                asset_2_Lon.Text = asset_status_List[1].longitude_deg.ToString("f6");
                asset_2_Alt.Text = asset_status_List[1].altitude_m.ToString("f1");
                asset_2_Hdg.Text = asset_status_List[1].heading.ToString("f1");
                asset_2_Pow.Text = asset_status_List[1].battery_voltage.ToString("f1");
            }
            {
                asset_3_Lat.Text = asset_status_List[2].latitude_deg.ToString("f6");
                asset_3_Lon.Text = asset_status_List[2].longitude_deg.ToString("f6");
                asset_3_Alt.Text = asset_status_List[2].altitude_m.ToString("f1");
                asset_3_Hdg.Text = asset_status_List[2].heading.ToString("f1");
                asset_3_Pow.Text = asset_status_List[2].battery_voltage.ToString("f1");
            }
            {
                asset_4_Lat.Text = asset_status_List[3].latitude_deg.ToString("f6");
                asset_4_Lon.Text = asset_status_List[3].longitude_deg.ToString("f6");
                asset_4_Alt.Text = asset_status_List[3].altitude_m.ToString("f1");
                asset_4_Hdg.Text = asset_status_List[3].heading.ToString("f1");
                asset_4_Pow.Text = asset_status_List[3].battery_voltage.ToString("f1");
            }

        }

        private void Form1_Load(object sender, EventArgs e)
        {
            if (simulateTracks)
                initializeSimulatedTargets();

            // Load geo-referenced map
            Image img = AreaImage.Image;
            try
            {
                string mapFileName = ConfigurationManager.AppSettings["MissionMapFileName"];
                img = Image.FromFile(Directory.GetCurrentDirectory() + "/" + mapFileName);
            }
            finally
            {


            }

            if (MapImage != null)
                MapImage.Dispose();
            MapImage = (Image)img.Clone();
            img.Dispose();
            AreaImage.Image = MapImage;

            AreaImage.Paint += new System.Windows.Forms.PaintEventHandler(this.AreaImage_Paint);
            delLon = (SElon - NWlon) / (AreaImage.Size.Width);
            delLat = (NWlat - SElat) / (AreaImage.Size.Height);

            this.Size = new System.Drawing.Size(1484, 1089);
            this.Location = new Point(0, 0);

        }

        // convert latitude and longitude to GUI map pixel coordiantes
        public Point MapPosition(double latitude, double longitude)
        {
            Point MapCoord = new Point();
            MapCoord.X = (int)((longitude - NWlon) / delLon);
            MapCoord.Y = (int)((NWlat - latitude) / delLat);   //Measured down from NW corner
            return(MapCoord);
        }

        // Update GUI map display with current status of UAVs with history trail, and pod health and detections
        private void AreaImage_Paint(object sender, System.Windows.Forms.PaintEventArgs e)
        {
            Graphics gtemp = e.Graphics;
            Point temp = new Point();
            Point[] tempArray;
 
            // plot radar location
            int corner_x, corner_y;
            int xbox = 10;
            int ybox = 20;
            corner_x = (int)((RadarParameter.longitude_deg - NWlon) / (SElon - NWlon) * (AreaImage.Size.Width) - xbox / 2);
            corner_y = (int)((RadarParameter.latitude_deg - NWlat) / (SElat - NWlat) * (AreaImage.Size.Height) - ybox / 2);
            Rectangle tempRect = new Rectangle(corner_x, corner_y, xbox, ybox);
            gtemp.DrawRectangle(new Pen(Color.Yellow, 3), tempRect);

            // plot acoustic pod locations
            Point[] triangle = new Point[4];
            Point[] baseTriangle = new Point[3];
            baseTriangle[0].X = 0;
            baseTriangle[0].Y = 10;
            baseTriangle[1].X = 7;
            baseTriangle[1].Y = 0;
            baseTriangle[2].X = -7;
            baseTriangle[2].Y = 0;

            xbox = 6;
            ybox = 6;
            int podRadius_x = (int)(RadarParameter.podRadius_m * deg2meterE / delLon);
            int podRadius_y = (int)(RadarParameter.podRadius_m * deg2meterN / delLat);

            foreach(var pod in podAssets)
            {
                // plot pods
                corner_x = (int)((pod.Value.pos_long_deg - NWlon) / (SElon - NWlon) * (AreaImage.Size.Width) - xbox / 2);
                corner_y = (int)((pod.Value.pos_lat_deg - NWlat) / (SElat - NWlat) * (AreaImage.Size.Height) - ybox / 2);
                // verify pod is in image
                if (corner_x >= 0 & corner_x <= AreaImage.Size.Width & corner_y >= 0 & corner_y <= AreaImage.Size.Height)
                {
                    tempRect = new Rectangle(corner_x, corner_y, xbox, ybox);
                    if (pod.Value.podHealthy)
                        gtemp.DrawRectangle(new Pen(displayColors.idleEllipse, 3), tempRect);
                    else
                        gtemp.DrawRectangle(new Pen(displayColors.noSignal, 3), tempRect);

                    // plot ellipse
                    corner_x = (int)((pod.Value.pos_long_deg - NWlon) / (SElon - NWlon) * (AreaImage.Size.Width) - podRadius_x);
                    corner_y = (int)((pod.Value.pos_lat_deg - NWlat) / (SElat - NWlat) * (AreaImage.Size.Height) - podRadius_y);
                    tempRect = new Rectangle(corner_x, corner_y, 2 * podRadius_x, 2 * podRadius_y);
                    if (currColorCycle > 0)
                    {
                        gtemp.DrawEllipse(new Pen(pod.Value.targetColorsSolo, 3), tempRect);
                        // also change pod name in pod table to know which pod is making detection
                        switch (pod.Key)
                        {
                            case 1:
                                {
                                    pod1Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 2:
                                {
                                    pod2Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 3:
                                {
                                    pod3Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 4:
                                {
                                    pod4Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 5:
                                {
                                    pod5Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 6:
                                {
                                    pod6Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 7:
                                {
                                    pod7Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 8:
                                {
                                    pod8Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                            case 9:
                                {
                                    pod9Label.ForeColor = pod.Value.textColorsSolo;
                                    break;
                                }
                        }
                    }
                    else
                    {
                        gtemp.DrawEllipse(new Pen(pod.Value.targetColorsRacer, 3), tempRect);
                        // also change color in pod table to know which pods are making detection
                        switch (pod.Key)
                        {
                            case 1:
                                {
                                    pod1Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 2:
                                {
                                    pod2Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 3:
                                {
                                    pod3Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 4:
                                {
                                    pod4Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 5:
                                {
                                    pod5Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 6:
                                {
                                    pod6Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 7:
                                {
                                    pod7Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 8:
                                {
                                    pod8Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                            case 9:
                                {
                                    pod9Label.ForeColor = pod.Value.textColorsRacer;
                                    break;
                                }
                        }
                    }
                }
            }
            currColorCycle *= -1;  // change to other color to provide flashing of target detections

            // Display AoA possible location if three angles available
            // assume three pods have direction capability
            Dictionary<int, aoa_base_data> PodAoa = new Dictionary<int, aoa_base_data>();
            int PodAoaCnt = 0;
            foreach (var pod in podAssets)
            {
                // Add info for direction finding (only for Solo  currently)
                if (pod.Value.targetIdSolo)  // good Solo target
                {
                    if ((int)(pod.Value.soloSensorEstimates.AoA_deg) <990)  // pod provides AoA
                    {
                        aoa_base_data tempPod = new aoa_base_data();
                        tempPod.podId = pod.Value.podId;
                        tempPod.pod_pos_lat_deg = pod.Value.pos_lat_deg;
                        tempPod.pod_pos_lon_deg = pod.Value.pos_long_deg;
                        tempPod.pod_utmCoord = pod.Value.utmCoord;
                        tempPod.pod_pos_alt_mMSL = pod.Value.pos_elev_m;
                        tempPod.pod_aoa = pod.Value.soloSensorEstimates.AoA_deg;
                        PodAoa.Add(PodAoaCnt, tempPod);
                        PodAoaCnt++;
                    }
                }
            }
            
            if (PodAoa.Count == 3)
            {
                // clear old estimates
                aoaTargetEstimate.lat_deg = 0;
                aoaTargetEstimate.long_deg = 0;
                aoaTargetEstimate.elev_m = 0;

                // update target estimate
                generateAoAIntercepts(PodAoa);

                Point P1, P2, P3, target;
                Color targetLineColor = Color.BlueViolet;
                P1 = MapPosition(PodAoa[0].pod_pos_lat_deg, PodAoa[0].pod_pos_lon_deg);
                P2 = MapPosition(PodAoa[1].pod_pos_lat_deg, PodAoa[1].pod_pos_lon_deg);
                P3 = MapPosition(PodAoa[2].pod_pos_lat_deg, PodAoa[2].pod_pos_lon_deg);

                if (aoaTargetEstimate.lat_deg != 0.0 && aoaTargetEstimate.long_deg != 0.0 && aoaTargetEstimate.elev_m != 0.0)
                {
                    target = MapPosition(aoaTargetEstimate.lat_deg, aoaTargetEstimate.long_deg);

                    gtemp.DrawLine(new Pen(targetLineColor, 3), P1, target);
                    gtemp.DrawLine(new Pen(targetLineColor, 3), P2, target);
                    gtemp.DrawLine(new Pen(targetLineColor, 3), P3, target);
                }

            }
            


            // Plot UAV tracks
            Rectangle uavRect;
            int iconWid = 6;
            for (int kk=0; kk<4; kk++)
            {
                if(asset_status_List[kk].latitude_deg!=0.0)
                {
                    if (asset_status_List[kk].EnableTrack)
                    {
                        {
                            temp = MapPosition(asset_status_List[kk].latitude_deg,
                                asset_status_List[kk].longitude_deg);
                            asset_status_List[kk].trackQueue.Enqueue(temp);
                            if (asset_status_List[kk].trackQueue.Count > maxQueueSize)
                                asset_status_List[kk].trackQueue.Dequeue();     //Remove lowest in queue
                            if (asset_status_List[kk].trackQueue.Count > 2)
                            {
                                tempArray = new Point[asset_status_List[kk].trackQueue.Count];
                                asset_status_List[kk].trackQueue.CopyTo(tempArray, 0);

                                gtemp.DrawLines(new Pen(asset_status_List[kk].ellipse.color,
                                    asset_status_List[kk].ellipse.PenWidth), tempArray);
                                uavRect = new Rectangle(temp.X - iconWid/2, temp.Y - iconWid/2, iconWid, iconWid);
                                gtemp.DrawEllipse(new Pen(asset_status_List[kk].ellipse.color,
                                    asset_status_List[kk].ellipse.PenWidth), uavRect);
                            }
                        }
                    }

                }
            }
        }

        // This method determines the intersection of three pod angle of arrival angles
        private void generateAoAIntercepts(Dictionary<int, aoa_base_data> PodAoa)
        {

            try
            {
                {
                    double Px1 = PodAoa[0].pod_utmCoord.UTMEasting;
                    double Py1 = PodAoa[0].pod_utmCoord.UTMNorthing;
                    double Pz1 = PodAoa[0].pod_pos_alt_mMSL;
                    double AoA1 = PodAoa[0].pod_aoa;
                    if (AoA1 == 90.0)
                        AoA1 *= 0.999; // add offset if Aoa=90 deg
                    AoA1 = AoA1 / 180.0 * Math.PI;

                    double Px2 = PodAoa[1].pod_utmCoord.UTMEasting;
                    double Py2 = PodAoa[1].pod_utmCoord.UTMNorthing;
                    double Pz2 = PodAoa[1].pod_pos_alt_mMSL;
                    double AoA2 = PodAoa[1].pod_aoa;
                    if (AoA2 == 90.0)
                        AoA2 *= 0.999; // add offset if Aoa=90 deg
                    AoA2 = AoA2 / 180.0 * Math.PI;

                    double Px3 = PodAoa[2].pod_utmCoord.UTMEasting;
                    double Py3 = PodAoa[2].pod_utmCoord.UTMNorthing;
                    double Pz3 = PodAoa[2].pod_pos_alt_mMSL;
                    double AoA3 = PodAoa[2].pod_aoa;
                    if (AoA3 == 90.0)
                        AoA3 *= 0.999; // add offset if Aoa=90 deg
                    AoA3 = AoA3 / 180.0 * Math.PI;

                    utmCoord_t baseZone = PodAoa[0].pod_utmCoord;

                    // ensure that intersection of cones is broadly possible (!!! This is dependent on array pointing north, need to generalize if other angles are allowed)
                    double rightAng_rad = Math.PI / 2.0;
                    if (!((Py1 > Py2 && AoA1 < rightAng_rad && AoA2 > rightAng_rad) ||
                        (Py1 < Py2 && AoA2 < rightAng_rad && AoA1 > rightAng_rad) ||
                        (Py1 > Py3 && AoA1 < rightAng_rad && AoA3 > rightAng_rad) ||
                        (Py1 < Py3 && AoA3 < rightAng_rad && AoA1 > rightAng_rad)))
                    {
                        // use relative numbers since smaller to work with
                        double[] baseLoc = new double[3] { Px1, Py1, Pz1 };
                        Px1 -= baseLoc[0];
                        Py1 -= baseLoc[1];
                        Pz1 -= baseLoc[2];
                        Px2 -= baseLoc[0];
                        Py2 -= baseLoc[1];
                        Pz2 -= baseLoc[2];
                        Px3 -= baseLoc[0];
                        Py3 -= baseLoc[1];
                        Pz3 -= baseLoc[2];

                        double SA1 = Math.Sin(AoA1);
                        double CA1 = Math.Cos(AoA1);
                        double TA1 = Math.Tan(AoA1);
                        double SA2 = Math.Sin(AoA2);
                        double CA2 = Math.Cos(AoA2);
                        double TA2 = Math.Tan(AoA2);
                        double SA3 = Math.Sin(AoA3);
                        double CA3 = Math.Cos(AoA3);
                        double TA3 = Math.Tan(AoA3);

                        int PSIsz = 1000;
                        double[,] ellipse12plus = new double[PSIsz, 3];
                        double[,] ellipse13plus = new double[PSIsz, 3];
                        double[,] ellipse12minus = new double[PSIsz, 3];
                        double[,] ellipse13minus = new double[PSIsz, 3];
                        double[,] ellipse12plusPart = new double[PSIsz, 3];
                        double[,] ellipse13plusPart = new double[PSIsz, 3];
                        double[,] ellipse12minusPart = new double[PSIsz, 3];
                        double[,] ellipse13minusPart = new double[PSIsz, 3];
                        double[,] ellipse12 = new double[1, 1]; // initialize to single zero for testing later
                        double[,] ellipse13 = new double[1, 1];

                        double PSI;
                        double SPP;
                        double CPP;
                        double R1_plus, R1_minus;
                        double min_R2_plus = 0.0;
                        double min_R2_minus = 0.0;
                        double sum_ellPlus_z = 0.0; 
                        double sum_ellMinus_z = 0.0;
                        double b, d;
                        double[] possibleSol = new double[3];

                        bool trueEllipse12 = true;  // only true if all points in ellipse are real
                        bool trueEllipse13 = true;
                        int[] validIndex12plus = new int[PSIsz];  // index of valid (real) ellipse values
                        int[] validIndex12minus = new int[PSIsz];
                        int[] validIndex13plus = new int[PSIsz];
                        int[] validIndex13minus = new int[PSIsz];
                        int validCntr12plus = 0;  // counter of how many valid (real) values in an ellipse
                        int validCntr12minus = 0;
                        int validCntr13plus = 0;
                        int validCntr13minus = 0;
                        bool goodIndexPlus;  // used to flag valid indexes each loop
                        bool goodIndexMinus;

                        double eps = 1e-14;  // used to check for zero with doubles
                        double a12 = SA1 * SA1 - TA2 * TA2 * CA1 * CA1;
                        double c12 = (Px1 - Px2) * (Px1 - Px2) - (Py1 - Py2) * (Py1 - Py2) * TA2 *TA2 + (Pz1 - Pz2) * (Pz1 - Pz2);
                        double a13 = SA1 * SA1 - TA3 * TA3 * CA1 * CA1;
                        double c13 = (Px1 - Px3) * (Px1 - Px3) - (Py1 - Py3) * (Py1 - Py3) * TA3 * TA3 + (Pz1 - Pz3) * (Pz1 - Pz3);

                        bool chooseEllPlus;
                        bool chooseEllMinus;
                        double dist_1;
                        double dist_2;
                        double dist_endminus1;
                        double dist_end;
                        bool specialCase;

                        // develop plus and minus ellipses of real values
                        // ellipse 12
                        if (Math.Abs(a12) < eps) // a12 = 0 so target is equal distance between cones
                            specialCase = true;
                        else
                            specialCase = false;

                        for (int i = 0; i < PSIsz; i++)
                        {

                            if (specialCase)   
                            {
                                // choose side of cone for intersect
                                if(Px2 >= Px1)
                                    PSI = (double)(i + 1) * 1.0 / (double)PSIsz * Math.PI / 2.0; 
                                else
                                    PSI = Math.PI / 2.0 + (double)(i + 1) * 1.0 / (double)PSIsz * Math.PI / 2.0; 
                                SPP = Math.Sin(PSI);
                                CPP = Math.Cos(PSI);
                                b = 2 * ((Px1 - Px2) * SA1 * CPP - (Py1 - Py2) * CA1 * TA2 * TA2 + (Pz1 - Pz2) * SA1 * SPP);
                                R1_plus = -c12 / b;

                                ellipse12[i, 0] = Px1 + SA1 * R1_plus * CPP;
                                ellipse12[i, 1] = Py1 + CA1 * R1_plus;
                                ellipse12[i, 2] = Pz1 + SA1 * R1_plus * SPP;
                            }
                            else  // all other cases
                            {
                                PSI = (double)(i + 1) * 1.0 / (double)PSIsz * Math.PI; 
                                SPP = Math.Sin(PSI);
                                CPP = Math.Cos(PSI);

                                b = 2 * ((Px1 - Px2) * SA1 * CPP - (Py1 - Py2) * CA1 * TA2 * TA2 + (Pz1 - Pz2) * SA1 * SPP);
                                d = b * b - 4 * a12 * c12;
                                if (d >= 0)
                                {
                                    R1_plus = (-b + Math.Sqrt(d)) / 2.0 / a12;
                                    R1_minus = (-b - Math.Sqrt(d)) / 2.0 / a12;
                                    if (i == 0)
                                    {
                                        min_R2_plus = ((Py1 - Py2) + R1_plus * Math.Cos(AoA1)) / Math.Cos(AoA2);
                                        min_R2_minus = ((Py1 - Py2) + R1_minus * Math.Cos(AoA1)) / Math.Cos(AoA2);
                                    }
                                    else
                                    {
                                        min_R2_plus = Math.Min(min_R2_plus, ((Py1 - Py2) + R1_plus * Math.Cos(AoA1)) / Math.Cos(AoA2));
                                        min_R2_minus = Math.Min(min_R2_minus, ((Py1 - Py2) + R1_minus * Math.Cos(AoA1)) / Math.Cos(AoA2));
                                    }

                                    ellipse12plus[i, 0] = Px1 + SA1 * R1_plus * CPP;
                                    ellipse12plus[i, 1] = Py1 + CA1 * R1_plus;
                                    ellipse12plus[i, 2] = Pz1 + SA1 * R1_plus * SPP;
                                    ellipse12minus[i, 0] = Px1 + SA1 * R1_minus * CPP;
                                    ellipse12minus[i, 1] = Py1 + CA1 * R1_minus;
                                    ellipse12minus[i, 2] = Pz1 + SA1 * R1_minus * SPP;
                                    sum_ellPlus_z += ellipse12plus[i, 2];
                                    sum_ellMinus_z += ellipse12minus[i, 2];

                                    // track partial ellipses for combining separately
                                    ellipse12plusPart[i, 0] = ellipse12plus[i, 0];
                                    ellipse12plusPart[i, 1] = ellipse12plus[i, 1];
                                    ellipse12plusPart[i, 2] = ellipse12plus[i, 2];
                                    ellipse12minusPart[i, 0] = ellipse12minus[i, 0];
                                    ellipse12minusPart[i, 1] = ellipse12minus[i, 1];
                                    ellipse12minusPart[i, 2] = ellipse12minus[i, 2];

                                    goodIndexPlus = true;
                                    goodIndexMinus = true;
                                    // remove bad ellipse points for partial ellipse
                                    if(R1_plus < 0 || ellipse12plusPart[i, 2] < Math.Min(Pz1,Pz2))
                                    {
                                        ellipse12plusPart[i, 0] = 0.0;
                                        ellipse12plusPart[i, 1] = 0.0;
                                        ellipse12plusPart[i, 2] = 0.0;
                                        goodIndexPlus = false;
                                    }
                                    if (R1_minus < 0 || ellipse12plusPart[i, 2] < Math.Min(Pz1, Pz2))
                                    {
                                        ellipse12minusPart[i, 0] = 0.0;
                                        ellipse12minusPart[i, 1] = 0.0;
                                        ellipse12minusPart[i, 2] = 0.0;
                                        goodIndexMinus = false;
                                    }
                                    if (goodIndexPlus)
                                    {
                                        validIndex12plus[validCntr12plus] = i;
                                        validCntr12plus++;
                                    }
                                    if (goodIndexMinus)
                                    {
                                        validIndex12minus[validCntr12minus] = i;
                                        validCntr12minus++;
                                    }
                                }
                                else
                                {
                                    ellipse12plusPart[i, 0] = 0.0;
                                    ellipse12plusPart[i, 1] = 0.0;
                                    ellipse12plusPart[i, 2] = 0.0;
                                    ellipse12minusPart[i, 0] = 0.0;
                                    ellipse12minusPart[i, 1] = 0.0;
                                    ellipse12minusPart[i, 2] = 0.0;
                                    trueEllipse12 = false;  // negative values to need to construct ellipse with plus and minus real components
                                }
                            }
                        }
                        // After plus and minus ellipses of real numbers are determined, two alternatives are addressed. If the entire ellipse has real values,
                        // 2 solutions are possible, using either the plus ellipse or minus, but not both or combining plus and minus ellipses.  With two full ellipses,
                        // different rules are used based on AoA and relative y positions to select either the plus or minus ellipse to use.  For partial plus and minus ellipses, 
                        // the real portions of both plus and minus ellipses together define the ellipse.  Rules are used to combine the two ellipses into the correct order.
                        if (!specialCase)
                        {
                            chooseEllPlus = false;
                            chooseEllMinus = false;

                            if (trueEllipse12)  // need to determine if EllipseA or EllipseB is used
                            {
                                if (min_R2_plus > 0 && min_R2_minus < 0)
                                    chooseEllPlus = true;
                                else if (min_R2_plus < 0 && min_R2_minus > 0)
                                    chooseEllMinus = true;
                                else if ((sum_ellPlus_z / (double)PSIsz) > Pz1 && (sum_ellMinus_z / (double)PSIsz) < Pz1)
                                    chooseEllPlus = true;
                                else if ((sum_ellPlus_z / (double)PSIsz) < Pz1 && (sum_ellMinus_z / (double)PSIsz) > Pz1)
                                    chooseEllMinus = true;
                                else if ((sum_ellPlus_z / (double)PSIsz) > (sum_ellMinus_z / (double)PSIsz))
                                    chooseEllPlus = true;
                                else
                                    chooseEllMinus = true;

                                if (chooseEllPlus || chooseEllMinus)
                                    ellipse12 = new double[PSIsz, 3];

                                for (int i = 0; i < PSIsz; i++)
                                {
                                    if (chooseEllPlus)
                                    {
                                        ellipse12[i, 0] = ellipse12plus[i, 0];
                                        ellipse12[i, 1] = ellipse12plus[i, 1];
                                        ellipse12[i, 2] = ellipse12plus[i, 2];
                                    }
                                    if (chooseEllMinus)
                                    {
                                        ellipse12[i, 0] = ellipse12minus[i, 0];
                                        ellipse12[i, 1] = ellipse12minus[i, 1];
                                        ellipse12[i, 2] = ellipse12minus[i, 2];
                                    }
                                }
                            }
                            else // build ellipse from plus and minus real values
                            {
                                // knowns: ellipse12plus, ellipse12minus
                                //         validIndex12plus,validIndex12minus
                                //         validCntr12plus,validCntr12minus
                                if (validCntr12plus > 1 && validCntr12minus > 1)
                                {
                                    ellipse12 = new double[validCntr12plus + validCntr12minus, 3];
                                    dist_1 = Math.Sqrt((ellipse12plusPart[validIndex12plus[0], 0] - ellipse12minusPart[validIndex12minus[0], 0]) *
                                        (ellipse12plusPart[validIndex12plus[0], 0] - ellipse12minusPart[validIndex12minus[0], 0]) +
                                        (ellipse12plusPart[validIndex12plus[0], 1] - ellipse12minusPart[validIndex12minus[0], 1]) *
                                        (ellipse12plusPart[validIndex12plus[0], 1] - ellipse12minusPart[validIndex12minus[0], 1]) +
                                        (ellipse12plusPart[validIndex12plus[0], 2] - ellipse12minusPart[validIndex12minus[0], 2]) *
                                        (ellipse12plusPart[validIndex12plus[0], 2] - ellipse12minusPart[validIndex12minus[0], 2]));
                                    dist_2 = Math.Sqrt((ellipse12plusPart[validIndex12plus[1], 0] - ellipse12minusPart[validIndex12minus[1], 0]) *
                                        (ellipse12plusPart[validIndex12plus[1], 0] - ellipse12minusPart[validIndex12minus[1], 0]) +
                                        (ellipse12plusPart[validIndex12plus[1], 1] - ellipse12minusPart[validIndex12minus[1], 1]) *
                                        (ellipse12plusPart[validIndex12plus[1], 1] - ellipse12minusPart[validIndex12minus[1], 1]) +
                                        (ellipse12plusPart[validIndex12plus[1], 2] - ellipse12minusPart[validIndex12minus[1], 2]) *
                                        (ellipse12plusPart[validIndex12plus[1], 2] - ellipse12minusPart[validIndex12minus[1], 2]));
                                    dist_end = Math.Sqrt((ellipse12plusPart[validIndex12plus[validCntr12plus - 1], 0] - ellipse12minusPart[validIndex12minus[validCntr12minus - 1], 0]) *
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 1], 0] - ellipse12minusPart[validIndex12minus[validCntr12minus - 1], 0]) +
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 1], 1] - ellipse12minusPart[validIndex12minus[validCntr12minus - 1], 1]) *
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 1], 1] - ellipse12minusPart[validIndex12minus[validCntr12minus - 1], 1]) +
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 1], 2] - ellipse12minusPart[validIndex12minus[validCntr12minus - 1], 2]) *
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 1], 2] - ellipse12minusPart[validIndex12minus[validCntr12minus - 1], 2]));
                                    dist_endminus1 = Math.Sqrt((ellipse12plusPart[validIndex12plus[validCntr12plus - 2], 0] - ellipse12minusPart[validIndex12minus[validCntr12minus - 2], 0]) *
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 2], 0] - ellipse12minusPart[validIndex12minus[validCntr12minus - 2], 0]) +
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 2], 1] - ellipse12minusPart[validIndex12minus[validCntr12minus - 2], 1]) *
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 2], 1] - ellipse12minusPart[validIndex12minus[validCntr12minus - 2], 1]) +
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 2], 2] - ellipse12minusPart[validIndex12minus[validCntr12minus - 2], 2]) *
                                        (ellipse12plusPart[validIndex12plus[validCntr12plus - 2], 2] - ellipse12minusPart[validIndex12minus[validCntr12minus - 2], 2]));
                                    if (Math.Min(dist_1,dist_endminus1) < Math.Min(dist_2,dist_end))
                                    {
                                        for (int i = 0; i < validCntr12plus; i++)  //first plus ellipse reversed
                                        {
                                            ellipse12[i, 0] = ellipse12plusPart[validIndex12plus[validCntr12plus - 1 - i], 0];  //reverse order plus ellipse
                                            ellipse12[i, 1] = ellipse12plusPart[validIndex12plus[validCntr12plus - 1 - i], 1];
                                            ellipse12[i, 2] = ellipse12plusPart[validIndex12plus[validCntr12plus - 1 - i], 2];
                                        }
                                        for (int i = validCntr12plus; i < validCntr12plus + validCntr12minus; i++)  //then minus ellipse
                                        {
                                            ellipse12[i, 0] = ellipse12minusPart[validIndex12minus[i - validCntr12plus], 0];
                                            ellipse12[i, 1] = ellipse12minusPart[validIndex12minus[i - validCntr12plus], 1];
                                            ellipse12[i, 2] = ellipse12minusPart[validIndex12minus[i - validCntr12plus], 2];
                                        }
                                    }
                                    else
                                    {
                                        for (int i = 0; i < validCntr12minus; i++)  //first minus ellipse reversed
                                        {
                                            ellipse12[i, 0] = ellipse12minusPart[validIndex12minus[i], 0];
                                            ellipse12[i, 1] = ellipse12minusPart[validIndex12minus[i], 1];
                                            ellipse12[i, 2] = ellipse12minusPart[validIndex12minus[i], 2];
                                        }
                                        for (int i = validCntr12minus; i < validCntr12minus + validCntr12plus; i++)  //then plus ellipse reversed
                                        {
                                            ellipse12[i, 0] = ellipse12plusPart[validIndex12plus[validCntr12minus + validCntr12plus - 1 - i], 0];  //reverse order plus ellipse
                                            ellipse12[i, 1] = ellipse12plusPart[validIndex12plus[validCntr12minus + validCntr12plus - 1 - i], 1];
                                            ellipse12[i, 2] = ellipse12plusPart[validIndex12plus[validCntr12minus + validCntr12plus - 1 - i], 2];
                                        }
                                    }
                                }
                            }
                        }


                        // ellipse 13
                        sum_ellPlus_z = 0.0;
                        sum_ellMinus_z = 0.0;

                        if (Math.Abs(a13) < eps) // a13 = 0 so target is equal distance between cones
                            specialCase = true;
                        else
                            specialCase = false;

                        for (int i = 0; i < PSIsz; i++)
                        {

                            if (specialCase)
                            {
                                // choose side of cone for intersect
                                if (Px3 >= Px1)
                                    PSI = (double)(i + 1) * 1.0 / (double)PSIsz * Math.PI / 2.0;
                                else
                                    PSI = Math.PI / 2.0 + (double)(i + 1) * 1.0 / (double)PSIsz * Math.PI / 2.0;
                                SPP = Math.Sin(PSI);
                                CPP = Math.Cos(PSI);
                                b = 2 * ((Px1 - Px3) * SA1 * CPP - (Py1 - Py3) * CA1 * TA3 * TA3 + (Pz1 - Pz3) * SA1 * SPP);
                                R1_plus = -c13 / b;

                                ellipse13[i, 0] = Px1 + SA1 * R1_plus * CPP;
                                ellipse13[i, 1] = Py1 + CA1 * R1_plus;
                                ellipse13[i, 2] = Pz1 + SA1 * R1_plus * SPP;
                            }
                            else  // all other cases
                            {
                                PSI = (double)(i + 1) * 1.0 / (double)PSIsz * Math.PI;
                                SPP = Math.Sin(PSI);
                                CPP = Math.Cos(PSI);

                                b = 2 * ((Px1 - Px3) * SA1 * CPP - (Py1 - Py3) * CA1 * TA3 * TA3 + (Pz1 - Pz3) * SA1 * SPP);
                                d = b * b - 4 * a13 * c13;
                                if (d >= 0)
                                {
                                    R1_plus = (-b + Math.Sqrt(d)) / 2.0 / a13;
                                    R1_minus = (-b - Math.Sqrt(d)) / 2.0 / a13;
                                    if (i == 0)
                                    {
                                        min_R2_plus = ((Py1 - Py3) + R1_plus * Math.Cos(AoA1)) / Math.Cos(AoA3);
                                        min_R2_minus = ((Py1 - Py3) + R1_minus * Math.Cos(AoA1)) / Math.Cos(AoA3);
                                    }
                                    else
                                    {
                                        min_R2_plus = Math.Min(min_R2_plus, ((Py1 - Py3) + R1_plus * Math.Cos(AoA1)) / Math.Cos(AoA3));
                                        min_R2_minus = Math.Min(min_R2_minus, ((Py1 - Py3) + R1_minus * Math.Cos(AoA1)) / Math.Cos(AoA3));
                                    }

                                    ellipse13plus[i, 0] = Px1 + SA1 * R1_plus * CPP;
                                    ellipse13plus[i, 1] = Py1 + CA1 * R1_plus;
                                    ellipse13plus[i, 2] = Pz1 + SA1 * R1_plus * SPP;
                                    ellipse13minus[i, 0] = Px1 + SA1 * R1_minus * CPP;
                                    ellipse13minus[i, 1] = Py1 + CA1 * R1_minus;
                                    ellipse13minus[i, 2] = Pz1 + SA1 * R1_minus * SPP;
                                    sum_ellPlus_z += ellipse13plus[i, 2];
                                    sum_ellMinus_z += ellipse13minus[i, 2];

                                    // track partial ellipses for combining separately
                                    ellipse13plusPart[i, 0] = ellipse13plus[i, 0];
                                    ellipse13plusPart[i, 1] = ellipse13plus[i, 1];
                                    ellipse13plusPart[i, 2] = ellipse13plus[i, 2];
                                    ellipse13minusPart[i, 0] = ellipse13minus[i, 0];
                                    ellipse13minusPart[i, 1] = ellipse13minus[i, 1];
                                    ellipse13minusPart[i, 2] = ellipse13minus[i, 2];

                                    goodIndexPlus = true;
                                    goodIndexMinus = true;
                                    // remove bad ellipse points for partial ellipse
                                    if (R1_plus < 0 || ellipse13plusPart[i, 2] < Math.Min(Pz1, Pz3))
                                    {
                                        ellipse13plusPart[i, 0] = 0.0;
                                        ellipse13plusPart[i, 1] = 0.0;
                                        ellipse13plusPart[i, 2] = 0.0;
                                        goodIndexPlus = false;
                                    }
                                    if (R1_minus < 0 || ellipse13plusPart[i, 2] < Math.Min(Pz1, Pz3))
                                    {
                                        ellipse13minusPart[i, 0] = 0.0;
                                        ellipse13minusPart[i, 1] = 0.0;
                                        ellipse13minusPart[i, 2] = 0.0;
                                        goodIndexMinus = false;
                                    }
                                    if (goodIndexPlus)
                                    {
                                        validIndex13plus[validCntr13plus] = i;
                                        validCntr13plus++;
                                    }
                                    if (goodIndexMinus)
                                    {
                                        validIndex13minus[validCntr13minus] = i;
                                        validCntr13minus++;
                                    }
                                }
                                else
                                {
                                    ellipse13plusPart[i, 0] = 0.0;
                                    ellipse13plusPart[i, 1] = 0.0;
                                    ellipse13plusPart[i, 2] = 0.0;
                                    ellipse13minusPart[i, 0] = 0.0;
                                    ellipse13minusPart[i, 1] = 0.0;
                                    ellipse13minusPart[i, 2] = 0.0;
                                    trueEllipse13 = false;  // negative values to need to construct ellipse with plus and minus real components
                                }
                            }
                        }
                        // After plus and minus ellipses of real numbers are determined, two alternatives are addressed. If the entire ellipse has real values,
                        // 2 solutions are possible, using either the plus ellipse or minus, but not both or combining plus and minus ellipses.  With two full ellipses,
                        // different rules are used based on AoA and relative y positions to select either the plus or minus ellipse to use.  For partial plus and minus ellipses, 
                        // the real portions of both plus and minus ellipses together define the ellipse.  Rules are used to combine the two ellipses into the correct order.
                        if (!specialCase)
                        {
                            chooseEllPlus = false;
                            chooseEllMinus = false;

                            if (trueEllipse13)  // need to determine if EllipseA or EllipseB is used
                            {

                                if (min_R2_plus > 0 && min_R2_minus < 0)
                                    chooseEllPlus = true;
                                else if (min_R2_plus < 0 && min_R2_minus > 0)
                                    chooseEllMinus = true;
                                else if ((sum_ellPlus_z / (double)PSIsz) > Pz1 && (sum_ellMinus_z / (double)PSIsz) < Pz1)
                                    chooseEllPlus = true;
                                else if ((sum_ellPlus_z / (double)PSIsz) < Pz1 && (sum_ellMinus_z / (double)PSIsz) > Pz1)
                                    chooseEllMinus = true;
                                else if ((sum_ellPlus_z / (double)PSIsz) > (sum_ellMinus_z / (double)PSIsz))
                                    chooseEllPlus = true;
                                else
                                    chooseEllMinus = true;

                                if (chooseEllPlus || chooseEllMinus)
                                    ellipse13 = new double[PSIsz, 3];

                                for (int i = 0; i < PSIsz; i++)
                                {
                                    if (chooseEllPlus)
                                    {
                                        ellipse13[i, 0] = ellipse13plus[i, 0];
                                        ellipse13[i, 1] = ellipse13plus[i, 1];
                                        ellipse13[i, 2] = ellipse13plus[i, 2];
                                    }
                                    if (chooseEllMinus)
                                    {
                                        ellipse13[i, 0] = ellipse13minus[i, 0];
                                        ellipse13[i, 1] = ellipse13minus[i, 1];
                                        ellipse13[i, 2] = ellipse13minus[i, 2];
                                    }
                                }
                            }
                            else // build ellipse from plus and minus real values
                            {
                                // knowns: ellipse13plus, ellipse13minus
                                //         validIndex13plus,validIndex13minus
                                //         validCntr13plus,validCntr13minus
                                if (validCntr13plus > 1 && validCntr13minus > 1)
                                {
                                    ellipse13 = new double[validCntr13plus + validCntr13minus, 3];
                                    dist_1 = Math.Sqrt((ellipse13plusPart[validIndex13plus[0], 0] - ellipse13minusPart[validIndex13minus[0], 0]) *
                                        (ellipse13plusPart[validIndex13plus[0], 0] - ellipse13minusPart[validIndex13minus[0], 0]) +
                                        (ellipse13plusPart[validIndex13plus[0], 1] - ellipse13minusPart[validIndex13minus[0], 1]) *
                                        (ellipse13plusPart[validIndex13plus[0], 1] - ellipse13minusPart[validIndex13minus[0], 1]) +
                                        (ellipse13plusPart[validIndex13plus[0], 2] - ellipse13minusPart[validIndex13minus[0], 2]) *
                                        (ellipse13plusPart[validIndex13plus[0], 2] - ellipse13minusPart[validIndex13minus[0], 2]));
                                    dist_2 = Math.Sqrt((ellipse13plusPart[validIndex13plus[1], 0] - ellipse13minusPart[validIndex13minus[1], 0]) *
                                        (ellipse13plusPart[validIndex13plus[1], 0] - ellipse13minusPart[validIndex13minus[1], 0]) +
                                        (ellipse13plusPart[validIndex13plus[1], 1] - ellipse13minusPart[validIndex13minus[1], 1]) *
                                        (ellipse13plusPart[validIndex13plus[1], 1] - ellipse13minusPart[validIndex13minus[1], 1]) +
                                        (ellipse13plusPart[validIndex13plus[1], 2] - ellipse13minusPart[validIndex13minus[1], 2]) *
                                        (ellipse13plusPart[validIndex13plus[1], 2] - ellipse13minusPart[validIndex13minus[1], 2]));
                                    dist_end = Math.Sqrt((ellipse13plusPart[validIndex13plus[validCntr13plus - 1], 0] - ellipse13minusPart[validIndex13minus[validCntr13minus - 1], 0]) *
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 1], 0] - ellipse13minusPart[validIndex13minus[validCntr13minus - 1], 0]) +
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 1], 1] - ellipse13minusPart[validIndex13minus[validCntr13minus - 1], 1]) *
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 1], 1] - ellipse13minusPart[validIndex13minus[validCntr13minus - 1], 1]) +
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 1], 2] - ellipse13minusPart[validIndex13minus[validCntr13minus - 1], 2]) *
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 1], 2] - ellipse13minusPart[validIndex13minus[validCntr13minus - 1], 2]));
                                    dist_endminus1 = Math.Sqrt((ellipse13plusPart[validIndex13plus[validCntr13plus - 2], 0] - ellipse13minusPart[validIndex13minus[validCntr13minus - 2], 0]) *
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 2], 0] - ellipse13minusPart[validIndex13minus[validCntr13minus - 2], 0]) +
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 2], 1] - ellipse13minusPart[validIndex13minus[validCntr13minus - 2], 1]) *
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 2], 1] - ellipse13minusPart[validIndex13minus[validCntr13minus - 2], 1]) +
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 2], 2] - ellipse13minusPart[validIndex13minus[validCntr13minus - 2], 2]) *
                                        (ellipse13plusPart[validIndex13plus[validCntr13plus - 2], 2] - ellipse13minusPart[validIndex13minus[validCntr13minus - 2], 2]));
                                    if (Math.Min(dist_1, dist_endminus1) < Math.Min(dist_2, dist_end))
                                    {
                                        for (int i = 0; i < validCntr13plus; i++)  //first plus ellipse reversed
                                        {
                                            ellipse13[i, 0] = ellipse13plusPart[validIndex13plus[validCntr13plus - 1 - i], 0];  //reverse order plus ellipse
                                            ellipse13[i, 1] = ellipse13plusPart[validIndex13plus[validCntr13plus - 1 - i], 1];
                                            ellipse13[i, 2] = ellipse13plusPart[validIndex13plus[validCntr13plus - 1 - i], 2];
                                        }
                                        for (int i = validCntr13plus; i < validCntr13plus + validCntr13minus; i++)  //then minus ellipse
                                        {
                                            ellipse13[i, 0] = ellipse13minusPart[validIndex13minus[i - validCntr13plus], 0];
                                            ellipse13[i, 1] = ellipse13minusPart[validIndex13minus[i - validCntr13plus], 1];
                                            ellipse13[i, 2] = ellipse13minusPart[validIndex13minus[i - validCntr13plus], 2];
                                        }
                                    }
                                    else
                                    {
                                        for (int i = 0; i < validCntr13minus; i++)  //first minus ellipse reversed
                                        {
                                            ellipse13[i, 0] = ellipse13minusPart[validIndex13minus[i], 0];
                                            ellipse13[i, 1] = ellipse13minusPart[validIndex13minus[i], 1];
                                            ellipse13[i, 2] = ellipse13minusPart[validIndex13minus[i], 2];
                                        }
                                        for (int i = validCntr13minus; i < validCntr13minus + validCntr13plus; i++)  //then plus ellipse reversed
                                        {
                                            ellipse13[i, 0] = ellipse13plusPart[validIndex13plus[validCntr13minus + validCntr13plus - 1 - i], 0];  //reverse order plus ellipse
                                            ellipse13[i, 1] = ellipse13plusPart[validIndex13plus[validCntr13minus + validCntr13plus - 1 - i], 1];
                                            ellipse13[i, 2] = ellipse13plusPart[validIndex13plus[validCntr13minus + validCntr13plus - 1 - i], 2];
                                        }
                                    }
                                }
                            }
                        }
                        // determine intersections by finding minimum distance between ellipses by comparing all combinations

                        if (ellipse12.GetLength(0) != 1 && ellipse13.GetLength(0) != 1)  // valid ellipses exist
                        {
                            double minValue = 1e6;
                            double dist;
                            double minThreshold = 5.0;

                            for (int ii = 0; ii < ellipse12.GetLength(0); ii++)
                            {
                                for (int jj = 0; jj < ellipse13.GetLength(0); jj++)
                                {
                                    if (ellipse12[ii, 0] != 0 && ellipse13[jj, 0] != 0)  //ensure no zero values slipped in
                                    {
                                        dist = Math.Sqrt((ellipse12[ii, 0] - ellipse13[jj, 0]) * (ellipse12[ii, 0] - ellipse13[jj, 0]) +
                                            (ellipse12[ii, 1] - ellipse13[jj, 1]) * (ellipse12[ii, 1] - ellipse13[jj, 1]) +
                                            (ellipse12[ii, 2] - ellipse13[jj, 2]) * (ellipse12[ii, 2] - ellipse13[jj, 2]));
                                        if ((dist < minValue) && dist < minThreshold)
                                        {
                                            minValue = dist;
                                            possibleSol[0] = ellipse12[ii, 0];
                                            possibleSol[1] = ellipse12[ii, 1];
                                            possibleSol[2] = ellipse12[ii, 2];
                                        }
                                    }
                                }
                            }
                            if (possibleSol[0] != 0)
                            {
                                baseZone.UTMEasting = possibleSol[0] + baseLoc[0];
                                baseZone.UTMNorthing = possibleSol[1] + baseLoc[1];
                                LatLonAltCoord_t solPos = LatLonUtmWgs84Conv.UTMtoLL_Degrees(baseZone);
                                aoaTargetEstimate.long_deg = solPos.LongitudeDegrees;  //convert back to long
                                aoaTargetEstimate.lat_deg = solPos.LatitudeDegrees; //convert back to lat
                                aoaTargetEstimate.elev_m = possibleSol[2] + baseLoc[2];
                            }
                        }

                        // log AoA results
                        if (logReceivedDataCheckBox.Checked)
                        {
//                            if (aoaTargetEstimate.lat_deg != 0)  // save everything
                            {

                                double currentTimeSub_sec = (DateTime.UtcNow -
                                    Ref1970_Utc).TotalMilliseconds / 1000.0;
                                bwAoa.Write(term);
                                bwAoa.Write(currentTimeSub_sec);
                                bwAoa.Write(PodAoa[0].pod_pos_lat_deg);
                                bwAoa.Write(PodAoa[0].pod_pos_lon_deg);
                                bwAoa.Write(PodAoa[0].pod_pos_alt_mMSL);
                                bwAoa.Write(PodAoa[0].pod_aoa);
                                bwAoa.Write(PodAoa[1].pod_pos_lat_deg);
                                bwAoa.Write(PodAoa[1].pod_pos_lon_deg);
                                bwAoa.Write(PodAoa[1].pod_pos_alt_mMSL);
                                bwAoa.Write(PodAoa[1].pod_aoa);
                                bwAoa.Write(PodAoa[2].pod_pos_lat_deg);
                                bwAoa.Write(PodAoa[2].pod_pos_lon_deg);
                                bwAoa.Write(PodAoa[2].pod_pos_alt_mMSL);
                                bwAoa.Write(PodAoa[2].pod_aoa);
                                bwAoa.Write(aoaTargetEstimate.lat_deg);
                                bwAoa.Write(aoaTargetEstimate.long_deg);
                                bwAoa.Write(aoaTargetEstimate.elev_m);
                                if (LogSimUav0)  //write data for debug use
                                {
                                    bwAoa.Write(LogSimUav0);
                                    bwAoa.Write(asset_status_List[0].latitude_deg);
                                    bwAoa.Write(asset_status_List[0].longitude_deg);
                                    bwAoa.Write(asset_status_List[0].altitude_m);
                                }
                                else
                                    bwAoa.Write(false);
                                bwAoa.Write(term);
                            }
                        }

                        // Send estimated location to UDP users
                        if (sendAcousticTargetEstimateMsgCheckbox.Checked)
                        {
                            pod_target_estimate_struct podEstimate = new pod_target_estimate_struct();
                            podEstimate.timestamp_sec = (DateTime.UtcNow -
                                    Ref1970_Utc).TotalMilliseconds / 1000.0;
                            podEstimate.targetLatitude_deg = aoaTargetEstimate.lat_deg;
                            podEstimate.targetLongitude_deg = aoaTargetEstimate.long_deg;
                            podEstimate.targetAltitude_mMSL = aoaTargetEstimate.elev_m;
                            podEstimate.estimateSigma_m = 5; //need to develop method
                            podEstimate.pod1Id = PodAoa[0].podId;
                            podEstimate.pod1Aoa_deg = PodAoa[0].pod_aoa;
                            podEstimate.pod2Id = PodAoa[1].podId;
                            podEstimate.pod2Aoa_deg = PodAoa[1].pod_aoa;
                            podEstimate.pod3Id = PodAoa[2].podId;
                            podEstimate.pod3Aoa_deg = PodAoa[2].pod_aoa;

                            byte[] newMsg = outgoingMessage.packPodTargetEstimateMsg(podEstimate, isUdpMessageFormatBigEndian);
                            udpClient.Send(newMsg, podEstimateMsgEndpoint);
                        }
                    }

                }
            }
            catch
            {
            }

        }

        private bool ifAnyGreater(double[,] datain, double value, int axis)
        {
            bool isGreater = false;
            for (int i = 0; i < datain.GetLength(0); i++)
            {
                if (datain[i, axis] >= value)
                    isGreater = true;
            }
            return isGreater;
        }

        private bool ifAnyLess(double[,] datain, double value, int axis)
        {
            bool isLess = false;
            for (int i = 0; i < datain.GetLength(0); i++)
            {
                if (datain[i, axis] < value)
                    isLess = true;
            }
            return isLess;
        }

        // update simulated positions and asset status
        private void mainLoopTimer_Tick(object sender, EventArgs e)
        {
            double currentTime_sec = (DateTime.UtcNow -
                Ref1970_Utc).TotalMilliseconds / 1000.0;
            // move nonTargets
            //   movement is always start point to end point, then repeated. After returning to start point, radar will create a new track number.
            LatLonAltCoord_t tempLoc;
            double rangeRatio = 0;
            for (int i = 0; i < numberOfNonTargets; i++)
            {
                rangeRatio = ((currentTime_sec - non_target_List[i].start_unix_epoch_time_sec) % non_target_List[i].lapTime_sec)/
                    non_target_List[i].lapTime_sec;
                non_target_List[i].asset_info.unix_epoch_time_sec = currentTime_sec;
                non_target_List[i].asset_info.utmCoord.UTMEasting = non_target_List[i].start_utmCoord.UTMEasting + (non_target_List[i].end_utmCoord.UTMEasting -
                    non_target_List[i].start_utmCoord.UTMEasting) * rangeRatio;
                non_target_List[i].asset_info.utmCoord.UTMNorthing = non_target_List[i].start_utmCoord.UTMNorthing + (non_target_List[i].end_utmCoord.UTMNorthing -
                    non_target_List[i].start_utmCoord.UTMNorthing) * rangeRatio;
                tempLoc = LatLonUtmWgs84Conv.UTMtoLL_Degrees(non_target_List[i].asset_info.utmCoord);
                non_target_List[i].asset_info.latitude_deg = tempLoc.LatitudeDegrees;
                non_target_List[i].asset_info.longitude_deg = tempLoc.LongitudeDegrees;
                non_target_List[i].asset_info.radar_report.rcs_dBsm = rcs_dBsm;
            }

            if (simulateTracks)
            {

                // If ingress only, reset track when within radar distance
                if (IngressOnly.Checked)
                {
                    double repeatDist_m = 50;
                    utmCoord_t radarPos = RadarParameter.posUtmCoord;

                    //UAV1
                    utmCoord_t pos = asset_status_List[0].utmCoord;
                    double tempDelta = Math.Abs(pos.UTMEasting - radarPos.UTMEasting);
                    if (tempDelta < repeatDist_m)
                    {
                        asset_status_List[0].trackQueue.Clear();
                        wpIndexUav1 = 0;
                        uav1LatTemp = wayPointsUav1[0, 0];
                        uav1LonTemp = wayPointsUav1[0, 1];
                        uav1VelXtemp = stepVectorsUav1[0, 1] / deg2meterE;
                        uav1VelYtemp = stepVectorsUav1[0, 0] / deg2meterN;
                        uav1VelZtemp = 0;
                    }

                    //UAV2
                    pos = asset_status_List[1].utmCoord;
                    tempDelta = Math.Abs(pos.UTMEasting - radarPos.UTMEasting);
                    if (tempDelta < repeatDist_m)
                    {
                        asset_status_List[1].trackQueue.Clear();
                        wpIndexUav2 = 0;
                        uav2LatTemp = wayPointsUav2[0, 0];
                        uav2LonTemp = wayPointsUav2[0, 1];
                        uav2VelXtemp = stepVectorsUav2[0, 1] / deg2meterE;
                        uav2VelYtemp = stepVectorsUav2[0, 0] / deg2meterN;
                        uav2VelZtemp = 0;
                    }

                    //UAV3
                    pos = asset_status_List[2].utmCoord;
                    tempDelta = Math.Abs(pos.UTMEasting - radarPos.UTMEasting);
                    if (tempDelta < repeatDist_m)
                    {
                        asset_status_List[2].trackQueue.Clear();
                        wpIndexUav3 = 0;
                        uav3LatTemp = wayPointsUav3[0, 0];
                        uav3LonTemp = wayPointsUav3[0, 1];
                        uav3VelXtemp = stepVectorsUav3[0, 1] / deg2meterE;
                        uav3VelYtemp = stepVectorsUav3[0, 0] / deg2meterN;
                        uav3VelZtemp = 0;
                    }
                    //UAV4
                    pos = asset_status_List[3].utmCoord;
                    tempDelta = Math.Abs(pos.UTMEasting - radarPos.UTMEasting);
                    if (tempDelta < repeatDist_m)
                    {
                        asset_status_List[3].trackQueue.Clear();
                        wpIndexUav4 = 0;
                        uav4LatTemp = wayPointsUav4[0, 0];
                        uav4LonTemp = wayPointsUav4[0, 1];
                        uav4VelXtemp = stepVectorsUav4[0, 1] / deg2meterE;
                        uav4VelYtemp = stepVectorsUav4[0, 0] / deg2meterN;
                        uav4VelZtemp = 0;
                    }
                }
                //Define UAV 1 simulated position
                uav1LatTemp += stepVectorsUav1[wpIndexUav1, 0];
                uav1LonTemp += stepVectorsUav1[wpIndexUav1, 1];
                uav1VelXtemp = stepVectorsUav1[wpIndexUav1, 1] / deg2meterE;
                uav1VelYtemp = stepVectorsUav1[wpIndexUav1, 0] / deg2meterN;
                uav1VelZtemp = 0;
                if (Math.Sqrt(Math.Pow((uav1LatTemp - wayPointsUav1[(wpIndexUav1 + 1) % wayPointsUav1.GetLength(0), 0]) / deg2meterN, 2) +
                    Math.Pow((uav1LonTemp - wayPointsUav1[(wpIndexUav1 + 1) % wayPointsUav1.GetLength(0), 1]) / deg2meterE, 2)) < turnToleranceUav1)
                {
                    wpIndexUav1 = (wpIndexUav1 + 1) % wayPointsUav1.GetLength(0);
                }

                //Define UAV 2 simulated position
                uav2LatTemp += stepVectorsUav2[wpIndexUav2, 0];
                uav2LonTemp += stepVectorsUav2[wpIndexUav2, 1];
                uav2VelXtemp = stepVectorsUav2[wpIndexUav2, 1] / deg2meterE;
                uav2VelYtemp = stepVectorsUav2[wpIndexUav2, 0] / deg2meterN;
                uav2VelZtemp = 0;
                if (Math.Sqrt(Math.Pow((uav2LatTemp - wayPointsUav2[(wpIndexUav2 + 1) % wayPointsUav2.GetLength(0), 0]) / deg2meterN, 2) +
                    Math.Pow((uav2LonTemp - wayPointsUav2[(wpIndexUav2 + 1) % wayPointsUav2.GetLength(0), 1]) / deg2meterE, 2)) < turnToleranceUav2)
                {
                    wpIndexUav2 = (wpIndexUav2 + 1) % wayPointsUav2.GetLength(0);
                }

                //Define UAV 3 simulated position
                uav3LatTemp += stepVectorsUav3[wpIndexUav3, 0];
                uav3LonTemp += stepVectorsUav3[wpIndexUav3, 1];
                uav3VelXtemp = stepVectorsUav3[wpIndexUav3, 1] / deg2meterE;
                uav3VelYtemp = stepVectorsUav3[wpIndexUav3, 0] / deg2meterN;
                uav3VelZtemp = 0;
                if (Math.Sqrt(Math.Pow((uav3LatTemp - wayPointsUav3[(wpIndexUav3 + 1) % wayPointsUav3.GetLength(0), 0]) / deg2meterN, 2) +
                    Math.Pow((uav3LonTemp - wayPointsUav3[(wpIndexUav3 + 1) % wayPointsUav3.GetLength(0), 1]) / deg2meterE, 2)) < turnToleranceUav3)
                {
                    wpIndexUav3 = (wpIndexUav3 + 1) % wayPointsUav3.GetLength(0);
                }

                //Define UAV 4 simulated position
                uav4LatTemp += stepVectorsUav4[wpIndexUav4, 0];
                uav4LonTemp += stepVectorsUav4[wpIndexUav4, 1];
                uav4VelXtemp = stepVectorsUav4[wpIndexUav4, 1] / deg2meterE;
                uav4VelYtemp = stepVectorsUav4[wpIndexUav4, 0] / deg2meterN;
                uav4VelZtemp = 0;
                if (Math.Sqrt(Math.Pow((uav4LatTemp - wayPointsUav4[(wpIndexUav4 + 1) % wayPointsUav4.GetLength(0), 0]) / deg2meterN, 2) +
                    Math.Pow((uav4LonTemp - wayPointsUav4[(wpIndexUav4 + 1) % wayPointsUav4.GetLength(0), 1]) / deg2meterE, 2)) < turnToleranceUav4)
                {
                    wpIndexUav4 = (wpIndexUav4 + 1) % wayPointsUav4.GetLength(0);
                }

                asset_status_List[0].latitude_deg = uav1LatTemp;
                asset_status_List[0].longitude_deg = uav1LonTemp;
                asset_status_List[0].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(uav1LatTemp, uav1LonTemp);
                asset_status_List[0].uavAssociation = uav1Number;
                asset_status_List[0].altitude_m = uav1AltTemp;
                asset_status_List[0].velX_mps = uav1VelXtemp;
                asset_status_List[0].velY_mps = uav1VelYtemp;
                asset_status_List[0].velZ_mps = uav1VelZtemp;
                asset_status_List[0].heading = Math.Atan2(asset_status_List[0].velX_mps, asset_status_List[0].velY_mps) / Math.PI * 180;
                asset_status_List[0].unix_epoch_time_sec = currentTime_sec;
                asset_status_List[0].radar_report.rcs_dBsm = rcs_dBsm;
                asset_status_List[1].latitude_deg = uav2LatTemp;
                asset_status_List[1].longitude_deg = uav2LonTemp;
                asset_status_List[1].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(uav2LatTemp, uav2LonTemp);
                asset_status_List[1].uavAssociation = uav2Number;
                asset_status_List[1].altitude_m = uav2AltTemp;
                asset_status_List[1].velX_mps = uav2VelXtemp;
                asset_status_List[1].velY_mps = uav2VelYtemp;
                asset_status_List[1].velZ_mps = uav2VelZtemp;
                asset_status_List[1].heading = Math.Atan2(asset_status_List[1].velX_mps, asset_status_List[1].velY_mps) / Math.PI * 180;
                asset_status_List[1].unix_epoch_time_sec = currentTime_sec;
                asset_status_List[1].radar_report.rcs_dBsm = rcs_dBsm;
                asset_status_List[2].latitude_deg = uav3LatTemp;
                asset_status_List[2].longitude_deg = uav3LonTemp;
                asset_status_List[2].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(uav3LatTemp, uav3LonTemp);
                asset_status_List[2].uavAssociation = uav3Number;
                asset_status_List[2].altitude_m = uav3AltTemp;
                asset_status_List[2].velX_mps = uav3VelXtemp;
                asset_status_List[2].velY_mps = uav3VelYtemp;
                asset_status_List[2].velZ_mps = uav3VelZtemp;
                asset_status_List[2].heading = Math.Atan2(asset_status_List[2].velX_mps, asset_status_List[2].velY_mps) / Math.PI * 180;
                asset_status_List[2].unix_epoch_time_sec = currentTime_sec;
                asset_status_List[2].radar_report.rcs_dBsm = rcs_dBsm;
                asset_status_List[3].latitude_deg = uav4LatTemp;
                asset_status_List[3].longitude_deg = uav4LonTemp;
                asset_status_List[3].utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(uav4LatTemp, uav4LonTemp);
                asset_status_List[3].uavAssociation = uav4Number;
                asset_status_List[3].altitude_m = uav4AltTemp;
                asset_status_List[3].velX_mps = uav4VelXtemp;
                asset_status_List[3].velY_mps = uav4VelYtemp;
                asset_status_List[3].velZ_mps = uav4VelZtemp;
                asset_status_List[3].heading = Math.Atan2(asset_status_List[3].velX_mps, asset_status_List[3].velY_mps) / Math.PI * 180;
                asset_status_List[3].unix_epoch_time_sec = currentTime_sec;
                asset_status_List[3].radar_report.rcs_dBsm = rcs_dBsm;
                // send target positions to UDP users
                if (sendUavTruthMsgCheckbox.Checked)
                {
                    byte[] msgArray = outgoingMessage.packTrueUavLocationMsg(asset_status_List, isUdpMessageFormatBigEndian);
                    if (msgArray.Length > (3 * sizeof(UInt16)))  // only send if target info is available)
                        udpClient.Send(msgArray, trueUavMsgEndpoint);
                }


                updateStatus();
                SimPodTestTargetsInRange();  // Test if simulated target is within range
                UpdateRadarEstimate();
                Asterix048MsgManager();
                CheckPodLostComm();
                AreaImage.Invalidate();         // Force repaint of image
            }
            else
            {
                updateStates();
                updateStatus();
                if (simulatePods)  //if simulating pods, test if targets are in range of sensor
                    SimPodTestTargetsInRange();

                UpdateRadarEstimate();
                Asterix048MsgManager();
                CheckPodLostComm();
                AreaImage.Invalidate();         // Force repaint of image
            }

        }

        // Initialize non-targets (max of four non-targets)
        private void initializeNonTargets()
        {
            // max number of non targets is four
            double currentTime_sec = (DateTime.UtcNow -
                Ref1970_Utc).TotalMilliseconds / 1000.0;
            double tempDist;
            // nonTarget 1
            if (numberOfNonTargets >= 1)
            {
                non_target_List[0].asset_info.unix_epoch_time_sec = currentTime_sec;
                non_target_List[0].start_unix_epoch_time_sec = currentTime_sec;
                non_target_List[0].start_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget1Pt1Lat_deg"]);
                non_target_List[0].start_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget1Pt1Long_deg"]);
                non_target_List[0].end_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget1Pt2Lat_deg"]);
                non_target_List[0].end_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget1Pt2Long_deg"]);
                non_target_List[0].asset_info.altitude_m = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget1Alt_m-MSL"]);
                non_target_List[0].speed_mps = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget1Vel_mps"]);

            }
            // nonTarget 2
            if (numberOfNonTargets >= 2)
            {
                non_target_List[1].asset_info.unix_epoch_time_sec = currentTime_sec;
                non_target_List[1].start_unix_epoch_time_sec = currentTime_sec;
                non_target_List[1].start_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget2Pt1Lat_deg"]);
                non_target_List[1].start_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget2Pt1Long_deg"]);
                non_target_List[1].end_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget2Pt2Lat_deg"]);
                non_target_List[1].end_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget2Pt2Long_deg"]);
                non_target_List[1].asset_info.altitude_m = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget2Alt_m-MSL"]);
                non_target_List[1].speed_mps = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget2Vel_mps"]);
            }
            // nonTarget 3
            if (numberOfNonTargets >= 3)
            {
                non_target_List[2].asset_info.unix_epoch_time_sec = currentTime_sec;
                non_target_List[2].start_unix_epoch_time_sec = currentTime_sec;
                non_target_List[2].start_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget3Pt1Lat_deg"]);
                non_target_List[2].start_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget3Pt1Long_deg"]);
                non_target_List[2].end_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget3Pt2Lat_deg"]);
                non_target_List[2].end_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget3Pt2Long_deg"]);
                non_target_List[2].asset_info.altitude_m = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget3Alt_m-MSL"]);
                non_target_List[2].speed_mps = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget3Vel_mps"]);
            }
            // nonTarget 4
            if (numberOfNonTargets >= 4)
            {
                non_target_List[3].asset_info.unix_epoch_time_sec = currentTime_sec;
                non_target_List[3].start_unix_epoch_time_sec = currentTime_sec;
                non_target_List[3].start_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget4Pt1Lat_deg"]);
                non_target_List[3].start_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget4Pt1Long_deg"]);
                non_target_List[3].end_latitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget4Pt2Lat_deg"]);
                non_target_List[3].end_longitude_deg = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget4Pt2Long_deg"]);
                non_target_List[3].asset_info.altitude_m = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget4Alt_m-MSL"]);
                non_target_List[3].speed_mps = Convert.ToDouble(ConfigurationManager.AppSettings["NonTarget4Vel_mps"]);
            }
            for (int i = 0; i < Math.Min(numberOfNonTargets,4); i++)  // limit to four non targets
            {
                non_target_List[i].asset_info.latitude_deg = non_target_List[i].start_latitude_deg;
                non_target_List[i].asset_info.longitude_deg = non_target_List[i].start_longitude_deg;
                non_target_List[i].asset_info.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(non_target_List[i].asset_info.latitude_deg,
                    non_target_List[i].asset_info.longitude_deg);
                // determine velocities
                non_target_List[i].start_utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(non_target_List[i].start_latitude_deg, non_target_List[i].start_longitude_deg);
                non_target_List[i].end_utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(non_target_List[i].end_latitude_deg, non_target_List[i].end_longitude_deg);
                tempDist = Math.Sqrt((non_target_List[i].start_utmCoord.UTMEasting - non_target_List[i].end_utmCoord.UTMEasting)
                    * (non_target_List[i].start_utmCoord.UTMEasting - non_target_List[i].end_utmCoord.UTMEasting) +
                    (non_target_List[i].start_utmCoord.UTMNorthing - non_target_List[i].end_utmCoord.UTMNorthing)
                    * (non_target_List[i].start_utmCoord.UTMNorthing - non_target_List[i].end_utmCoord.UTMNorthing));
                non_target_List[i].asset_info.velZ_mps = 0;  // no altitude changes
                non_target_List[i].asset_info.velX_mps = (non_target_List[i].end_utmCoord.UTMEasting - non_target_List[i].start_utmCoord.UTMEasting)
                    / tempDist * non_target_List[i].speed_mps;
                non_target_List[i].asset_info.velY_mps = (non_target_List[i].end_utmCoord.UTMNorthing - non_target_List[i].start_utmCoord.UTMNorthing)
                    / tempDist * non_target_List[i].speed_mps;
                non_target_List[i].lapTime_sec = tempDist / non_target_List[i].speed_mps;
                non_target_List[i].asset_info.radar_report.sentEndTrackMessage = true;
            }
        }
        // Initialize simulated target setup
        private void initializeSimulatedTargets()
        {
            // UAV1 settings 

            double tempX, tempY, magXY;
            simSpeedUav1 = Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Vel_mps"]);
            turnToleranceUav1 = 1.5 * simSpeedUav1;
            wpIndexUav1 = 0;
            wayPointsUav1 = new double[,] {
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt1Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt1Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt2Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt2Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt3Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt3Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt4Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Pt4Long_deg"])},
            };

            int arrayLen = wayPointsUav1.GetLength(0);
            stepVectorsUav1 = new double[arrayLen, 2];
            for (int i = 0; i < arrayLen; i++)
            {
                tempX = (wayPointsUav1[(i + 1) % arrayLen, 1] - wayPointsUav1[i, 1]) / deg2meterE;
                tempY = (wayPointsUav1[(i + 1) % arrayLen, 0] - wayPointsUav1[i, 0]) / deg2meterN;
                magXY = Math.Sqrt(tempX * tempX + tempY * tempY);
                stepVectorsUav1[i, 0] = tempY * simSpeedUav1 / magXY * deg2meterN;
                stepVectorsUav1[i, 1] = tempX * simSpeedUav1 / magXY * deg2meterE;
            }
            //set initial conditions
            uav1AltTemp = Convert.ToDouble(ConfigurationManager.AppSettings["Uav1Alt_m-MSL"]);
            uav1LatTemp = wayPointsUav1[0, 0];
            uav1LonTemp = wayPointsUav1[0, 1];
            uav1VelXtemp = stepVectorsUav1[0, 1] / deg2meterE;
            uav1VelYtemp = stepVectorsUav1[0, 0] / deg2meterN;
            uav1VelZtemp = 0;
            // end UAV1 setup

            //UAV2 setting
            simSpeedUav2 = Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Vel_mps"]);
            turnToleranceUav2 = 1.5 * simSpeedUav2;
            wpIndexUav2 = 0;
            wayPointsUav2 = new double[,] {
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt1Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt1Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt2Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt2Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt3Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt3Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt4Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Pt4Long_deg"])},
            };

            arrayLen = wayPointsUav2.GetLength(0);
            stepVectorsUav2 = new double[arrayLen, 2];
            for (int i = 0; i < arrayLen; i++)
            {
                tempX = (wayPointsUav2[(i + 1) % arrayLen, 1] - wayPointsUav2[i, 1]) / deg2meterE;
                tempY = (wayPointsUav2[(i + 1) % arrayLen, 0] - wayPointsUav2[i, 0]) / deg2meterN;
                magXY = Math.Sqrt(tempX * tempX + tempY * tempY);
                stepVectorsUav2[i, 0] = tempY * simSpeedUav2 / magXY * deg2meterN;
                stepVectorsUav2[i, 1] = tempX * simSpeedUav2 / magXY * deg2meterE;
            }
            //set initial conditions
            uav2AltTemp = Convert.ToDouble(ConfigurationManager.AppSettings["Uav2Alt_m-MSL"]);
            uav2LatTemp = wayPointsUav2[0, 0];
            uav2LonTemp = wayPointsUav2[0, 1];
            uav2VelXtemp = stepVectorsUav2[0, 1] / deg2meterE;
            uav2VelYtemp = stepVectorsUav2[0, 0] / deg2meterN;
            uav2VelZtemp = 0;
            // end UAV2 setup

            //UAV3 setting
            simSpeedUav3 = Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Vel_mps"]);
            turnToleranceUav3 = 1.5 * simSpeedUav3;
            wpIndexUav3 = 0;
            wayPointsUav3 = new double[,] {
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt1Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt1Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt2Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt2Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt3Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt3Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt4Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Pt4Long_deg"])},
            };

            arrayLen = wayPointsUav3.GetLength(0);
            stepVectorsUav3 = new double[arrayLen, 2];
            for (int i = 0; i < arrayLen; i++)
            {
                tempX = (wayPointsUav3[(i + 1) % arrayLen, 1] - wayPointsUav3[i, 1]) / deg2meterE;
                tempY = (wayPointsUav3[(i + 1) % arrayLen, 0] - wayPointsUav3[i, 0]) / deg2meterN;
                magXY = Math.Sqrt(tempX * tempX + tempY * tempY);
                stepVectorsUav3[i, 0] = tempY * simSpeedUav3 / magXY * deg2meterN;
                stepVectorsUav3[i, 1] = tempX * simSpeedUav3 / magXY * deg2meterE;
            }
            //set initial conditions
            uav3AltTemp = Convert.ToDouble(ConfigurationManager.AppSettings["Uav3Alt_m-MSL"]);
            uav3LatTemp = wayPointsUav3[0, 0];
            uav3LonTemp = wayPointsUav3[0, 1];
            uav3VelXtemp = stepVectorsUav3[0, 1] / deg2meterE;
            uav3VelYtemp = stepVectorsUav3[0, 0] / deg2meterN;
            uav3VelZtemp = 0;
            // end UAV3 setup

            //UAV4 setting
            simSpeedUav4 = Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Vel_mps"]);
            turnToleranceUav4 = 1.5 * simSpeedUav4;
            wpIndexUav4 = 0;
            wayPointsUav4 = new double[,] {
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt1Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt1Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt2Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt2Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt3Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt3Long_deg"])},
                {Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt4Lat_deg"]),Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Pt4Long_deg"])},
            };

            arrayLen = wayPointsUav4.GetLength(0);
            stepVectorsUav4 = new double[arrayLen, 2];
            for (int i = 0; i < arrayLen; i++)
            {
                tempX = (wayPointsUav4[(i + 1) % arrayLen, 1] - wayPointsUav4[i, 1]) / deg2meterE;
                tempY = (wayPointsUav4[(i + 1) % arrayLen, 0] - wayPointsUav4[i, 0]) / deg2meterN;
                magXY = Math.Sqrt(tempX * tempX + tempY * tempY);
                stepVectorsUav4[i, 0] = tempY * simSpeedUav4 / magXY * deg2meterN;
                stepVectorsUav4[i, 1] = tempX * simSpeedUav4 / magXY * deg2meterE;
            }
            //set initial conditions
            uav4AltTemp = Convert.ToDouble(ConfigurationManager.AppSettings["Uav4Alt_m-MSL"]);
            uav4LatTemp = wayPointsUav4[0, 0];
            uav4LonTemp = wayPointsUav4[0, 1];
            uav4VelXtemp = stepVectorsUav4[0, 1] / deg2meterE;
            uav4VelYtemp = stepVectorsUav4[0, 0] / deg2meterN;
            uav4VelZtemp = 0;
            // end UAV2 setup
        }
        // Simulate target entering pod coverage
        private void SimPodTestTargetsInRange()
        {
            double currentTime_sec = (DateTime.UtcNow -
               Ref1970_Utc).TotalMilliseconds / 1000.0;
            double xDist_m;
            double yDist_m;
            double zDist_m;

            pod_asset_struct tempPod = new pod_asset_struct();
            
            lock(podAssets)
            {
                for (int ii = 1; ii <= podAssets.Count; ii++)
                {
                    if ((!lostCommCheckBox1.Checked & ii == 1) | ii > 1)
                    {
                        tempPod = podAssets[ii];
                        tempPod.lastSystemMsgTime_sec = currentTime_sec;
                        tempPod.timestamp_sec = currentTime_sec;

                        tempPod.targetColorsSolo = displayColors.idleEllipse;
                        tempPod.targetColorsRacer = displayColors.idleEllipse;
                        tempPod.textColorsSolo = Color.Black;
                        tempPod.textColorsRacer = Color.Black;
                        tempPod.targetIdSolo = false;
                        tempPod.targetIdRacer = false;
                        tempPod.podHealthy = true;
                        tempPod.receivedPodMsg.podId = tempPod.podId;
                        tempPod.receivedPodMsg.timestamp_sec = currentTime_sec;
                        tempPod.receivedPodMsg.lat_deg = tempPod.pos_lat_deg;
                        tempPod.receivedPodMsg.long_deg = tempPod.pos_long_deg;
                        tempPod.receivedPodMsg.elevation_mMSL = tempPod.pos_elev_m;
                        tempPod.receivedPodMsg.battery_volt = tempPod.podHealthParameters.battery_volt;
                        tempPod.receivedPodMsg.sendTargetMsg = false;

                        for (int jj = 0; jj < 4; jj++)
                        {
                            xDist_m = (asset_status_List[jj].longitude_deg - tempPod.pos_long_deg)
                                / deg2meterE;
                            yDist_m = (asset_status_List[jj].latitude_deg - tempPod.pos_lat_deg)
                                / deg2meterN;
                            zDist_m = asset_status_List[jj].altitude_m - tempPod.pos_elev_m;

                            if (Math.Sqrt(xDist_m * xDist_m + yDist_m * yDist_m) < RadarParameter.podRadius_m &
                                asset_status_List[jj].EnableTrack & tempPod.podHealthy)
                            {
                                tempPod.receivedPodMsg.sendTargetMsg = true;
                                switch (asset_status_List[jj].targetType)
                                {
                                    case TargetType.Solo:
                                        {
                                            tempPod.targetColorsSolo = displayColors.SoloColor;
                                            tempPod.textColorsSolo = displayColors.SoloColor;
                                            tempPod.targetIdSolo = true;
                                            tempPod.targetSoloRunningResult = Math.Min(2.0, tempPod.targetSoloRunningResult + 1.0);
                                            if (tempPod.isPodStereo)
                                                tempPod.soloSensorEstimates.AoA_deg = Math.Acos(yDist_m / Math.Sqrt(xDist_m * xDist_m + yDist_m * yDist_m + zDist_m * zDist_m)) / Math.PI * 180.0;
                                            else
                                                tempPod.soloSensorEstimates.AoA_deg = 990;
                                            tempPod.receivedPodMsg.targetClassification = pod_classification_enum.solo;
                                            tempPod.receivedPodMsg.aoa_deg = tempPod.soloSensorEstimates.AoA_deg;
                                            break;
                                        }
                                    case TargetType.Racer:
                                        {
                                            tempPod.targetColorsRacer = displayColors.RacerColor;
                                            tempPod.textColorsRacer = displayColors.RacerColor;
                                            tempPod.targetIdRacer = true;
                                            tempPod.targetRacerRunningResult = Math.Min(2.0, tempPod.targetRacerRunningResult + 1.0);
                                            if (tempPod.isPodStereo)
                                                tempPod.racerSensorEstimates.AoA_deg = Math.Acos(yDist_m / Math.Sqrt(xDist_m * xDist_m + yDist_m * yDist_m + zDist_m * zDist_m)) / Math.PI * 180.0;
                                            else
                                                tempPod.soloSensorEstimates.AoA_deg = 990;
                                            tempPod.receivedPodMsg.targetClassification = pod_classification_enum.racingDrone;
                                            tempPod.receivedPodMsg.aoa_deg = tempPod.racerSensorEstimates.AoA_deg;
                                            break;
                                        }
                                }
                            }
                        }

                        podAssets[ii] = tempPod;
                    }
                }
            }
        }

        private void CheckPodLostComm()
        {
            double currentTime_sec = (DateTime.UtcNow -
                Ref1970_Utc).TotalMilliseconds / 1000.0;

            pod_asset_struct tempPod = new pod_asset_struct();
            List<int> lostCommPods = new List<int>();

            foreach (var tempAsset in podAssets)
            {
                if ((currentTime_sec - tempAsset.Value.lastSystemMsgTime_sec) > podLostCommTimeout_sec)
                {
                    lostCommPods.Add(tempAsset.Key);
                }
            }
            if (lostCommPods.Count > 0)
            {
                lock (podAssets)
                {
                    foreach (var podIndex in lostCommPods)
                    {
                        tempPod = podAssets[podIndex];
                        tempPod.podHealthy = false;
                        tempPod.targetColorsRacer = displayColors.noSignal;
                        tempPod.targetColorsSolo = displayColors.noSignal;
                        tempPod.textColorsSolo = Color.Black;
                        tempPod.textColorsRacer = Color.Black;
                        podAssets[podIndex] = tempPod;
                    }
                }
            }
        }
        // close threads and logs when GUI form is closed
        private void FormSoloSimRadar_FormClosing(object sender, FormClosingEventArgs e)
        {
            try
            {
                udpPodServerMsgThread.Abort();
                
            }
            catch
            {
            }
            // close binary log file
            try
            {
                bw.Close();
            }
            catch { }
            try
            {
                bwAir.Close();
            }
            catch { }
            try
            {
                bwAoa.Close();
            }
            catch { }

            // close serial ports
            try
            {
                if (enableSerialComm1Checkbox.Checked)
                {
                    uavStatus1.StopSerialCom();
                    uav1MsgThread.Abort();
                }

            }
            catch { }
            try
            {
                if (enableSerialComm2Checkbox.Checked)
                {
                    uavStatus2.StopSerialCom();
                    uav2MsgThread.Abort();
                }
            }
            catch { }
            try
            {
                if (enableSerialComm3Checkbox.Checked)
                {
                    uavStatus3.StopSerialCom();
                    uav3MsgThread.Abort();
                }
            }
            catch { }
            try
            {
                if (enableSerialComm4Checkbox.Checked)
                {
                    uavStatus4.StopSerialCom();
                    uav4MsgThread.Abort();
                }
            }
            catch { }

            Environment.Exit(0);
        }


        private void asset_1_enable_track_CheckedChanged(object sender, EventArgs e)
        {
            if (asset_1_enable_track.Checked)
            {
                asset_status_List[0].EnableTrack = true;
                asset_status_List[0].trackQueue.Clear();
                asset_status_List[0].radar_report.sentEndTrackMessage = true;
                
            }
            else
                asset_status_List[0].EnableTrack = false;

        }

        private void asset_2_enable_track_CheckedChanged(object sender, EventArgs e)
        {
            if (asset_2_enable_track.Checked)
            {
                asset_status_List[1].EnableTrack = true;
                asset_status_List[1].trackQueue.Clear();
                asset_status_List[1].radar_report.sentEndTrackMessage = true;
            }
            else
                asset_status_List[1].EnableTrack = false;

        }


        private void asset_3_enable_track_CheckedChanged(object sender, EventArgs e)
        {
            if (asset_3_enable_track.Checked)
            {
                asset_status_List[2].EnableTrack = true;
                asset_status_List[2].trackQueue.Clear();
                asset_status_List[2].radar_report.sentEndTrackMessage = true;
            }
            else
                asset_status_List[2].EnableTrack = false;

        }

        private void asset_4_enable_track_CheckedChanged(object sender, EventArgs e)
        {
            if (asset_4_enable_track.Checked)
            {
                asset_status_List[3].EnableTrack = true;
                asset_status_List[3].trackQueue.Clear();
                asset_status_List[3].radar_report.sentEndTrackMessage = true;
            }
            else
                asset_status_List[3].EnableTrack = false;

        }

        private void ClrTracksBtn_Click(object sender, EventArgs e)
        {
            for(int kk=0; kk<4; kk++)
                asset_status_List[kk].trackQueue.Clear();
            wpIndexUav1 = 0;
            uav1LatTemp = wayPointsUav1[0, 0];
            uav1LonTemp = wayPointsUav1[0, 1];
            wpIndexUav2 = 0;
            uav2LatTemp = wayPointsUav2[0, 0];
            uav2LonTemp = wayPointsUav2[0, 1];
            wpIndexUav3 = 0;
            uav3LatTemp = wayPointsUav3[0, 0];
            uav3LonTemp = wayPointsUav3[0, 1];
            wpIndexUav4 = 0;
            uav4LatTemp = wayPointsUav4[0, 0];
            uav4LonTemp = wayPointsUav4[0, 1];
        }

        private void BaseLat_deg_TextChanged(object sender, EventArgs e)
        {
            RadarParameter.latitude_deg = Convert.ToDouble(RadarLat_deg.Text);
            RadarParameter.posUtmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(RadarParameter.latitude_deg, RadarParameter.longitude_deg);
        }

        private void BaseLong_deg_TextChanged(object sender, EventArgs e)
        {
            RadarParameter.longitude_deg = Convert.ToDouble(RadarLong_deg.Text);
            RadarParameter.posUtmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(RadarParameter.latitude_deg, RadarParameter.longitude_deg);
        }

        private void PodRadius_m_TextChanged(object sender, EventArgs e)
        {
            RadarParameter.podRadius_m = Convert.ToDouble(PodRadius_m.Text);
        }


        private void RtlCntlUav1Btn_Click(object sender, EventArgs e)
        {
            RtlCntlUav1Btn.ForeColor = Color.Red;
            AutoCntlUav1Btn.ForeColor = Color.Black;
            if (enableSerialComm1Checkbox.Checked)
                uavStatus1.SendUavHome(mavlinkInterface.Serial);
            else
                uavStatus1.SendUavHome(mavlinkInterface.Udp);
        }

        private void AutoCntlUav1Btn_Click(object sender, EventArgs e)
        {
            AutoCntlUav1Btn.ForeColor = Color.Red;
            RtlCntlUav1Btn.ForeColor = Color.Black;
            if (enableSerialComm1Checkbox.Checked)
                uavStatus1.SetModeAuto(mavlinkInterface.Serial);
            else
                uavStatus1.SetModeAuto(mavlinkInterface.Udp);
        }

        private void RtlCntlUav2Btn_Click(object sender, EventArgs e)
        {
            RtlCntlUav2Btn.ForeColor = Color.Red;
            AutoCntlUav2Btn.ForeColor = Color.Black;
            if (enableSerialComm2Checkbox.Checked)
                uavStatus2.SendUavHome(mavlinkInterface.Serial);
            else
                uavStatus2.SendUavHome(mavlinkInterface.Udp);
        }

        private void AutoCntlUav2Btn_Click(object sender, EventArgs e)
        {
            AutoCntlUav2Btn.ForeColor = Color.Red;
            RtlCntlUav2Btn.ForeColor = Color.Black;
            if (enableSerialComm2Checkbox.Checked)
                uavStatus2.SetModeAuto(mavlinkInterface.Serial);
            else
                uavStatus2.SetModeAuto(mavlinkInterface.Udp);
        }

        private void RtlCntlUav3Btn_Click(object sender, EventArgs e)
        {
            RtlCntlUav3Btn.ForeColor = Color.Red;
            AutoCntlUav3Btn.ForeColor = Color.Black;
            if (enableSerialComm3Checkbox.Checked)
                uavStatus3.SendUavHome(mavlinkInterface.Serial);
            else
                uavStatus3.SendUavHome(mavlinkInterface.Udp);
        }

        private void AutoCntlUav3Btn_Click(object sender, EventArgs e)
        {
            AutoCntlUav3Btn.ForeColor = Color.Red;
            RtlCntlUav3Btn.ForeColor = Color.Black;
            if (enableSerialComm3Checkbox.Checked)
                uavStatus3.SetModeAuto(mavlinkInterface.Serial);
            else
                uavStatus3.SetModeAuto(mavlinkInterface.Udp);

        }

        private void RtlCntlUav4Btn_Click(object sender, EventArgs e)
        {
            RtlCntlUav4Btn.ForeColor = Color.Red;
            AutoCntlUav4Btn.ForeColor = Color.Black;
            if (enableSerialComm4Checkbox.Checked)
                uavStatus4.SendUavHome(mavlinkInterface.Serial);
            else
                uavStatus4.SendUavHome(mavlinkInterface.Udp);
        }

        private void AutoCntlUav4Btn_Click(object sender, EventArgs e)
        {
            AutoCntlUav4Btn.ForeColor = Color.Red;
            RtlCntlUav4Btn.ForeColor = Color.Black;
            if (enableSerialComm4Checkbox.Checked)
                uavStatus4.SetModeAuto(mavlinkInterface.Serial);
            else
                uavStatus4.SetModeAuto(mavlinkInterface.Udp);
        }


        private void pod1Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod1Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[1];
                tempPod.pos_lat_deg = Convert.ToDouble(pod1Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);

                podAssets[1] = tempPod;
            }
        }

        private void pod1Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod1Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[1];
                tempPod.pos_long_deg = Convert.ToDouble(pod1Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[1] = tempPod;
            }
        }

        private void pod2Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod2Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[2];
                tempPod.pos_lat_deg = Convert.ToDouble(pod2Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[2] = tempPod;
            }
        }

        private void pod2Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod2Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[2];
                tempPod.pos_long_deg = Convert.ToDouble(pod2Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[2] = tempPod;
            }
        }

        private void pod3Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod3Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[3];
                tempPod.pos_lat_deg = Convert.ToDouble(pod3Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[3] = tempPod;
            }
        }

        private void pod3Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod3Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[3];
                tempPod.pos_long_deg = Convert.ToDouble(pod3Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[3] = tempPod;
            }
        }

        private void pod4Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod4Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[4];
                tempPod.pos_lat_deg = Convert.ToDouble(pod4Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[4] = tempPod;
            }
        }

        private void pod4Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod4Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[4];
                tempPod.pos_long_deg = Convert.ToDouble(pod4Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[4] = tempPod;
            }
            
        }
        private void pod5Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod5Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[5];
                tempPod.pos_lat_deg = Convert.ToDouble(pod5Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[5] = tempPod;
            }

        }

        private void pod5Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod5Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[5];
                tempPod.pos_long_deg = Convert.ToDouble(pod5Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[5] = tempPod;
            }

        }

        private void pod6Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod6Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[6];
                tempPod.pos_lat_deg = Convert.ToDouble(pod6Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[6] = tempPod;
            }

        }

        private void pod6Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod6Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[6];
                tempPod.pos_long_deg = Convert.ToDouble(pod6Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[6] = tempPod;
            }

        }

        private void pod7Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod7Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[7];
                tempPod.pos_lat_deg = Convert.ToDouble(pod7Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[7] = tempPod;
            }

        }

        private void pod7Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod7Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[7];
                tempPod.pos_long_deg = Convert.ToDouble(pod7Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[7] = tempPod;
            }

        }

        private void pod8Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod8Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[8];
                tempPod.pos_lat_deg = Convert.ToDouble(pod8Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[8] = tempPod;
            }

        }

        private void pod8Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod8Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[8];
                tempPod.pos_long_deg = Convert.ToDouble(pod8Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[8] = tempPod;
            }

        }

        private void pod9Lat_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod9Lat_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[9];
                tempPod.pos_lat_deg = Convert.ToDouble(pod9Lat_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[9] = tempPod;
            }

        }

        private void pod9Long_tbox_TextChanged(object sender, EventArgs e)
        {
            if (pod9Long_tbox.Text != "" & !usePodLocationData)
            {
                pod_asset_struct tempPod = podAssets[9];
                tempPod.pos_long_deg = Convert.ToDouble(pod9Long_tbox.Text);
                tempPod.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(tempPod.pos_lat_deg, tempPod.pos_long_deg);
                podAssets[9] = tempPod;
            }

        }
        public struct non_target_status_t
        {
            public asset_status_t asset_info;
            public double start_unix_epoch_time_sec;
            public double start_latitude_deg;
            public double start_longitude_deg;
            public double end_latitude_deg;
            public double end_longitude_deg;
            public utmCoord_t start_utmCoord;
            public utmCoord_t end_utmCoord;
            public double speed_mps;
            public double lapTime_sec;

        }


        public struct asset_status_t
        {
            public double unix_epoch_time_sec;
            public int uavAssociation;
            public double latitude_deg;
            public double longitude_deg;
            public utmCoord_t utmCoord;
            public double altitude_m;
            public double heading;
            public double battery_voltage;
            public Ellipse ellipse;
            public bool EnableTrack;
            public Queue trackQueue;
            public double velX_mps;
            public double velY_mps;
            public double velZ_mps;
            public double alt_offset_m;  // artificially change target altitude
            public TargetType targetType;
            public radar_report_t radar_report;
            
        }

        public struct radar_report_t
        {
            public int trackNumber;
            public double range_m;  //range from radar, including sigma noise
            public double azimuth_deg;  //azimuth from radar, including sigma noise
            public double elevation_deg;  //elevation from radar, including sigma noise
            public double x_relative_m;  //x (east) from radar, including sigma noise
            public double y_relative_m;  //y (north) from radar, including sigma noise
            public double z_relative_m;   //z (altitude) from radar, including sigma noise
            public double horStdDev_m;
            public double verStdDev_m;
            public double GndSpdStdDev_mps;
            public double headingStdDev_deg;
            public double gndVel_mps; // horizontal velocity in m/s;
            public double heading_deg; // heading of target in degrees;
            public double latitude_calc_deg;  // latitude based on noisy x,y
            public double longitude_calc_deg;  //longitude base on noisy x,y
            public double rcs_dBsm;  // radar cross section in dBm sq meters
            public int detectCntr; // count of number of detections
            public double trackQuality;
            public lstar_classification targetClass;
            public double timeFirstPlot_sec;  //time for first detection
            public double biomass;
            public bool ShouldCauseAlert;
            public double timeMsgSent_sec;
            public double timeMsgRecv_sec;
            public double classConfidence; // how confident of classification 0 - 100
            public bool sentEndTrackMessage;  //used to ensure only one end of track message sent
        }
        public enum lstar_classification
        {
            unknown=0,
            clutter=1,
            aircraft=2,
            projectile=3,
            bird=4,
            groundVehicle=5,
            targetNonInterest=6,
            pending=7,
        }
        public struct pod_status_t
        {
            public bool podHealthy;
            public bool targetInRange;
            public double latitude_deg;
            public double longitude_deg;
            public double elevation_m;
            public Color currentColor;
            public double lastMsgTime;
        }
        
        public struct radar_parameters_t
        {
            public double utcTime_sec;
            public double latitude_deg;
            public double longitude_deg;
            public double elevation_mMSL;
            public double podRadius_m;
            public double minRadarRange_m;
            public double maxRadarRange_m;
            public utmCoord_t posUtmCoord;
            public lstar_parameters_t radarParam;
            public double range_sigma_m;
            public double azimuth_beam_width_deg;
            public double elevation_beam_width_deg;
        }

        public struct lstar_parameters_t
        {
            public byte SAC;
            public byte SIC;
            public short S_N;
            public double rot_period_s;
            public int PSRamplitude_dBm;
        }
        private void checkPodMsgTimer_Tick(object sender, EventArgs e)
        {
            // When messages are received from the acoustic pod server, they are entered into a message queue.  
            // The PodMsgTimer is used to periodically check the message queue and act on any received messages.
            // Also, these messages are forwarded to other users that may find them useful.


            // Message format
            //======================================================
            //podMsg = (PodMsgType)BitConverter.ToDouble (0:health, 1: target)
            //podId = (int)BitConverter.ToDouble
            //timestamp_sec = BitConverter.ToDouble
            //latitude_deg = BitConverter.ToDouble
            //longitude_deg = BitConverter.ToDouble
            //elevation_mMSL = BitConverter.ToDouble
            // if target message
            //   podEstimate.targetType = (short)BitConverter.ToDouble
            //   podEstimate.AoA_deg = BitConverter.ToDouble
            // if pod health message
            //   podHealth.battery_volt = BitConverter.ToDouble

            // server pod location message
            // podMsg (2) server position message
            // latitude_deg
            // longitude_deg
            // elevation_m_MSL
            //=======================================================

            byte[] msgIn = null;
            byte[] msg = null;
            pod_asset_struct podAssetPlaceholder;
            PodMsgType podMsg;
            AcousticSource acousticSource;
            int podId;
            int msg_offset;
            TargetType tempTarget = TargetType.Background;
            int tempCntr;
            double aoa_deg = 0;
            double currentTime_sec = (DateTime.UtcNow -
                Ref1970_Utc).TotalMilliseconds / 1000.0;
            // Make terminator
            while (incomingPodMsg.msgQueue.Count > 0)
            {
                lock(_locker)
                    msg = incomingPodMsg.msgQueue.Dequeue();
                int len = BitConverter.ToUInt16(msg, 2);
                msgIn = new byte[len - 4];
                Array.Copy(msg, 4, msgIn, 0, len - 4);  // strip off message ID and size

                msg_offset = 0;
                podAssetPlaceholder=new pod_asset_struct();
                podMsg = (PodMsgType)BitConverter.ToUInt16(msgIn, msg_offset); msg_offset += sizeof(UInt16);
                if (podMsg == PodMsgType.Server)
                    acousticSource = AcousticSource.Server;
                else
                    acousticSource = AcousticSource.Pod;

                switch (acousticSource)
                {
                    case AcousticSource.Pod:
                        {
                            // send pod Message to cadets
                            if(sendAcousticPodMsgCheckbox.Checked)
                                udpClient.Send(msg, podSimMsgEndpoint);

                            // interpret pod messages
                            lock(podAssets)
                            {
                                podId = (int)BitConverter.ToUInt16(msgIn, msg_offset); msg_offset += sizeof(UInt16);
                                podAssetPlaceholder.receivedPodMsg.podId = podId;

                                if (PodAssociations.ContainsKey(podId))  // copy existing info
                                {
                                    podAssetPlaceholder = podAssets[PodAssociations[podId]];
                                }
                                else  // initialize pod info
                                {
                                    podAssetPlaceholder.podId = podId;
                                    podAssetPlaceholder.targetIdSolo = false;
                                    podAssetPlaceholder.targetIdRacer = false;
                                    podAssetPlaceholder.targetColorsSolo = displayColors.idleEllipse;
                                    podAssetPlaceholder.targetColorsRacer = displayColors.idleEllipse;
                                    podAssetPlaceholder.textColorsSolo = Color.Black;
                                    podAssetPlaceholder.textColorsRacer = Color.Black;
                                    podAssetPlaceholder.podHealthy = true;  // assume healthy since message received
                                }
                                podAssetPlaceholder.timestamp_sec = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                podAssetPlaceholder.receivedPodMsg.timestamp_sec = podAssetPlaceholder.timestamp_sec;
                                if (usePodLocationData)
                                {
                                    podAssetPlaceholder.pos_lat_deg = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                    podAssetPlaceholder.receivedPodMsg.lat_deg = podAssetPlaceholder.pos_lat_deg;

                                    podAssetPlaceholder.pos_long_deg = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                    podAssetPlaceholder.receivedPodMsg.long_deg = podAssetPlaceholder.pos_long_deg;

                                    podAssetPlaceholder.utmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(podAssetPlaceholder.pos_lat_deg, podAssetPlaceholder.pos_long_deg);
                                    podAssetPlaceholder.pos_elev_m = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                    podAssetPlaceholder.receivedPodMsg.elevation_mMSL = podAssetPlaceholder.pos_elev_m;
                                }
                                else
                                {
                                    msg_offset += 3 * sizeof(double);
                                }
                                podAssetPlaceholder.lastSystemMsgTime_sec = (DateTime.UtcNow -
                                    Ref1970_Utc).TotalMilliseconds / 1000.0;

                                switch (podMsg)
                                {
                                    case PodMsgType.Target:
                                        {
                                            // update target values
                                            tempTarget = (TargetType)BitConverter.ToUInt16(msgIn, msg_offset);
                                            msg_offset += sizeof(UInt16);
                                            aoa_deg = BitConverter.ToDouble(msgIn, msg_offset);
                                            if (aoa_deg == 90)
                                                aoa_deg *= 0.999;  //add slight offset to angle if 90 degrees to ensure no divide by zero
                                            msg_offset += sizeof(double);

                                            switch (tempTarget)
                                            {
                                                case TargetType.Solo:
                                                    {
                                                        // save AoA data
                                                        podAssetPlaceholder.soloSensorEstimates.AoA_deg = aoa_deg;

                                                        // update running value
                                                        podAssetPlaceholder.targetSoloRunningResult = Math.Min(2.0, podAssetPlaceholder.targetSoloRunningResult + 1.0);

                                                        break;
                                                    }
                                                case TargetType.Racer:
                                                    {
                                                        //save AoA
                                                        podAssetPlaceholder.racerSensorEstimates.AoA_deg = aoa_deg;

                                                        // update running value
                                                        podAssetPlaceholder.targetRacerRunningResult = Math.Min(2.0, podAssetPlaceholder.targetRacerRunningResult + 1.0);

                                                        break;
                                                    }
                                            }
                                            break;
                                        }
                                    case PodMsgType.PodHealth:
                                        {
                                            podAssetPlaceholder.podHealthParameters.battery_volt = BitConverter.ToDouble(msgIn, msg_offset);
//                                            msg_offset += sizeof(double);
                                            if (podAssetPlaceholder.podHealthParameters.battery_volt >=
                                                podThresholdVoltage)
                                                podAssetPlaceholder.podHealthy = true;
                                            else
                                                podAssetPlaceholder.podHealthy = false;
                                            break;
                                        }
                                }
                                // 
                                if (!PodAssociations.ContainsKey(podId))  // create new pod
                                {
                                    tempCntr = podAssets.Count + 1;
                                    podAssets.Add(tempCntr, podAssetPlaceholder);
                                    PodAssociations.Add(podId, tempCntr);
                                    addPodToTable();

                                }
                                else
                                {
                                    podAssets[PodAssociations[podId]] = podAssetPlaceholder;
                                }
                            }
                            // log data if checked
                            if (logReceivedDataCheckBox.Checked)
                            {
                                double currentTimeSub_sec = (DateTime.UtcNow -
                                    Ref1970_Utc).TotalMilliseconds / 1000.0;
                                bw.Write(term);
                                bw.Write(currentTimeSub_sec);
                                bw.Write(podAssetPlaceholder.lastSystemMsgTime_sec);  // current system time
                                bw.Write(podAssetPlaceholder.targetIdSolo);  // current outputted Solo result
                                bw.Write(podAssetPlaceholder.targetIdRacer);  // current outputted Racer result
                                bw.Write(podAssetPlaceholder.targetSoloRunningResult);  // current running Solo metric
                                bw.Write(podAssetPlaceholder.targetRacerRunningResult);  // current running Racer metric
                                bw.Write(msgIn);  // save actual message
                                bw.Write(term);
                            }
                            break;
                        }
                    case AcousticSource.Server:
                        {
                            // Pod server reported location is used for simulated radar location
                            if (usePodServerGpsLocation)
                            {
                                RadarParameter.latitude_deg = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                RadarParameter.longitude_deg = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                RadarParameter.elevation_mMSL = BitConverter.ToDouble(msgIn, msg_offset); msg_offset += sizeof(double);
                                RadarLat_deg.Text = RadarParameter.latitude_deg.ToString("f6");
                                RadarLong_deg.Text = RadarParameter.longitude_deg.ToString("f6");
                                RadarElev_m.Text = RadarParameter.elevation_mMSL.ToString("f2");
                                RadarParameter.posUtmCoord = LatLonUtmWgs84Conv.LLtoUTM_Degrees(RadarParameter.latitude_deg, RadarParameter.longitude_deg);
                            }
                            break;
                        }
                }
            }

        }
        public void addPodToTable()
        {
            // need to find better way to index table positions
            int newPodIndex = podAssets.Count;
            switch (newPodIndex)
            {
                case (1):
                    {
                        pod1Label.Text = "Pod " + podAssets[1].podId.ToString("d1");
                        pod1Lat_tbox.Text = podAssets[1].pos_lat_deg.ToString("f6");
                        pod1Long_tbox.Text = podAssets[1].pos_long_deg.ToString("f6");
                        pod1Label.Visible = true;
                        pod1Lat_tbox.Visible = true;
                        pod1Long_tbox.Visible = true;
                        break;
                    }
                case (2):
                    {
                        pod2Label.Text = "Pod " + podAssets[2].podId.ToString("d1");
                        pod2Lat_tbox.Text = podAssets[2].pos_lat_deg.ToString("f6");
                        pod2Long_tbox.Text = podAssets[2].pos_long_deg.ToString("f6");
                        pod2Label.Visible = true;
                        pod2Lat_tbox.Visible = true;
                        pod2Long_tbox.Visible = true;
                        break;
                    }
                case (3):
                    {
                        pod3Label.Text = "Pod " + podAssets[3].podId.ToString("d1");
                        pod3Lat_tbox.Text = podAssets[3].pos_lat_deg.ToString("f6");
                        pod3Long_tbox.Text = podAssets[3].pos_long_deg.ToString("f6");
                        pod3Label.Visible = true;
                        pod3Lat_tbox.Visible = true;
                        pod3Long_tbox.Visible = true;
                        break;
                    }
                case (4):
                    {
                        pod4Label.Text = "Pod " + podAssets[4].podId.ToString("d1");
                        pod4Lat_tbox.Text = podAssets[4].pos_lat_deg.ToString("f6");
                        pod4Long_tbox.Text = podAssets[4].pos_long_deg.ToString("f6");
                        pod4Label.Visible = true;
                        pod4Lat_tbox.Visible = true;
                        pod4Long_tbox.Visible = true;
                        break;
                    }
                case (5):
                    {
                        pod5Label.Text = "Pod " + podAssets[5].podId.ToString("d1");
                        pod5Lat_tbox.Text = podAssets[5].pos_lat_deg.ToString("f6");
                        pod5Long_tbox.Text = podAssets[5].pos_long_deg.ToString("f6");
                        pod5Label.Visible = true;
                        pod5Lat_tbox.Visible = true;
                        pod5Long_tbox.Visible = true;
                        break;
                    }
                case (6):
                    {
                        pod6Label.Text = "Pod " + podAssets[6].podId.ToString("d1");
                        pod6Lat_tbox.Text = podAssets[6].pos_lat_deg.ToString("f6");
                        pod6Long_tbox.Text = podAssets[6].pos_long_deg.ToString("f6");
                        pod6Label.Visible = true;
                        pod6Lat_tbox.Visible = true;
                        pod6Long_tbox.Visible = true;
                        break;
                    }
                case (7):
                    {
                        pod7Label.Text = "Pod " + podAssets[7].podId.ToString("d1");
                        pod7Lat_tbox.Text = podAssets[7].pos_lat_deg.ToString("f6");
                        pod7Long_tbox.Text = podAssets[7].pos_long_deg.ToString("f6");
                        pod7Label.Visible = true;
                        pod7Lat_tbox.Visible = true;
                        pod7Long_tbox.Visible = true;
                        break;
                    }
                case (8):
                    {
                        pod8Label.Text = "Pod " + podAssets[8].podId.ToString("d1");
                        pod8Lat_tbox.Text = podAssets[8].pos_lat_deg.ToString("f6");
                        pod8Long_tbox.Text = podAssets[8].pos_long_deg.ToString("f6");
                        pod8Label.Visible = true;
                        pod8Lat_tbox.Visible = true;
                        pod8Long_tbox.Visible = true;
                        break;
                    }
                case (9):
                    {
                        pod9Label.Text = "Pod " + podAssets[9].podId.ToString("d1");
                        pod9Lat_tbox.Text = podAssets[9].pos_lat_deg.ToString("f6");
                        pod9Long_tbox.Text = podAssets[9].pos_long_deg.ToString("f6");
                        pod9Label.Visible = true;
                        pod9Lat_tbox.Visible = true;
                        pod9Long_tbox.Visible = true;
                        break;
                    }
                default:
                    {
                        break;
                    }
            }
        }
        // create logs for post-analysis
        private void logReceivedDataCheckBox_CheckedChanged(object sender, EventArgs e)
        {
            if (logReceivedDataCheckBox.Checked)
            {
                double utcTimeT = (DateTime.UtcNow -
                    new DateTime(1970, 1, 1, 0, 0, 0, DateTimeKind.Utc)).TotalMilliseconds / 1000.0;     //capture computer time to send with messages

                rawDataFilename = rawDataFilenameBase + ((int)utcTimeT).ToString() + ".dat";
                aircraftDataFilename = aircraftDataFilenameBase + ((int)utcTimeT).ToString() + ".dat";
                aoaDataFilename = aoaDataFilenameBase + ((int)utcTimeT).ToString() + ".dat";
                // create binary file for raw data

                try
                {
                    bw = new BinaryWriter(new FileStream(rawDataFilename, FileMode.Create));
                }
                catch (IOException ex)
                {
                    Console.Write(ex.ToString());
                }
                try
                {
                    bwAir = new BinaryWriter(new FileStream(aircraftDataFilename, FileMode.Create));
                }
                catch (IOException ex)
                {
                    Console.Write(ex.ToString());
                }
                try
                {
                    bwAoa = new BinaryWriter(new FileStream(aoaDataFilename, FileMode.Create));
                }
                catch (IOException ex)
                {
                    Console.Write(ex.ToString());
                }
            }
            else
            {
                bw.Close();
                bwAir.Close();
                bwAoa.Close();
            }
            

        }

        // used to determine is target is still detected by pod.  Ensures single wrong detections are not displayed, and also
        // continues presenting a continuous detection if one detection is missed.
        private void targetDecayTimer_Tick(object sender, EventArgs e)
        {
            pod_asset_struct tempPod = new pod_asset_struct();
            lock (podAssets)
            {
                // decay pod target values
                for (int ii = 1; ii <= podAssets.Count; ii++)
                {
                    tempPod = podAssets[ii];

                    if (tempPod.targetRacerRunningResult > reportTargetThreshold)
                    {
                        tempPod.targetIdRacer = true;
                        tempPod.targetColorsRacer = displayColors.RacerColor;
                        tempPod.textColorsRacer = displayColors.RacerColor;
                    }
                    else
                    {
                        tempPod.targetIdRacer = false;
                        tempPod.targetColorsRacer = displayColors.idleEllipse;
                        tempPod.textColorsRacer = Color.Black;
                    }
                    tempPod.targetRacerRunningResult *= oldTargetDegradeRate;

                    if (tempPod.targetSoloRunningResult > reportTargetThreshold)
                    {
                        tempPod.targetIdSolo = true;
                        tempPod.targetColorsSolo = displayColors.SoloColor;
                        tempPod.textColorsSolo = displayColors.SoloColor;
                    }
                    else
                    {
                        tempPod.targetIdSolo = false;
                        tempPod.targetColorsSolo = displayColors.idleEllipse;
                        tempPod.textColorsSolo = Color.Black;
                    }
                    tempPod.targetSoloRunningResult *= oldTargetDegradeRate;
                    podAssets[ii] = tempPod;

                }
            }
        }

        // ground station must send heartbeat messages to Mavlink compliant UAVs and also request what information the UAVs should stream to the ground station
        private void mavlinkHeartbeat_timer_Tick(object sender, EventArgs e)
        {
            const int maxStreams = 3;
            MAVLink.MAV_DATA_STREAM[] MavStreams = new MAVLink.MAV_DATA_STREAM[maxStreams] { MAVLink.MAV_DATA_STREAM.POSITION, MAVLink.MAV_DATA_STREAM.EXTENDED_STATUS, MAVLink.MAV_DATA_STREAM.EXTRA3 };
            ushort[] MavRates = new ushort[maxStreams] { 0x02, 0x02, 0x02 };

            if (enableSerialComm1Checkbox.Checked)
            {
                if (!uavStatus1.streamsStarted)  //request data streams
                {
                    for (int i = 0; i < maxStreams; i++)
                    {
                        uavStatus1.RequestDataStream(MavStreams[i], MavRates[i], 1, mavlinkInterface.Serial);  // send stream request until streams are received
                    }
                }
                else  // send heartbeat
                    uavStatus1.SendHeartBeat(mavlinkInterface.Serial);
            }

            if (enableSerialComm2Checkbox.Checked)
            {
                if (!uavStatus2.streamsStarted)  //request data streams
                {
                    for (int i = 0; i < maxStreams; i++)
                    {
                        uavStatus2.RequestDataStream(MavStreams[i], MavRates[i], 1, mavlinkInterface.Serial);
                    }
                }
                else  // send heartbeat
                    uavStatus2.SendHeartBeat(mavlinkInterface.Serial);
            }

            if (enableSerialComm3Checkbox.Checked)
            {
                if (!uavStatus3.streamsStarted)  //request data streams
                {
                    for (int i = 0; i < maxStreams; i++)
                    {
                        uavStatus3.RequestDataStream(MavStreams[i], MavRates[i], 1, mavlinkInterface.Serial);
                    }
                }
                else  // send heartbeat
                    uavStatus3.SendHeartBeat(mavlinkInterface.Serial);
            }

            if (enableSerialComm4Checkbox.Checked)
            {
                if (!uavStatus4.streamsStarted)  //request data streams
                {
                    for (int i = 0; i < maxStreams; i++)
                    {
                        uavStatus4.RequestDataStream(MavStreams[i], MavRates[i], 1, mavlinkInterface.Serial);
                    }
                }
                else  // send heartbeat
                    uavStatus4.SendHeartBeat(mavlinkInterface.Serial);
            }
            
        }
        //////////////////////////////////////////////////////////////////////////////////////
        // Generate Asterix Messages
        //////////////////////////////////////////////////////////////////////////////////////
        // Manage Asterix messaging
        private void asterix034_timer_Tick(object sender, EventArgs e)
        {
            // This message is sent based on timer settings.
            GenerateAsterix_034();
        }
        /* 
         * 
         * FRN Data Item Data Item Description
Length
in Octets
1 I048/010 Data Source Identifier 2
2 I048/140 Time-of-Day 3
3 I048/020 Target Report Descriptor 1+
4 I048/040 Measured Position in Slant Polar Coordinates 4
5 I048/070 Mode-3/A Code in Octal Representation 2
6 I048/090 Flight Level in Binary Representation 2
7 I048/130 Radar Plot Characteristics 1+1+
FX n.a. Field Extension Indicator n.a.
8 I048/220 Aircraft Address 3
9 I048/240 Aircraft Identification 6
10 I048/250 Mode S MB Data 1+8*n
11 I048/161 Track Number 2
12 I048/042 Calculated Position in Cartesian Coordinates 4
13 I048/200 Calculated Track Velocity in Polar Representation 4
14 I048/170 Track Status 1+
FX n.a. Field Extension Indicator n.a.
15 I048/210 Track Quality 4
16 I048/030 Warning/Error Conditions 1+
17 I048/080 Mode-3/A Code Confidence Indicator 2
18 I048/100 Mode-C Code and Confidence Indicator 4
19 I048/110 Height Measured by 3D Radar 2
20 I048/120 Radial Doppler Speed 1+
21 I048/230 Communications / ACAS Capability and Flight
Status
2
FX n.a. Field Extension Indicator n.a.
22 I048/260 ACAS Resolution Advisory Report 7
23 I048/055 Mode-1 Code in Octal Representation 1
24 I048/050 Mode-2 Code in Octal Representation 2
25 I048/065 Mode-1 Code Confidence Indicator 1
26 I048/060 Mode-2 Code Confidence Indicator 2
27 SP-Data Item Special Purpose Field 1+1+
28 RE-Data Item Reserved Expansion Field 1+1+
FX n.a. Field Extension Indicator n
        */
        private void Asterix048MsgManager()
        {
            // Notes: Wireshark capture only contains Track and End of Track messages.  This algorithm will do the same.
            double maxTimeNoComm_s = 5;  //time with no comm to send endTrack message
            byte[] AstMsg = new byte[3];
            AstMsg[0] = 0x30; //048 message
            int byteCtr = AstMsg.Length;
            int tempCtr = 0;
            asset_status_t target = new asset_status_t();
            uint fspecTrack = 0xe11f8d04;  // use four bytes.  Add zeros to right if necessary
            uint fspecEndTrack = 0xe1120000;  // use four bytes.  Add zeros to right if necessary
            uint sp_spec = 0xffeffc00;  // use four bytes. Add zeros to right if necessary
            
            double timeNow_s = (DateTime.UtcNow -
                Ref1970_Utc).TotalMilliseconds / 1000.0;

            for (int i = 0; i < asset_status_List.Length; i++)
            {
                target = asset_status_List[i];
                if ((timeNow_s - target.radar_report.timeMsgSent_sec > maxTimeNoComm_s) & !target.radar_report.sentEndTrackMessage)
                {
                    target.radar_report.timeMsgSent_sec = (DateTime.UtcNow -
                            Ref1970_Utc).TotalMilliseconds / 1000.0;

                    tempCtr = GenerateAsterix_048(fspecEndTrack, target);

                    target.radar_report.sentEndTrackMessage = true;
                    target.radar_report.trackNumber = 0;  // reset track number so new number is assigned
                    asset_status_List[i] = target;

                    Array.Resize(ref AstMsg, byteCtr + tempCtr);
                    Array.Copy(Asterix048Message, 0, AstMsg, byteCtr, Asterix048Message.Length);
                    byteCtr += Asterix048Message.Length;
                }
                else if (target.EnableTrack & target.radar_report.range_m >= RadarParameter.minRadarRange_m &
                    target.radar_report.range_m <= RadarParameter.maxRadarRange_m & 
                    (minAltToRegisterTarget_m_agl < (target.altitude_m - RadarParameter.elevation_mMSL)))
                {
                    if (target.radar_report.trackNumber == 0)
                    {
                        target.radar_report.trackNumber = trackNumber;
                        trackNumber++;
                    }
                    // send track messages
                    if (target.radar_report.detectCntr == 0)
                        target.radar_report.timeFirstPlot_sec = (DateTime.UtcNow -
                            Ref1970_Utc).TotalMilliseconds / 1000.0;

                    target.radar_report.detectCntr++;
                    target.radar_report.timeMsgSent_sec = (DateTime.UtcNow -
                            Ref1970_Utc).TotalMilliseconds / 1000.0;

                    target.radar_report.sentEndTrackMessage = false;
                    asset_status_List[i] = target;

                    tempCtr = GenerateAsterix_048(fspecTrack, target);
                    tempCtr += GenerateAsterix_048_SpecialField(sp_spec, target);

                    Array.Resize(ref AstMsg, byteCtr + tempCtr);
                    Array.Copy(Asterix048Message, 0, AstMsg, byteCtr, Asterix048Message.Length);
                    byteCtr += Asterix048Message.Length;
                    Array.Copy(AsterixSpFieldMessage, 0, AstMsg, byteCtr, AsterixSpFieldMessage.Length);
                    byteCtr += AsterixSpFieldMessage.Length;

                }
            }
            if (simulateNonTargets)
            {
                asset_status_t nontarget = new asset_status_t();

                for (int i = 0; i < numberOfNonTargets; i++)
                {
                    nontarget = non_target_List[i].asset_info;
                    if ((timeNow_s - nontarget.radar_report.timeMsgSent_sec > maxTimeNoComm_s) & !nontarget.radar_report.sentEndTrackMessage)
                    {
                        nontarget.radar_report.timeMsgSent_sec = (DateTime.UtcNow -
                                Ref1970_Utc).TotalMilliseconds / 1000.0;

                        tempCtr = GenerateAsterix_048(fspecEndTrack, nontarget);

                        nontarget.radar_report.sentEndTrackMessage = true;
                        nontarget.radar_report.trackNumber = 0;  // reset track number so new number is assigned
                        non_target_List[i].asset_info = nontarget;

                        Array.Resize(ref AstMsg, byteCtr + tempCtr);
                        Array.Copy(Asterix048Message, 0, AstMsg, byteCtr, Asterix048Message.Length);
                        byteCtr += Asterix048Message.Length;
                    }
                    else if (nontarget.radar_report.range_m >= RadarParameter.minRadarRange_m &
                        nontarget.radar_report.range_m <= RadarParameter.maxRadarRange_m &
                        (minAltToRegisterTarget_m_agl < Math.Abs(nontarget.altitude_m - RadarParameter.elevation_mMSL)))
                    {
                        if (nontarget.radar_report.trackNumber == 0)
                        {
                            nontarget.radar_report.trackNumber = trackNumber;
                            trackNumber++;
                        }
                        // send track messages
                        if (nontarget.radar_report.detectCntr == 0)
                            nontarget.radar_report.timeFirstPlot_sec = (DateTime.UtcNow -
                                Ref1970_Utc).TotalMilliseconds / 1000.0;

                        nontarget.radar_report.detectCntr++;
                        nontarget.radar_report.timeMsgSent_sec = (DateTime.UtcNow -
                                Ref1970_Utc).TotalMilliseconds / 1000.0;

                        nontarget.radar_report.sentEndTrackMessage = false;
                        non_target_List[i].asset_info = nontarget;

                        tempCtr = GenerateAsterix_048(fspecTrack, nontarget);
                        tempCtr += GenerateAsterix_048_SpecialField(sp_spec, nontarget);

                        Array.Resize(ref AstMsg, byteCtr + tempCtr);
                        Array.Copy(Asterix048Message, 0, AstMsg, byteCtr, Asterix048Message.Length);
                        byteCtr += Asterix048Message.Length;
                        Array.Copy(AsterixSpFieldMessage, 0, AstMsg, byteCtr, AsterixSpFieldMessage.Length);
                        byteCtr += AsterixSpFieldMessage.Length;

                    }
                }
            }
            // fill in length
            byte[] tmpAr = BitConverter.GetBytes(byteCtr);
            if (BitConverter.IsLittleEndian)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 2, AstMsg, 1, 2);
            if(byteCtr>3 & sendRadarMsgCheckBox.Checked)
                udpClient.Send(AstMsg, radarEstimateEndpoint);

            
        }
        // Cat 034
        private void GenerateAsterix_034()
        {
            int[] FieldLength = new int[] { 1, 2, 2, 2, 1, 3, 2, 2, 2, 3, 3 };
            Asterix034Message = new byte[23];
            byte Cat = 0x22;
            byte[] Len = new byte [] {0x00, 0x17};
            byte[] Fspec = new byte[] { 0xED, 0x10 };
            byte[] I010 = new byte[] {RadarParameter.radarParam.SAC, RadarParameter.radarParam.SIC};
            byte I000 = 0x01;  // north marker

            DateTime midnight = new DateTime(DateTime.UtcNow.Year, DateTime.UtcNow.Month, DateTime.UtcNow.Day);
            byte[] I030 = BitConverter.GetBytes((UInt32)Math.Round(((DateTime.UtcNow - midnight).TotalMilliseconds / 1000)) * 128);
            
            short AntRotPeriod = (short)Math.Round(RadarParameter.radarParam.rot_period_s * 128);
            byte[] I041 = BitConverter.GetBytes(AntRotPeriod);
            byte[] I050 = new byte[] { 0x80, 0x04 };  // from example in LStar docs and observation
            byte[] I120Hae = BitConverter.GetBytes((short)Math.Round(RadarParameter.elevation_mMSL));
            double pow_2_23=Math.Pow(2,23);
            double lat = RadarParameter.latitude_deg / 180.0 * pow_2_23;
            byte[] I120Lat = BitConverter.GetBytes((int)Math.Round(lat));
            double lon = RadarParameter.longitude_deg / 180.0 * pow_2_23;
            byte[] I120Long = BitConverter.GetBytes((int)Math.Round(lon));

            // convert to bigendian if needed
            if (BitConverter.IsLittleEndian)
            {
                Array.Reverse(I030);
                Array.Reverse(I041);
                Array.Reverse(I120Hae);
                Array.Reverse(I120Lat);
                Array.Reverse(I120Long);

            }
            // Build message
            int fieldCtr = 0;
            int index = 0;
            Asterix034Message[fieldCtr] = Cat; fieldCtr += FieldLength[index]; index++;
            Len.CopyTo(Asterix034Message, fieldCtr); fieldCtr += FieldLength[index]; index++;
            Fspec.CopyTo(Asterix034Message, fieldCtr); fieldCtr += FieldLength[index]; index++;
            I010.CopyTo(Asterix034Message, fieldCtr); fieldCtr += FieldLength[index]; index++;
            Asterix034Message[fieldCtr] = I000; fieldCtr += FieldLength[index]; index++;
            Array.Copy(I030, 1, Asterix034Message, fieldCtr, 3); fieldCtr += FieldLength[index]; index++;
            I041.CopyTo(Asterix034Message, fieldCtr); fieldCtr += FieldLength[index]; index++;
            I050.CopyTo(Asterix034Message, fieldCtr); fieldCtr += FieldLength[index]; index++;
            I120Hae.CopyTo(Asterix034Message, fieldCtr); fieldCtr += FieldLength[index]; index++;
            Array.Copy(I120Lat, 1, Asterix034Message, fieldCtr, 3); fieldCtr += FieldLength[index];
            Array.Copy(I120Long, 1, Asterix034Message, fieldCtr, 3);
            if(sendRadarMsgCheckBox.Checked)
                udpClient.Send(Asterix034Message, radarEstimateEndpoint);


        }
        // Cat 048
        private int GenerateAsterix_048(uint fspec, asset_status_t target)
        {
            // Msgs used: 010,140,020,040,130,161,042,200,170,210,110,120,sp field
            double NM2meters = 1852;
            double meters2ft = 3.28084;
            byte[] tempField = new byte[37];  // max bytes available
            byte[] tmpAr = new byte[1];
            // FieldLength based on LStar fields used
            int[] FieldLength = new int[] { 1, 1, 3, 1, 2, 2, 1, 1, 2, 2, 2, 2, 2, 2, 1, 1, 1, 1, 2, 1, 2 };

            // determine sp_fspec size (since inputted as uint, significant bytes are always to the left.  Zeros are added to the right for unused bytes.)
            int Fspec_len = 1;
            if ((fspec & 1 << 24) != 0)
            {
                Fspec_len = 2;
                if ((fspec & 1 << 16) != 0)
                {
                    Fspec_len = 3;
                    if ((fspec & 1 << 8) != 0)
                        Fspec_len = 4;
                }
            }

            int fieldCtr = Fspec_len;
            int index = 0;


            if ((fspec & 1 << 31) != 0) //M010
            {
                tempField[fieldCtr] = RadarParameter.radarParam.SAC; fieldCtr += FieldLength[index]; index++;
                tempField[fieldCtr] = RadarParameter.radarParam.SIC; fieldCtr += FieldLength[index]; index++;
            }
            else index += 2;
            if ((fspec & 1 << 30) != 0) //M140
            {
                DateTime midnight = new DateTime(DateTime.UtcNow.Year, DateTime.UtcNow.Month, DateTime.UtcNow.Day);
                tmpAr = BitConverter.GetBytes((UInt32)Math.Round(((DateTime.UtcNow - midnight).TotalMilliseconds / 1000) * 128));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 1, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index++;
            if ((fspec & 1 << 29) != 0) //M020
            {
                tempField[fieldCtr] = 0x20; fieldCtr += FieldLength[index]; index++;
            }
            else index++;
            if ((fspec & 1 << 28) != 0) //M040
            {
                tmpAr = BitConverter.GetBytes((UInt16)Math.Round(target.radar_report.range_m / NM2meters * 256));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                tmpAr = BitConverter.GetBytes((UInt16)Math.Round(target.radar_report.azimuth_deg / 360 * Math.Pow(2, 16)));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index += 2;
            if ((fspec & 1 << 25) != 0) //M130
            {
                tempField[fieldCtr] = 0x08; fieldCtr += FieldLength[index]; index++;  // PSR Amplitude

                tmpAr = BitConverter.GetBytes(RadarParameter.radarParam.PSRamplitude_dBm);
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index+=2;
            if ((fspec & 1 << 24) != 0) // extension bit
            {
                if ((fspec & 1 << 20) != 0) //M161
                {
                    tmpAr = BitConverter.GetBytes(target.radar_report.trackNumber);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 2, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;
                if ((fspec & 1 << 19) != 0) //M042
                {
                    tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.radar_report.x_relative_m / NM2meters * 128));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                    tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.radar_report.y_relative_m / NM2meters * 128));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index += 2;
                if ((fspec & 1 << 18) != 0) //M200
                {
                    tmpAr = BitConverter.GetBytes((UInt16)Math.Round(target.radar_report.gndVel_mps / NM2meters * Math.Pow(2, 14)));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                    tmpAr = BitConverter.GetBytes((UInt16)Math.Round(target.radar_report.heading_deg / 360 * Math.Pow(2, 16)));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index += 2;
                if ((fspec & 1 << 17) != 0) //M170
                {
                    if (fspec == 0xe1120000 | fspec == 0x1130104)  // End of track message

                        tmpAr = BitConverter.GetBytes(0x2790);
                    else
                        tmpAr = BitConverter.GetBytes(0x2710);  // Standard track messages

                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 2, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;
                if ((fspec & 1 << 16) != 0) // extension bit
                {
                    if ((fspec & 1 << 15) != 0) //M210
                    {
                        tmpAr = BitConverter.GetBytes((UInt32)Math.Round(target.radar_report.horStdDev_m / NM2meters * 128));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                        tmpAr = BitConverter.GetBytes((UInt32)Math.Round(target.radar_report.verStdDev_m / NM2meters * 128));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                        tmpAr = BitConverter.GetBytes((UInt32)Math.Round(target.radar_report.GndSpdStdDev_mps / NM2meters * Math.Pow(2, 14)));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                        tmpAr = BitConverter.GetBytes((UInt32)Math.Round(target.radar_report.headingStdDev_deg / 360 * Math.Pow(2, 12)));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index += 4;
                    if ((fspec & 1 << 11) != 0) //M110
                    {
                        tmpAr = BitConverter.GetBytes((UInt16)Math.Round((target.radar_report.z_relative_m + RadarParameter.elevation_mMSL) * meters2ft / 25));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;
                    if ((fspec & 1 << 10) != 0) //M120
                    {
                        tempField[fieldCtr] = 0x80; fieldCtr += FieldLength[index]; index++;  // calculated doppler

                        tmpAr = BitConverter.GetBytes((UInt16)Math.Round(target.radar_report.gndVel_mps / NM2meters * Math.Pow(2, 14)));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index += 2;
                }

            }
            // fill in Fspec
            tmpAr = BitConverter.GetBytes(fspec);
            if (BitConverter.IsLittleEndian)
                Array.Reverse(tmpAr);
            Array.Copy(tmpAr, 0, tempField, 0, Fspec_len);

            Asterix048Message = new byte[fieldCtr];
            Array.Copy(tempField, 0, Asterix048Message, 0, fieldCtr);

            return fieldCtr;
        }

        private int GenerateAsterix_048_SpecialField(uint sp_fspec, asset_status_t target)
        {
            /*
            SP-FSPEC Bit 	Data Item 	Data Field 	Type 	Units 	LSB 
            1 	Radar Serial Number 		uint16 	N/A 	N/A 
            2 	Geo-coordinates 	Latitude 	int32 	Degrees 	0.00001 
		            Longitude 	int32 	Degrees 	0.00001 
            3 	Polar coordinates 	Range 	int32 	Meters 	1 
		            Azimuth 	int16 	Degrees 	0.1 
            4 	Velocity polar coordinates 	Speed 	int16 	Meters/Second 	1 
		            Heading 	int16 	Degrees 	0.1 
            5 	Cartesian coordinates 	X 	int32 	Meters 	1 
		            Y 	int32 	Meters 	1 
		            Z 	int32 	Meters 	1 
            6 	Velocity Cartesian coordinates 	X 	int16 	Meters/Second 	1 
		            Y 	int16 	Meters/Second 	1 
		            Z 	int16 	Meters/Second 	1 
            7 	Altitude 		int32 	Meters 	1 
            8 	Extension indicator 
            9 	Elevation 		int16 	Degrees 	0.1 
            10 	Time of detection 		int32 	Milliseconds (since midnight), UTC time 	0.1 
            11 	Date 	Year 	uint16 	Years 	1 
		            Month 	uint8 	Months 	1 
		            Day 	uint8 	Days 	1 
            12 	RCS 		int16 	dBsm (or binned value) 	0.1 (or 1, if binned) 
            13 	Number detections in track 		uint16 	N/A 	1 
            14 	Track quality 		uint8 	N/A 	N/A 
            15 	Track classification 		uint8 	{ unknown = 0, 
            clutter = 1, 
            aircraft = 2, 
            projectile = 3, 
            bird = 4, 
            ground vehicle = 5, target of non-interest = 6, pending = 7 } 	N/A 
            16 	Extension indicator 
            17 	Time of 1st plot in track 		int32 	Milliseconds (since midnight), UTC time 	0.1 
            18 	Biomass 		int16 	Grams 	1 
            19 	Should cause alert 		uint8 	{ false = 0, true = 1 } 	N/A 
            20 	Time message sent 		int32 	Milliseconds (since midnight), UTC time 	0.1 
            21 	Time message received 		int32 	Milliseconds (since midnight), UTC time 	0.1 
            22 	Classification Confidence 		Uint8 	N/A 	N/A 
            23 	Track Number 		Uint16 	N/A 	N/A 
*/
            // example: sp_fspec=ffeffc  (only version found in captures observations)
            //sp_fspec = 0xffeffc00;
            byte[] tempField = new byte[80];  // max bytes available
            int[] FieldLength = new int[] { 2, 4, 4, 4, 2, 2, 2, 4, 4, 4, 2, 2, 2, 4, 2, 4, 2, 1, 1, 2, 2, 1, 1, 4, 2, 1, 4, 4, 1, 2 };
            byte[] tmpAr = new byte[1];

            // determine sp_fspec size (since inputted as uint, significant bytes are always to the left.  Zeros are added to the right for unused bytes.)
            int Fspec_len = 1;
            if ((sp_fspec & 1 << 24) != 0)
            {
                Fspec_len = 2;
                if ((sp_fspec & 1 << 16) != 0)
                    Fspec_len = 3;
            }

            int fieldCtr = 1 + Fspec_len;
            int index = 0;


            if ((sp_fspec & 1 << 31) != 0) //Radar serial number
            {
                tmpAr = BitConverter.GetBytes(RadarParameter.radarParam.S_N);
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index++;

            if ((sp_fspec & 1 << 30) != 0) //Target geo-coordinates
            {
                // Latitude
                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.latitude_calc_deg * 1e5));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                // Longitude
                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.longitude_calc_deg * 1e5));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

            }
            else index += 2;

            if ((sp_fspec & 1 << 29) != 0) //Polar coordinates
            {
                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.range_m));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.radar_report.azimuth_deg * 10));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index += 2;

            if ((sp_fspec & 1 << 28) != 0) //Horizontal Speed
            {
                double speed = Math.Sqrt(target.velX_mps * target.velX_mps + target.velY_mps * target.velY_mps);
                tmpAr = BitConverter.GetBytes((Int16)Math.Round(speed));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.heading * 10));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index += 2;

            if ((sp_fspec & 1 << 27) != 0) //Cartesian coordinates
            {
                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.x_relative_m));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.y_relative_m));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.z_relative_m));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index += 3;

            if ((sp_fspec & 1 << 26) != 0) //Velocity cartesian coordinates
            {
                tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.velX_mps));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.velY_mps));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.velZ_mps));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index += 3;

            if ((sp_fspec & 1 << 25) != 0) //Altitude
            {
                tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.z_relative_m + RadarParameter.elevation_mMSL));
                if (BitConverter.IsLittleEndian)
                    Array.Reverse(tmpAr);

                Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
            }
            else index++;

            if ((sp_fspec & 1 << 24) != 0) //Continuation bit
            {
                if ((sp_fspec & 1 << 23) != 0) //Elevation
                {
                    tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.radar_report.elevation_deg * 10));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;

                if ((sp_fspec & 1 << 22) != 0) //Time of detection
                {
                    DateTime midnight = new DateTime(DateTime.UtcNow.Year, DateTime.UtcNow.Month, DateTime.UtcNow.Day);
                    double midnightTimeSub_sec = (midnight -
                        Ref1970_Utc).TotalMilliseconds / 1000.0;
                    tmpAr = BitConverter.GetBytes((Int32)Math.Round((target.unix_epoch_time_sec - midnightTimeSub_sec) * 10000));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;

                if ((sp_fspec & 1 << 21) != 0) //Date
                {
                    tmpAr = BitConverter.GetBytes((Int16)DateTime.UtcNow.Year);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                    tmpAr = BitConverter.GetBytes(DateTime.UtcNow.Month);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;

                    tmpAr = BitConverter.GetBytes(DateTime.UtcNow.Day);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index += 3;

                if ((sp_fspec & 1 << 20) != 0) //RCS 
                {
                    tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.radar_report.rcs_dBsm * 10));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;


                if ((sp_fspec & 1 << 19) != 0) //Number of detections
                {
                    tmpAr = BitConverter.GetBytes((UInt16)target.radar_report.detectCntr);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;

                if ((sp_fspec & 1 << 18) != 0) //Track Quality
                {
                    tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.trackQuality));
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;

                if ((sp_fspec & 1 << 17) != 0) //Track classification
                {
                    tmpAr = BitConverter.GetBytes((Int32)target.radar_report.targetClass);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);

                    Array.Copy(tmpAr, 3, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                }
                else index++;

                if ((sp_fspec & 1 << 16) != 0) //Continuation bit
                {
                    if ((sp_fspec & 1 << 15) != 0) //Time of 1st detection
                    {
                        DateTime midnight = new DateTime(DateTime.UtcNow.Year, DateTime.UtcNow.Month, DateTime.UtcNow.Day);
                        double midnightTimeSub_sec = (midnight -
                            Ref1970_Utc).TotalMilliseconds / 1000.0;
                        tmpAr = BitConverter.GetBytes((Int32)Math.Round((target.radar_report.timeFirstPlot_sec - midnightTimeSub_sec) * 10000));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;

                    if ((sp_fspec & 1 << 14) != 0) //biomass
                    {
                        tmpAr = BitConverter.GetBytes((Int16)Math.Round(target.radar_report.biomass));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;

                    if ((sp_fspec & 1 << 13) != 0) //Should alert
                    {
                        tmpAr = BitConverter.GetBytes(target.radar_report.ShouldCauseAlert);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;

                    if ((sp_fspec & 1 << 12) != 0) //Time message sent
                    {
                        DateTime midnight = new DateTime(DateTime.UtcNow.Year, DateTime.UtcNow.Month, DateTime.UtcNow.Day);
                        double midnightTimeSub_sec = (midnight -
                            Ref1970_Utc).TotalMilliseconds / 1000.0;

                        tmpAr = BitConverter.GetBytes((Int32)Math.Round((target.radar_report.timeMsgSent_sec - midnightTimeSub_sec) * 10000));
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;

                    if ((sp_fspec & 1 << 11) != 0) //Time message recv (always send zero)
                    {
                        tmpAr = new byte[4];

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;

                    if ((sp_fspec & 1 << 10) != 0) //Classification Confidence
                    {
                        tmpAr = BitConverter.GetBytes((Int32)Math.Round(target.radar_report.classConfidence));

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }
                    else index++;

                    if ((sp_fspec & 1 << 9) != 0) //Track number
                    {
                        tmpAr = BitConverter.GetBytes((UInt16)target.radar_report.trackNumber);
                        if (BitConverter.IsLittleEndian)
                            Array.Reverse(tmpAr);

                        Array.Copy(tmpAr, 0, tempField, fieldCtr, FieldLength[index]); fieldCtr += FieldLength[index]; index++;
                    }

                    // fill in length
                    tmpAr = BitConverter.GetBytes(fieldCtr);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 3, tempField, 0, 1);
                    // fill in Fspec
                    tmpAr = BitConverter.GetBytes(sp_fspec);
                    if (BitConverter.IsLittleEndian)
                        Array.Reverse(tmpAr);
                    Array.Copy(tmpAr, 0, tempField, 1, Fspec_len);
                    AsterixSpFieldMessage = new byte[fieldCtr];
                    Array.Copy(tempField, 0, AsterixSpFieldMessage, 0, fieldCtr);
                }
            }
            return fieldCtr;

        }

        // determine if UAVs and non-targets are within radar range
        public void UpdateRadarEstimate()
        {
            // Propogate locations of targets based on the latest time available for UAVs
            double latestTime = (DateTime.UtcNow -
                    Ref1970_Utc).TotalMilliseconds / 1000.0;
            foreach (asset_status_t target in asset_status_List)
            {
                if (target.EnableTrack)
                {
                    if (target.unix_epoch_time_sec > latestTime)
                        latestTime = target.unix_epoch_time_sec;
                }
            }
            RadarParameter.utcTime_sec = latestTime;

            // add data
            MathNet.Numerics.Distributions.Normal rangeDist =
                new MathNet.Numerics.Distributions.Normal(0, RadarParameter.range_sigma_m);
            asset_status_t tempTarget = new asset_status_t();
            utmCoord_t uav_position = new utmCoord_t();

            for(int k=0; k<asset_status_List.Count(); k++)
            {
                tempTarget = asset_status_List[k];
                if (tempTarget.EnableTrack)  // target is enabled in simulator
                {
                    uav_position = tempTarget.utmCoord;
                    // propagate position to current time
                    double delTime = latestTime - tempTarget.unix_epoch_time_sec;
                    double tempAlt;
                    uav_position.UTMEasting += delTime * tempTarget.velX_mps;
                    uav_position.UTMNorthing += delTime * tempTarget.velY_mps;
                    tempAlt = tempTarget.altitude_m + delTime * tempTarget.velZ_mps
                        + tempTarget.alt_offset_m;


                    tempTarget.radar_report.range_m = magnitude(uav_position.UTMEasting - RadarParameter.posUtmCoord.UTMEasting,
                        uav_position.UTMNorthing - RadarParameter.posUtmCoord.UTMNorthing,
                        tempAlt - RadarParameter.elevation_mMSL);

                    tempTarget.radar_report.azimuth_deg =  (Math.Round((Math.Atan2(uav_position.UTMEasting - RadarParameter.posUtmCoord.UTMEasting,
                        uav_position.UTMNorthing - RadarParameter.posUtmCoord.UTMNorthing) / Math.PI * 180.0) / RadarParameter.azimuth_beam_width_deg)
                        * RadarParameter.azimuth_beam_width_deg + 360) % 360;  // want angle between 0 and 360 degrees

                    tempTarget.radar_report.elevation_deg = Math.Round((Math.Asin((tempAlt -
                        RadarParameter.elevation_mMSL) / tempTarget.radar_report.range_m) / Math.PI * 180.0) / RadarParameter.elevation_beam_width_deg)
                        * RadarParameter.elevation_beam_width_deg;

                    // determine sigmas
                    double xRel = uav_position.UTMEasting - RadarParameter.posUtmCoord.UTMEasting;
                    double yRel = uav_position.UTMNorthing - RadarParameter.posUtmCoord.UTMNorthing;
                    double zRel = tempAlt - RadarParameter.elevation_mMSL;

                    double rngTrue = magnitude(xRel, yRel, zRel);
                    double azTrue = Math.Atan2(xRel, yRel);
                    double elTrue = Math.Asin(zRel / rngTrue);


                    double xShift = (rngTrue + RadarParameter.range_sigma_m) * Math.Cos(elTrue + RadarParameter.elevation_beam_width_deg / 180 * Math.PI / 12) *
                        Math.Sin(azTrue + RadarParameter.azimuth_beam_width_deg / 180 * Math.PI / 12);
                    double yShift = (rngTrue + RadarParameter.range_sigma_m) * Math.Cos(elTrue + RadarParameter.elevation_beam_width_deg / 180 * Math.PI / 12) *
                        Math.Cos(azTrue + RadarParameter.azimuth_beam_width_deg / 180 * Math.PI / 12);
                    double zShift = (rngTrue + RadarParameter.range_sigma_m) * Math.Sin(elTrue + RadarParameter.elevation_beam_width_deg / 180 * Math.PI / 12);

                    tempTarget.radar_report.horStdDev_m = Math.Sqrt((xShift - xRel) * (xShift - xRel) + (yShift - yRel) * (yShift - yRel));
                    tempTarget.radar_report.verStdDev_m = Math.Abs(zShift - zRel);
                    tempTarget.radar_report.GndSpdStdDev_mps = 0.5;  // guessed value
                    double hdgShift = Math.Atan2(Math.Abs(tempTarget.velX_mps) + Math.Sqrt(tempTarget.radar_report.GndSpdStdDev_mps),
                        Math.Abs(tempTarget.velY_mps) + Math.Sqrt(tempTarget.radar_report.GndSpdStdDev_mps));
                    double hdgR = Math.Atan2(Math.Abs(tempTarget.velX_mps), Math.Abs(tempTarget.velY_mps));
                    tempTarget.radar_report.headingStdDev_deg = Math.Abs(hdgShift - hdgR) / Math.PI * 180;



                    tempTarget.radar_report.range_m += rangeDist.Sample(); // add noise after above calculations

                    tempTarget.radar_report.x_relative_m = tempTarget.radar_report.range_m * Math.Cos(tempTarget.radar_report.elevation_deg / 180.0 * Math.PI)
                        * Math.Sin(tempTarget.radar_report.azimuth_deg / 180.0 * Math.PI);
                    tempTarget.radar_report.y_relative_m = tempTarget.radar_report.range_m * Math.Cos(tempTarget.radar_report.elevation_deg / 180.0 * Math.PI)
                        * Math.Cos(tempTarget.radar_report.azimuth_deg / 180.0 * Math.PI);
                    tempTarget.radar_report.z_relative_m = tempTarget.radar_report.range_m * Math.Sin(tempTarget.radar_report.elevation_deg / 180.0 * Math.PI);
                    tempTarget.radar_report.gndVel_mps = Math.Sqrt(tempTarget.velX_mps * tempTarget.velX_mps + tempTarget.velY_mps * tempTarget.velY_mps);
                    tempTarget.radar_report.heading_deg = (Math.Atan2(tempTarget.velX_mps, tempTarget.velY_mps) / Math.PI * 180 + 360) % 360;

                    utmCoord_t targetEst=RadarParameter.posUtmCoord;
                    targetEst.UTMEasting=RadarParameter.posUtmCoord.UTMEasting+tempTarget.radar_report.x_relative_m;
                    targetEst.UTMNorthing=RadarParameter.posUtmCoord.UTMNorthing+tempTarget.radar_report.y_relative_m;
                    
                    LatLonAltCoord_t solPos = LatLonUtmWgs84Conv.UTMtoLL_Degrees(targetEst);
                    tempTarget.radar_report.latitude_calc_deg = solPos.LatitudeDegrees;
                    tempTarget.radar_report.longitude_calc_deg = solPos.LongitudeDegrees;

                    asset_status_List[k] = tempTarget;
                }
            }
            // calculate range data for non targets
            if (simulateNonTargets)
            {
                asset_status_t tempNonTarget = new asset_status_t();
                for (int k = 0; k < numberOfNonTargets; k++)
                {
                    tempNonTarget = non_target_List[k].asset_info;
                    uav_position = tempNonTarget.utmCoord;
                    // propagate position to current time
                    double delTime = latestTime - tempNonTarget.unix_epoch_time_sec;
                    double tempAlt;
                    uav_position.UTMEasting += delTime * tempNonTarget.velX_mps;
                    uav_position.UTMNorthing += delTime * tempNonTarget.velY_mps;
                    tempAlt = tempNonTarget.altitude_m + delTime * tempNonTarget.velZ_mps;


                    tempNonTarget.radar_report.range_m = magnitude(uav_position.UTMEasting - RadarParameter.posUtmCoord.UTMEasting,
                        uav_position.UTMNorthing - RadarParameter.posUtmCoord.UTMNorthing,
                        tempAlt - RadarParameter.elevation_mMSL);

                    tempNonTarget.radar_report.azimuth_deg = (Math.Round((Math.Atan2(uav_position.UTMEasting - RadarParameter.posUtmCoord.UTMEasting,
                        uav_position.UTMNorthing - RadarParameter.posUtmCoord.UTMNorthing) / Math.PI * 180.0) / RadarParameter.azimuth_beam_width_deg)
                        * RadarParameter.azimuth_beam_width_deg + 360) % 360;  // want angle between 0 and 360 degrees

                    tempNonTarget.radar_report.elevation_deg = Math.Round((Math.Asin((tempAlt -
                        RadarParameter.elevation_mMSL) / tempNonTarget.radar_report.range_m) / Math.PI * 180.0) / RadarParameter.elevation_beam_width_deg)
                        * RadarParameter.elevation_beam_width_deg;

                    // determine sigmas
                    double xRel = uav_position.UTMEasting - RadarParameter.posUtmCoord.UTMEasting;
                    double yRel = uav_position.UTMNorthing - RadarParameter.posUtmCoord.UTMNorthing;
                    double zRel = tempAlt - RadarParameter.elevation_mMSL;

                    double rngTrue = magnitude(xRel, yRel, zRel);
                    double azTrue = Math.Atan2(xRel, yRel);
                    double elTrue = Math.Asin(zRel / rngTrue);


                    double xShift = (rngTrue + RadarParameter.range_sigma_m) * Math.Cos(elTrue + RadarParameter.elevation_beam_width_deg / 180 * Math.PI / 12) *
                        Math.Sin(azTrue + RadarParameter.azimuth_beam_width_deg / 180 * Math.PI / 12);
                    double yShift = (rngTrue + RadarParameter.range_sigma_m) * Math.Cos(elTrue + RadarParameter.elevation_beam_width_deg / 180 * Math.PI / 12) *
                        Math.Cos(azTrue + RadarParameter.azimuth_beam_width_deg / 180 * Math.PI / 12);
                    double zShift = (rngTrue + RadarParameter.range_sigma_m) * Math.Sin(elTrue + RadarParameter.elevation_beam_width_deg / 180 * Math.PI / 12);

                    tempNonTarget.radar_report.horStdDev_m = Math.Sqrt((xShift - xRel) * (xShift - xRel) + (yShift - yRel) * (yShift - yRel));
                    tempNonTarget.radar_report.verStdDev_m = Math.Abs(zShift - zRel);
                    tempNonTarget.radar_report.GndSpdStdDev_mps = 0.5;  // guessed value
                    double hdgShift = Math.Atan2(Math.Abs(tempNonTarget.velX_mps) + Math.Sqrt(tempNonTarget.radar_report.GndSpdStdDev_mps),
                        Math.Abs(tempNonTarget.velY_mps) + Math.Sqrt(tempNonTarget.radar_report.GndSpdStdDev_mps));
                    double hdgR = Math.Atan2(Math.Abs(tempNonTarget.velX_mps), Math.Abs(tempNonTarget.velY_mps));
                    tempNonTarget.radar_report.headingStdDev_deg = Math.Abs(hdgShift - hdgR) / Math.PI * 180;



                    tempNonTarget.radar_report.range_m += rangeDist.Sample(); // add noise after above calculations

                    tempNonTarget.radar_report.x_relative_m = tempNonTarget.radar_report.range_m * Math.Cos(tempNonTarget.radar_report.elevation_deg / 180.0 * Math.PI)
                        * Math.Sin(tempNonTarget.radar_report.azimuth_deg / 180.0 * Math.PI);
                    tempNonTarget.radar_report.y_relative_m = tempNonTarget.radar_report.range_m * Math.Cos(tempNonTarget.radar_report.elevation_deg / 180.0 * Math.PI)
                        * Math.Cos(tempNonTarget.radar_report.azimuth_deg / 180.0 * Math.PI);
                    tempNonTarget.radar_report.z_relative_m = tempNonTarget.radar_report.range_m * Math.Sin(tempNonTarget.radar_report.elevation_deg / 180.0 * Math.PI);
                    tempNonTarget.radar_report.gndVel_mps = Math.Sqrt(tempNonTarget.velX_mps * tempNonTarget.velX_mps + tempNonTarget.velY_mps * tempNonTarget.velY_mps);
                    tempNonTarget.radar_report.heading_deg = (Math.Atan2(tempNonTarget.velX_mps, tempNonTarget.velY_mps) / Math.PI * 180 + 360) % 360;

                    utmCoord_t targetEst = RadarParameter.posUtmCoord;
                    targetEst.UTMEasting = RadarParameter.posUtmCoord.UTMEasting + tempNonTarget.radar_report.x_relative_m;
                    targetEst.UTMNorthing = RadarParameter.posUtmCoord.UTMNorthing + tempNonTarget.radar_report.y_relative_m;

                    LatLonAltCoord_t solPos = LatLonUtmWgs84Conv.UTMtoLL_Degrees(targetEst);
                    tempNonTarget.radar_report.latitude_calc_deg = solPos.LatitudeDegrees;
                    tempNonTarget.radar_report.longitude_calc_deg = solPos.LongitudeDegrees;

                    non_target_List[k].asset_info = tempNonTarget;
                }
            }
            if (asset_status_List[0].EnableTrack)
            {
                uint sp_fspec = 0xffeffc00;
                GenerateAsterix_048_SpecialField(sp_fspec, asset_status_List[0]);
            }
        }

        private double magnitude(double x, double y, double z)
        {
            double mag;
            mag = Math.Sqrt(x * x + y * y + z * z);
            return mag;
        }

        private void enableSerialComm1Checkbox_CheckedChanged(object sender, EventArgs e)
        {
            if (uav1Association != "-")  //only allow serial comm if uav is listed
            {
                if (enableSerialComm1Checkbox.Checked)
                {

                    // first close com thread for udp that was started in initialize servers
                    try
                    {
                        uav1MsgThread.Abort();
                    }
                    catch { }

                    try
                    {
                        uav1MsgThread = new Thread(() => uavStatus1.StartSerialPort(serialCommPort1));  //format used to pass parameters
                        //                udpStartUav1Thread = new Thread(() => uavStatus1.StartUdpServer(0));  //format used to pass parameters
                        uav1MsgThread.Start();
                        Console.WriteLine("Started Uav1 serial thread");
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Error with Uav1 thread:" + ex.Message);
                    }
                }
                else
                {
                    try
                    {
                        uav1MsgThread.Abort();
                    }
                    catch { }
                }
            }
        }

        private void enableSerialComm2Checkbox_CheckedChanged(object sender, EventArgs e)
        {
            if (uav2Association != "-")  //only allow serial comm if uav is listed
            {
                if (enableSerialComm2Checkbox.Checked)
                {

                    // first close com thread for udp that was started in initialize servers
                    try
                    {
                        uav2MsgThread.Abort();
                    }
                    catch { }

                    try
                    {
                        uav2MsgThread = new Thread(() => uavStatus2.StartSerialPort(serialCommPort2));  //format used to pass parameters

                        uav2MsgThread.Start();
                        Console.WriteLine("Started Uav2 serial thread");
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Error with Uav2 thread:" + ex.Message);
                    }
                }
                else
                {
                    try
                    {
                        uav2MsgThread.Abort();
                    }
                    catch { }
                }
            }
        }

        private void enableSerialComm3Checkbox_CheckedChanged(object sender, EventArgs e)
        {
            if (uav3Association != "-")  //only allow serial comm if uav is listed
            {
                if (enableSerialComm3Checkbox.Checked)
                {

                    // first close com thread for udp that was started in initialize servers
                    try
                    {
                        uav3MsgThread.Abort();
                    }
                    catch { }

                    try
                    {
                        uav3MsgThread = new Thread(() => uavStatus3.StartSerialPort(serialCommPort3));  //format used to pass parameters

                        uav3MsgThread.Start();
                        Console.WriteLine("Started Uav3 serial thread");
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Error with Uav3 thread:" + ex.Message);
                    }
                }
                else
                {
                    try
                    {
                        uav3MsgThread.Abort();
                    }
                    catch { }
                }
            }
        }

        private void enableSerialComm4Checkbox_CheckedChanged(object sender, EventArgs e)
        {
            if (uav4Association != "-")  //only allow serial comm if uav is listed
            {
                if (enableSerialComm4Checkbox.Checked)
                {

                    // first close com thread for udp that was started in initialize servers
                    try
                    {
                        uav4MsgThread.Abort();
                    }
                    catch { }

                    try
                    {
                        uav4MsgThread = new Thread(() => uavStatus4.StartSerialPort(serialCommPort4));  //format used to pass parameters

                        uav4MsgThread.Start();
                        Console.WriteLine("Started Uav4 serial thread");
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine("Error with Uav4 thread:" + ex.Message);
                    }
                }
                else
                {
                    try
                    {
                        uav4MsgThread.Abort();
                    }
                    catch { }
                }
            }
        }


        private void serCommPort1TextBox_TextChanged(object sender, EventArgs e)
        {
            serialCommPort1 = serCommPort1TextBox.Text;

        }

        private void serCommPort2TextBox_TextChanged(object sender, EventArgs e)
        {
            serialCommPort2 = serCommPort2TextBox.Text;

        }

        private void serCommPort3TextBox_TextChanged(object sender, EventArgs e)
        {
            serialCommPort3 = serCommPort3TextBox.Text;

        }

        private void serCommPort4TextBox_TextChanged(object sender, EventArgs e)
        {
            serialCommPort4 = serCommPort4TextBox.Text;

        }

        private void simPodMsgsTimer_Tick(object sender, EventArgs e)
        {
            lock (podAssets)
            {
                pod_asset_struct tempPod = podAssets[simPodMsgCntr + 1];
                double currentTime = (DateTime.UtcNow -
                        Ref1970_Utc).TotalMilliseconds / 1000.0;

                if ((tempPod.receivedPodMsg.targetClassification != pod_classification_enum.other) & tempPod.receivedPodMsg.sendTargetMsg)
                {
                    tempPod.receivedPodMsg.msgType = PodMsgType.Target;
                    tempPod.receivedPodMsg.lastHealthMsg_sec = currentTime;
                    byte[] msgOut = outgoingMessage.packPodSimMsg(tempPod, isUdpMessageFormatBigEndian);
                    udpClient.Send(msgOut, podSimMsgEndpoint);
                    tempPod.receivedPodMsg.sendTargetMsg = false;

                }
                else if (((currentTime - tempPod.receivedPodMsg.lastHealthMsg_sec) > 10.0) & tempPod.podHealthy)
                {
                    tempPod.receivedPodMsg.msgType = PodMsgType.PodHealth;
                    tempPod.receivedPodMsg.lastHealthMsg_sec = currentTime;
                    byte[] msgOut = outgoingMessage.packPodSimMsg(tempPod, isUdpMessageFormatBigEndian);
                    udpClient.Send(msgOut, podSimMsgEndpoint);

                }
                podAssets[simPodMsgCntr + 1] = tempPod;

                simPodMsgCntr = (simPodMsgCntr + 1) % podAssets.Count;  //grab a new pod each tick of the timer
            }
        }

        private void sendAcousticPodMsgCheckbox_CheckedChanged(object sender, EventArgs e)
        {
            if (simulatePods)
            {
                if (sendAcousticPodMsgCheckbox.Checked)
                    simPodMsgsTimer.Start();
                else
                    simPodMsgsTimer.Stop();
            }
        }

        private void enableNonTargetsCheckBox_CheckedChanged(object sender, EventArgs e)
        {
            if (enableNonTargetsCheckBox.Checked)
            {
                if (numberOfNonTargets > 0)
                    simulateNonTargets = true;
                else
                    simulateNonTargets = false;
            }
            else
            {
                simulateNonTargets = false;
            }
        }

        private void RadarMinDetRng_m_TextChanged(object sender, EventArgs e)
        {
            if (RadarMinDetRng_m.Text != "")
                RadarParameter.minRadarRange_m = Convert.ToDouble(RadarMinDetRng_m.Text);
        }

        private void RadarMaxDetRng_m_TextChanged(object sender, EventArgs e)
        {
            if (RadarMaxDetRng_m.Text != "")
                RadarParameter.maxRadarRange_m = Convert.ToDouble(RadarMaxDetRng_m.Text);
        }

        private void RangeSigma_m_TextChanged(object sender, EventArgs e)
        {
            if(RangeSigma_m.Text != "")
                RadarParameter.range_sigma_m = Convert.ToDouble(RangeSigma_m.Text);
        }

        private void AzimuthBeamW_deg_TextChanged(object sender, EventArgs e)
        {
            if(AzimuthBeamW_deg.Text != "")
                RadarParameter.azimuth_beam_width_deg = Convert.ToDouble(AzimuthBeamW_deg.Text);
        }

        private void ElevationBeamW_deg_TextChanged(object sender, EventArgs e)
        {
            if(ElevationBeamW_deg.Text != "")
                RadarParameter.elevation_beam_width_deg = Convert.ToDouble(ElevationBeamW_deg.Text);
        }

        private void RadarElev_m_TextChanged(object sender, EventArgs e)
        {
            if(RadarElev_m.Text != "")
                RadarParameter.elevation_mMSL = Convert.ToDouble(RadarElev_m.Text);
        }


    }
    public class Ellipse
    {
        public int PenWidth = 3;
        public Color color;
        public Rectangle rectangle;
        public int sourceType;
    }
    public class PodDisplayColors
    {
        public Color SoloColor = Color.Red;
        public Color RacerColor = Color.Chartreuse;
        public Color idleEllipse = Color.AntiqueWhite;
        public Color noSignal = Color.Gray;
    }

    public enum TargetType
    {
        Solo = 1,
        Racer = 2,
        Vehicle = 3,
        Aircraft = 4,
        Tractor = 5,
        Background = 6,
    }

    public enum PodMsgType
    {
        PodHealth = 0,
        Target = 1,
        Server = 2,
    };

    public enum AcousticSource
    {
        Pod=0,
        Server=1,
    };

    public struct sensor_pod_estimate_struct
    {
        public double currentTargetMeasurement;  // running measurement for target
        public double lastTimeOfTargetDetect_sec; // last timestamp that this target was detected
        public double AoA_deg; //angle of arrival estimate in degrees
    }

    public struct pod_health_struct
    {
        public double battery_volt; // voltage of pod battery
    }

    public struct pod_asset_struct
    {
        public pod_health_struct podHealthParameters;
        public double timestamp_sec;
        public double pos_lat_deg;
        public double pos_long_deg;
        public utmCoord_t utmCoord;
        public double pos_elev_m;
        public sensor_pod_estimate_struct soloSensorEstimates;
        public sensor_pod_estimate_struct racerSensorEstimates;
        public bool podHealthy;
        public bool targetIdSolo;
        public bool targetIdRacer;
        public Color targetColorsSolo;
        public Color targetColorsRacer;
        public double lastSystemMsgTime_sec;
        public double targetSoloRunningResult;
        public double targetRacerRunningResult;
        public int podId; // podNumber
        public Color textColorsSolo;
        public Color textColorsRacer;
        public pod_incoming_msg_struct receivedPodMsg;
        public bool isPodStereo;

    }

    public struct pod_incoming_msg_struct
    {
        public PodMsgType msgType;
        public int podId;
        public double timestamp_sec;
        public double lat_deg;
        public double long_deg;
        public double elevation_mMSL;
        public double battery_volt;
        public pod_classification_enum targetClassification;
        public double aoa_deg;
        public double lastHealthMsg_sec;
        public bool sendTargetMsg;
    }

    public enum pod_classification_enum
    {
        solo=1,
        racingDrone=2,
        vehicle=3,
        airplane=4,
        background=5,
        other=99,
    }

    public struct pod_target_estimate_struct
    {
        public double timestamp_sec;
        public double targetLatitude_deg;
        public double targetLongitude_deg;
        public double targetAltitude_mMSL;
        public double estimateSigma_m;
        public int pod1Id;
        public double pod1Aoa_deg;
        public int pod2Id;
        public double pod2Aoa_deg;
        public int pod3Id;
        public double pod3Aoa_deg;
    }
    public struct aoa_base_data
    {
        public int podId;
        public double pod_pos_lat_deg;
        public double pod_pos_lon_deg;
        public utmCoord_t pod_utmCoord;
        public double pod_pos_alt_mMSL;
        public double pod_aoa;
    }
    public struct map_coordinate
    {
        public double lat_deg;
        public double long_deg;
        public double elev_m;
    }
    public enum mavlinkInterface
    {
        Udp = 0,
        Serial = 1,
    }
    public enum MessageIdentifier
    {
        PodTargetEstimate=0x6e27,
        TrueUavPosition=0xb3e5,
        SimPodMessage=0xe8a1,

    }
}
