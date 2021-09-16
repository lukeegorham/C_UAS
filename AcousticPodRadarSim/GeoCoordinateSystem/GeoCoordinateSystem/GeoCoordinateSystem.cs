/* ****************************************************************
 * Athr: Harry Direen, PhD, PE
 * DireenTech Inc.   (www.DireenTech.com)
 * Original Date: July 21, 2010
 * Updated:  May 20, 2016
 * 
 * Academy Center for UAS Research 
 * Department of Electrical and Computer Engineering 
 * HQ USAFA/DFEC 
 * 2354 Fairchild Drive 
 * USAF Academy, CO 80840-6236 
 * 
 * Desc: This class establishes the phyical/geo coordinate system
 * used by HOPS during a mission.  The coordinate system will be 
 * centered on the mid point of the search area.  The mid point 
 * between the Min and Max latitudes and longitudes will be choosen
 * as (0,0) on an x-y plane.  The x axis will be the east-west axis
 * with east being positive, and west being negative.  The y axis 
 * will be the north-south axis with north being positive and south
 * being negative.  
 * 
 * Distances are measured in meters.
 * Latitude longitude may be provided in degrees or radians.  
 * Positive longitude represent east of the prime meridian, 
 * negative longitude represent west of the prime meridian.
 * Positve latitude is north of the equator
 * and negative latitude is south of the equator.
 * 
 * The GeoCoordinate System will support a linear aproximation to 
 * Lat/Lon to X-Y conversions and supports WGS-84 UTM conversions.
 * If the mission area is relatively small (less that a few kilometers)
 * the linear conversion process should be sufficient and substantially
 * reduces the amount of calculation require for conversions.
 * For larger areas, use the full conversion process... a conversion on 
 * a fast Intel Core i7 is on the order of 1 - 2 microseconds.
 * 
 * STANAG 4586 Notes:
 * All earth-fixed position references shall be expressed in the latitude-longitude 
 * system with respect to the WGS-84 ellipsoid in units of radians using double 
 * precision floating-point numbers. Representations in other systems, such as 
 * Universal Transverse Mercator (UTM), shall be converted at the point of use. 
 * 
 * HOPS Time will follow the STANAG 4586 convention:
 * All times shall be represented in Universal Time Coordinated (UTC) in seconds 
 * since Jan 1, 1970 using IEEE double precision floating point numbers.
 * To ensure consistant time, the UAVTime object must be use.  This
 * system will be synchronized with the GPS system clock.
 * 
 * 
 * The methods in this class are thread safe.  The coordinate system
 * must not be used until it has been set up with misison parameters.
 * Once setup, the coordinate system cannot change during the course 
 * of a mission.
 * 
 *******************************************************************/
using System;
using System.Linq;
using System.Collections.Generic;

namespace GeoCoordinateSystemNS
{

    public enum GeoCoordinateSystemConversionType_e
    {
        /// <summary>
        /// For small areas < +/2 2/5 km
        /// </summary>
        Linear,
             
        /// <summary>
        /// Preferred for Larger Areas.
        /// Based upon WGS84 Conversions relative to a Reference location.
        /// X-Y zero (0,0) is at the provide Lat/Lon Reference location
        /// Does not have issues with Map Boundarys or crossing the equator
        /// </summary>
        WGS84_Relative, 
 
        /// <summary>
        /// Provides X-Y Coordinates that are established by the 
        /// WGS-84 Mapping standards.
        /// Warning!!! There are hugh step changes at map boundaries
        /// and at the equator.  Do not used this conversion if there
        /// are any chances of crossing from one WGS-84 map boundary to another.
        /// For this reason... I highly recommend using the WGS84_Relative option.
        /// </summary>
        WGS84_Map,

    }

    public class GeoCoordinateSystem
    {
        public const double RadiansToDegreesScaleFactor = (180.0 / Math.PI);
	    public const double DegreesToRadiansScaleFactor = (Math.PI / 180.0);
	    public const double PI = Math.PI;
	    public const double HalfPi = Math.PI / 2.0; 
	    public const double TwoPi = 2.0 * Math.PI;
        public const double EarthRadiusAveMeters = 6371000.0;   //Average
        public const double EarthRadiusWGS84Meters = 6378137.0;   //WGS84

        private LatLonAltCoord_t _referenceLocation = new LatLonAltCoord_t();

        /// <summary>
        /// The reference location is the Lat/Lon location
        /// where the X-Y coordintate will be (0, 0)
        /// The Altitude is typically set to the nominial ground
        /// altitude in meters above/below mean sea level.
        /// </summary>
        public LatLonAltCoord_t ReferenceLatLonAltLocation
        {
            get { return _referenceLocation; }
        }

        /// <summary>
        /// The reference location is the Lat/Lon location
        /// where the X-Y coordintate will be (0, 0)
        /// </summary>
        public LatLonCoord_t ReferenceLatLonLocation
        {
            get { return (LatLonCoord_t)_referenceLocation; }
        }

        /// <summary>
        /// The Altitude is typically set to the nominial ground
        /// altitude in meters above/below mean sea level.
        /// This is the same as Height above Height Above 
        /// the WGS-84 Ellipsoid.
        /// </summary>
        public double ReferenceAltitudeMSL
        {
            get { return _referenceLocation.Altitude; }
            set { _referenceLocation.Altitude = value; }
        }

        private int _UTMZoneNumberAtRefLatLon = 0;
        /// <summary>
        /// The is the UTM Zone Number at the Reference
        /// Lat/Lon Location
        /// </summary>
        public int UTMZoneNumberAtRefLatLon
        {
            get { return _UTMZoneNumberAtRefLatLon; }
        }

        private char _UTMZoneLatDesAtRefLatLon = 'N';
        /// <summary>
        /// This is the UTM Zone Latitude Reference Designator
        /// at the Reference Lat/Lon Location
        /// </summary>
        public char UTMZoneLatDesAtRefLatLon
        {
            get { return _UTMZoneLatDesAtRefLatLon; }
        }


        private GeoCoordinateSystemConversionType_e _GeoCoordinateSystemConversionType = GeoCoordinateSystemConversionType_e.WGS84_Map;
        /// <summary>
        /// The GeoCoordinateSystem will work with a number of different
        /// coordinate conversion types... this is the type.
        /// The type cannot be changed without resetting up the GeoCoordinateSystem.
        /// </summary>
        public GeoCoordinateSystemConversionType_e GeoCoordinateSystemConversionType
        {
            get { return _GeoCoordinateSystemConversionType; }
        }


        //Linear Scale Factors for Conversion between Lat/Lon 
        //and X-Y coordinates or X-Y to Lat/Lon
        //Note: inverses of the scale factors are created
        //so that division can be avoided during normal conversions.
        //Division is always more time consuming that multiplication.
        //The defaults below work for Colorado.
        private static double _xToLonCF = 2.007e-7;
        private static double _yToLatCF = 1.573e-7;
        private static double _lonToXCF = 4.982e6;
        private static double _latToYCF = 6.358e6;

        private xyCoord_t _UtmWgs84XYCenter = new xyCoord_t();
        /// <summary>
        ///The UTM X-Y Location of the ReferenceLatLonLocation
        ///For this system the X (or East-West) value will normally be zero.
        //The Y value will be the number of meter from the equator.
        /// </summary>
        public xyCoord_t UtmWgs84XYCenter
        {
            get { return _UtmWgs84XYCenter; }
        }

        private bool _IsCoordinateSystemValid = false;
        /// <summary>
        /// Gets a value indicating whether the coordinate system is valid or not.
        /// The coordinate system cannot be used until setup.
        /// </summary>
        /// <value>
        /// 	<c>true</c> if [coordinate system valid]; otherwise, <c>false</c>.
        /// </value>
        public bool IsCoordinateSystemValid
        {
            get { return _IsCoordinateSystemValid; }
        }

        public GeoCoordinateSystem()
        {
            _IsCoordinateSystemValid = false;
            _GeoCoordinateSystemConversionType = GeoCoordinateSystemConversionType_e.WGS84_Map;
        }

        /// <summary>
        /// Setup a GeoCoordinateSystem
        /// The reference Latitude and Longitude is the location where
        /// the X-Y coordinate will be zero (0,0).
        /// The Alititude is optional.  If provided it establishs the 
        /// nominal ground level at the reference point.
        /// </summary>
        /// <param name="latLonAlt"></param>
        /// <param name="useLinearConversion">if true linear conversion will be setup
        /// and used by default for lat/lon - x/y conversions</param>
        public GeoCoordinateSystem(LatLonAltCoord_t latLonAlt, 
            GeoCoordinateSystemConversionType_e conversionType = GeoCoordinateSystemConversionType_e.WGS84_Map)
        {
            SetupGeoCoordinateSystem(latLonAlt, conversionType);
        }

        /// <summary>
        /// Setup a GeoCoordinateSystem
        /// The reference Latitude and Longitude is the location where
        /// the X-Y coordinate will be zero (0,0).
        /// The Alititude is optional.  If provided it establishs the 
        /// nominal ground level at the reference point.
        /// </summary>
        /// <param name="refLatitude">The Reference Latitude</param>
        /// <param name="refLongitude">The Reference Longitude</param>
        /// <param name="nominalGroundAltitudeMSL">The Nominal Ground Level Altitude in 
        /// meters above mean sea level, or height above the standard elipsoid</param>
        /// <param name="inDegrees"></param>
        /// <param name="conversionType">if true linear conversion will be setup
        /// and used by default for lat/lon - x/y conversions</param>
        public GeoCoordinateSystem(double refLatitude, double refLongitude,
                                    double nominalGroundAltitudeMSL = 0.0, 
                                    bool inDegrees = false,
                                    GeoCoordinateSystemConversionType_e conversionType = GeoCoordinateSystemConversionType_e.WGS84_Map)
        {
            LatLonAltCoord_t latLonAlt = new LatLonAltCoord_t(refLatitude, refLongitude, nominalGroundAltitudeMSL, inDegrees);
            SetupGeoCoordinateSystem(latLonAlt, conversionType);
        }

        /// <summary>
        /// Setup a GeoCoordinateSystem
        /// The reference Latitude and Longitude is the location where
        /// the X-Y coordinate will be zero (0,0).
        /// The Alititude is optional.  If provided it establishs the 
        /// nominal ground level at the reference point.
        /// </summary>
        /// <param name="latLonAlt"></param>
        /// <param name="conversionType"></param>
        public bool SetupGeoCoordinateSystem(LatLonAltCoord_t latLonAlt, 
            GeoCoordinateSystemConversionType_e conversionType = GeoCoordinateSystemConversionType_e.WGS84_Map)
        {
            bool error = false;
            utmCoord_t utmXY;
            _referenceLocation = latLonAlt;
            _GeoCoordinateSystemConversionType = conversionType;
            _UtmWgs84XYCenter.clear();
            switch (conversionType)
            {
                case GeoCoordinateSystemConversionType_e.Linear:
                    List<LatLonCoord_t> latLonList = new List<LatLonCoord_t>();
                    latLonList.Add((LatLonCoord_t)_referenceLocation);
                    SetupGeoCoordinateSystem(latLonList, conversionType);
                    break;

                case GeoCoordinateSystemConversionType_e.WGS84_Relative:
            DefaultCoordSystemSetup:
                    utmXY = LatLonUtmWgs84Conv.LLtoUTM_Degrees(_referenceLocation.LatitudeDegrees,
                                    _referenceLocation.LongitudeDegrees, 0, true, _referenceLocation.LongitudeDegrees);
                    _UtmWgs84XYCenter.x = utmXY.UTMEasting;
                    _UtmWgs84XYCenter.y = utmXY.UTMNorthing;
                    _UTMZoneNumberAtRefLatLon = utmXY.UTMZoneNumber;
                    _UTMZoneLatDesAtRefLatLon = utmXY.UTMZoneLatDes;
                    _IsCoordinateSystemValid = true;
                    break;

                case GeoCoordinateSystemConversionType_e.WGS84_Map:
                    utmXY = LatLonUtmWgs84Conv.LLtoUTM_Degrees(_referenceLocation.LatitudeDegrees,
                                    _referenceLocation.LongitudeDegrees, 0, false, _referenceLocation.LongitudeDegrees);
                    _UtmWgs84XYCenter.x = utmXY.UTMEasting;
                    _UtmWgs84XYCenter.y = utmXY.UTMNorthing;
                    _UTMZoneNumberAtRefLatLon = utmXY.UTMZoneNumber;
                    _UTMZoneLatDesAtRefLatLon = utmXY.UTMZoneLatDes;
                    _IsCoordinateSystemValid = true;
                    break;

                default:
                    _GeoCoordinateSystemConversionType = GeoCoordinateSystemConversionType_e.WGS84_Relative;
                    goto DefaultCoordSystemSetup;
            }
            return error;
        }

        /// <summary>
        /// Setup the GeoCoordinateSystem
        /// Pass in a list of Lat/Lon locations that cover the extent of the region 
        /// the vehicle is expected to operate over.  The center of this region will be used
        /// to set the center of the of the X-Y coordinate system (0,0).
        /// The extent of the region will be used to esblishes the linearization of the 
        /// Lat/Lon to X-Y conversion if useLinearConversion = true;
        /// </summary>
        /// <param name="latLonList"></param>
        /// <param name="useLinearConversion"></param>
        public bool SetupGeoCoordinateSystem(List<LatLonCoord_t> latLonList, 
            GeoCoordinateSystemConversionType_e conversionType = GeoCoordinateSystemConversionType_e.Linear)
        {
            bool error = false;
            double refAlt = _referenceLocation.Altitude;
            if (latLonList.Count < 1)
            {
                return true;     //No valid data
            }
            if (latLonList.Count > 1)
            {
                _referenceLocation = LatLonCoord_t.FindCenterOfSetOfLatLonPoints(latLonList);
            }
            else
            {
                _referenceLocation = latLonList[0];
            }
            //Restore Altitude info lost in the above steps.
            _referenceLocation.Altitude = refAlt;

            //Set the Center UTM Location
            _UtmWgs84XYCenter.clear();
            bool relLatLon = conversionType == GeoCoordinateSystemConversionType_e.WGS84_Map ? false : true;
            utmCoord_t utmXY = LatLonUtmWgs84Conv.LLtoUTM_Degrees(_referenceLocation.LatitudeDegrees,
                            _referenceLocation.LongitudeDegrees, 0, relLatLon, _referenceLocation.LongitudeDegrees);
            _UtmWgs84XYCenter.x = utmXY.UTMEasting;
            _UtmWgs84XYCenter.y = utmXY.UTMNorthing;
            _UTMZoneNumberAtRefLatLon = utmXY.UTMZoneNumber;
            _UTMZoneLatDesAtRefLatLon = utmXY.UTMZoneLatDes;

            if (conversionType == GeoCoordinateSystemConversionType_e.Linear)
            {
                //We temporarily need the convertion type to be WGS84_Relative
                //in order to get various xy and lat/lon points to setup the 
                //linear scale factors... we will set it back to linear after the setup process.
                _GeoCoordinateSystemConversionType = GeoCoordinateSystemConversionType_e.WGS84_Relative;
                LatLonCoord_t maxLatLon;
                LatLonCoord_t minLatLon;
                LatLonCoord_t delLatLon;
                xyCoord_t xyMax;
                xyCoord_t xyMin;
                xyCoord_t delXY;
                if (latLonList.Count < 2)
                {
                    //Generate corner locations... assume +/- 1 km area
                    xyMax = new xyCoord_t(1000.0, 1000.0);
                    maxLatLon = xyToLatLon(xyMax);
                    xyMin = new xyCoord_t(-1000.0, -1000.0);
                    minLatLon = xyToLatLon(xyMin);
                }
                else
                {
                    maxLatLon = LatLonCoord_t.FindMaxNortEastCornerOfSetOfLatLonPoints(latLonList);
                    minLatLon = LatLonCoord_t.FindMinSouthWestCornerOfSetOfLatLonPoints(latLonList);
                    //don't use the full extent... use a partial exent... so that on average
                    //the linear approximation will be good.
                    delLatLon = 0.5 * (maxLatLon - minLatLon);
                    maxLatLon = (LatLonCoord_t)_referenceLocation + 0.75 * delLatLon;
                    minLatLon = (LatLonCoord_t)_referenceLocation - 0.75 * delLatLon;

                    xyMax = LatLonToXY(maxLatLon);
                    xyMin = LatLonToXY(minLatLon);
                    delXY = xyMax - xyMin;
                    if (delXY.x < 100.0 || delXY.y < 100.0)
                    {
                        //Too small of an area..generate a more reasonable size
                        if (delXY.x < 100.0)
                        {
                            xyMax.x = 50.0;
                            xyMin.x = -50.0;
                        }
                        if (delXY.y < 100.0)
                        {
                            xyMax.y = 50.0;
                            xyMin.y = -50.0;
                        }
                        maxLatLon = xyToLatLon(xyMax);
                        minLatLon = xyToLatLon(xyMin);
                    }
                }
                delLatLon = maxLatLon - minLatLon;
                delXY = xyMax - xyMin;
                _xToLonCF = (delLatLon.LongitudeRadians) / delXY.x;
                _lonToXCF = delXY.x / (delLatLon.LongitudeRadians);
                _yToLatCF = (delLatLon.LatitudeRadians) / delXY.y;
                _latToYCF = delXY.y / (delLatLon.LatitudeRadians);
                _GeoCoordinateSystemConversionType = GeoCoordinateSystemConversionType_e.Linear;
            }
            _IsCoordinateSystemValid = true;
            return error;
        }

        /// <summary>
        /// Convert Latitude/Longitude to the X-Y Coordinate system.
        /// The Center of the X-Y Coordinate system is the Reference (Lat/Lon) Location
        /// which is were X-Y is (0,0)
        /// </summary>
        /// <param name="latLon"></param>
        /// <param name="forceUTMTransformation">Optional... normally false.</param>
        /// <returns></returns>
        public xyCoord_t LatLonToXY(LatLonCoord_t latLon)
        {
            xyCoord_t xyPos = new xyCoord_t();
            xyPos.GeoCoordinateSys = this;
            utmCoord_t utmXY;
            switch (GeoCoordinateSystemConversionType)
            {
                case GeoCoordinateSystemConversionType_e.Linear:
                    //Linear Transormation... used for speed of operation.
                    LatLonCoord_t delLatLon = latLon - (LatLonCoord_t)_referenceLocation;
                    xyPos.x = _lonToXCF * delLatLon.LongitudeRadians;
                    xyPos.y = _latToYCF * delLatLon.LatitudeRadians;
                    break;

                case GeoCoordinateSystemConversionType_e.WGS84_Relative:
            LatLonToXYDefaultConversion:
                    utmXY = LatLonUtmWgs84Conv.LLtoUTM_Degrees(latLon.LatitudeDegrees,
                                    latLon.LongitudeDegrees, 0, true, _referenceLocation.LongitudeDegrees);
                    xyPos.x = utmXY.UTMEasting - _UtmWgs84XYCenter.x;
                    xyPos.y = utmXY.UTMNorthing - _UtmWgs84XYCenter.y;
                    break;

                case GeoCoordinateSystemConversionType_e.WGS84_Map:
                    utmXY = LatLonUtmWgs84Conv.LLtoUTM_Degrees(latLon.LatitudeDegrees,
                                    latLon.LongitudeDegrees, 0, false, _referenceLocation.LongitudeDegrees);
                    xyPos.x = utmXY.UTMEasting;
                    xyPos.y = utmXY.UTMNorthing;
                    break;

                default:
                    goto LatLonToXYDefaultConversion;
            }
            return xyPos;
        }

        /// <summary>
        /// Convert Latitude/Longitude Alitude to the X-Y-Z Coordinate system.
        /// The Center of the X-Y Coordinate system is the Reference (Lat/Lon) Location
        /// which is were X-Y is (0,0)
        /// The Z axix is set to the Altitude value.
        /// </summary>
        /// <param name="latLonAlt"></param>
        /// <param name="forceUTMTransformation">>Optional... normally false.</param>
        /// <returns></returns>
        public xyzCoord_t LatLonAltToXYZ(LatLonAltCoord_t latLonAlt)
        {
            xyzCoord_t xyzPos = LatLonToXY((LatLonCoord_t)latLonAlt);
            xyzPos.z = latLonAlt.Altitude;
            xyzPos.GeoCoordinateSys = this;
            return xyzPos;
        }

        /// <summary>
        /// Conver an X-Y coordinate to the Latitude/Longitude Location.
        /// The Center of the X-Y Coordinate system is the Reference (Lat/Lon) Location
        /// which is were X-Y is (0,0)
        /// </summary>
        /// <param name="xyCoord"></param>
        /// <param name="forceUTMTransformation"></param>
        /// <returns></returns>
        public LatLonCoord_t xyToLatLon(xyCoord_t xyCoord)
        {
            LatLonCoord_t latLonPt = new LatLonCoord_t();
            latLonPt.GeoCoordinateSys = this;
            utmCoord_t utmXY;
            switch (GeoCoordinateSystemConversionType)
            {
                case GeoCoordinateSystemConversionType_e.Linear:
                    //Linear Transformation... used for speed of operation.
                    latLonPt.LongitudeRadians= _referenceLocation.LongitudeRadians + _xToLonCF * xyCoord.x;
                    latLonPt.LatitudeRadians = _referenceLocation.LatitudeRadians + _yToLatCF * xyCoord.y;
                    break;

                case GeoCoordinateSystemConversionType_e.WGS84_Relative:
            xyToLatLonDefaultConversion:
                    utmXY = new utmCoord_t();
                    utmXY.UTMEasting = _UtmWgs84XYCenter.x + xyCoord.x;
                    utmXY.UTMNorthing = _UtmWgs84XYCenter.y + xyCoord.y;
                    utmXY.CenterLongitudeDegrees = _referenceLocation.LongitudeDegrees;
                    utmXY.UTMRelativeToCenterLongitude = true;
                    latLonPt = (LatLonCoord_t)LatLonUtmWgs84Conv.UTMtoLL_Degrees(utmXY);
                    break;

                case GeoCoordinateSystemConversionType_e.WGS84_Map:
                    utmXY = new utmCoord_t();
                    utmXY.UTMEasting = xyCoord.x;
                    utmXY.UTMNorthing = xyCoord.y;
                    utmXY.UTMZoneNumber = UTMZoneNumberAtRefLatLon;
                    utmXY.UTMZoneLatDes = UTMZoneLatDesAtRefLatLon;
                    utmXY.UTMRelativeToCenterLongitude = false;
                    latLonPt = (LatLonCoord_t)LatLonUtmWgs84Conv.UTMtoLL_Degrees(utmXY);
                    break;

                default:
                    goto xyToLatLonDefaultConversion;
            }
            return latLonPt;
        }

        /// <summary>
        /// Conver an X-Y-Z coordinate to the Latitude/Longitude Altitude Location.
        /// The Center of the X-Y Coordinate system is the Reference (Lat/Lon) Location
        /// which is were X-Y is (0,0)
        /// The Altitude is set equal to the Z value.
        /// </summary>
        /// <param name="xyzCoord"></param>
        /// <param name="forceUTMTransformation"></param>
        /// <returns></returns>
        public LatLonAltCoord_t xyzToLatLonAlt(xyzCoord_t xyzCoord)
        {
            LatLonAltCoord_t lla = xyToLatLon((xyCoord_t)xyzCoord);
            lla.Altitude = xyzCoord.z;
            lla.GeoCoordinateSys = this;
            return lla;
        }

        /// <summary>
        /// Get the Distance in meters between two Lat/Lon points.  The formula converts 
        /// to the xy cooridiante equivalents and then uses a Euclidian distance between them.
        /// </summary>
        /// <param name="pt1"></param>
        /// <param name="pt2"></param>
        /// <returns></returns>
        public double DistanceBetweenLatLonPointsEuclidian(LatLonCoord_t pt1, LatLonCoord_t pt2)
        {
            xyCoord_t xyPt1 = LatLonToXY(pt1);
            xyCoord_t xyPt2 = LatLonToXY(pt2);
            return xyPt1.Distance(xyPt2);
        }

        /// <summary>
        /// Get the Distance in meters between two Lat/Lon Altitude points.  The formula converts 
        /// to the xyz cooridiante equivalents and then uses a Euclidian distance between them.
        /// </summary>
        /// <param name="pt1"></param>
        /// <param name="pt2"></param>
        /// <returns></returns>
        public double DistanceBetweenLatLonAltPointsEuclidian(LatLonAltCoord_t pt1, LatLonAltCoord_t pt2)
        {
            xyzCoord_t xyPt1 = LatLonAltToXYZ(pt1);
            xyzCoord_t xyPt2 = LatLonAltToXYZ(pt2);
            return xyPt1.Distance(xyPt2);
        }


        /// <summary>
        /// Get the Distance in meters between two Lat/Lon points.  The distance is based 
        /// upon the great circles acrosss the survace of the earth using the Haversine formula
        /// </summary>
        /// <param name="pt1"></param>
        /// <param name="pt2"></param>
        /// <returns></returns>
        public double DistanceBetweenLatLonPointsGreatArc(LatLonCoord_t pt1, LatLonCoord_t pt2)
        {
            return HaversineDistanceBetweenLatLonPointsRad(pt1, pt2);
        }


        //Haversine
        //formula:a = sin²(Δφ/2) + cos φ1 ⋅ cos φ2 ⋅ sin²(Δλ/2)
        //c = 2 ⋅ atan2( √a, √(1−a) )
        //d = R ⋅ c
        //φ is latitude, λ is longitude, R is earth’s radius (mean radius = 6,371km);
        public double HaversineDistanceBetweenLatLonPointsRad(LatLonCoord_t pt1, LatLonCoord_t pt2)
        {
            double dist = 0;
            LatLonCoord_t deltaLatLon = pt2 - pt1;
            double sinSqLat = Math.Sin(0.5 * deltaLatLon.LatitudeRadians);
            sinSqLat = sinSqLat * sinSqLat;
            double sinSqLon = Math.Sin(0.5 * deltaLatLon.LongitudeRadians);
            sinSqLon = sinSqLon * sinSqLon;
            double a = sinSqLat + Math.Cos(pt1.LatitudeRadians) * Math.Cos(pt2.LatitudeRadians) * sinSqLon;
            double c = 2.0 * Math.Atan2( Math.Sqrt(a), Math.Sqrt(1 - a) );
            dist = EarthRadiusAveMeters * c;
            //dist = EarthRadiusWGS84Meters * c;
            return dist;
        }


    }

}
