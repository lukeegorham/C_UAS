/* ****************************************************************
 * Athr: Harry Direen PhD
 * DireenTech Inc.  (www.DireenTech.com)
 * Date: Aug. 2015
 * 
 * Academy Center for UAS Research 
 * Department of Electrical and Computer Engineering 
 * HQ USAFA/DFEC 
 * 2354 Fairchild Drive 
 * USAF Academy, CO 80840-6236 
 * 
 *******************************************************************/
using System;
using System.Collections.Generic;

namespace GeoCoordinateSystemNS
{
	/// <summary>
	/// A Coordinate System for Latitude, Longitude and Altitude
	/// Latitude, Longitude are well defined.
	/// The Altitude is typically in meters above Mean Sea Level
	/// or above the standard elipsoid model... but it could have
	/// different reference points based upon use.
	/// A structure is used to support simple copies and is typically
	/// effecient memory use.
	/// </summary>
	public struct LatLonAltCoord_t
	{
		public const double RadiansToDegreesScaleFactor = (180.0 / Math.PI);
		public const double DegreesToRadiansScaleFactor = (Math.PI / 180.0);
		public const double PI = Math.PI;
		public const double HalfPi = Math.PI / 2.0; 
		public const double TwoPi = 2.0 * Math.PI;
        public const double EqualEpslon = 1.0e-10;

		private double _latRad;

		/// <summary>
		/// Latitude in Radians [-PI/2, PI/2]
		/// </summary>
		/// <value>The latitude radians.</value>
		public double LatitudeRadians
		{
			get { return _latRad; }
			set { _latRad = value < -HalfPi ? -HalfPi : value > HalfPi ? HalfPi : value; }
		}

		/// <summary>
		/// Latitude in Degrees [-90.0, 90.0]
		/// </summary>
		/// <value>The latitude degrees.</value>
		public double LatitudeDegrees
		{
			get { return RadiansToDegreesScaleFactor * _latRad; }
			set { LatitudeRadians = DegreesToRadiansScaleFactor * value; }
		}

		private double _lonRad;

		/// <summary>
		/// Latitude in Radians [-PI, PI)
		/// </summary>
		/// <value>The latitude radians.</value>
		public double LongitudeRadians
		{
			get { return _lonRad; }
			set 
			{ 
				if (value < -PI || value >= PI)
				{
                    value = value % (TwoPi);
                    if (value >= PI)
                        value -= TwoPi;
                    else if (value < -PI)
                        value += TwoPi;
				} 
				_lonRad = value;
			}
		}

		/// <summary>
		/// Latitude in Degrees [-180.0, 180.0)
		/// </summary>
		/// <value>The latitude degrees.</value>
		public double LongitudeDegrees
		{
			get { return RadiansToDegreesScaleFactor * _lonRad; }
			set 
			{ 
				if (value < -180.0 || value >= 180.0)
				{
                    value = value % 360.0;
                    if (value >= 180.0)
                        value -= 360.0;
                    else if (value < -180.0)
                        value += 360.0;
				}
				_lonRad = DegreesToRadiansScaleFactor * value;
			}
		}

		private double _altitude;
		/// <summary>
		/// Altitude in Meters
		/// The Altitude is typically in meters above
		/// Mean SeaLevel or above the standard elipsoid model... 
		/// but it could have different reference points based upon use.
		/// </summary>
		/// <value>The altitude.</value>
		public double Altitude
		{
			get { return _altitude; }
			set { _altitude = value; }
		}

        private GeoCoordinateSystem _geoCoordinateSys;
        /// <summary>
        /// GeoCoordinate System is used to conversion of Lat/Lon to 
        /// xy coordinates and for determining distance between points.
        /// </summary>
		public GeoCoordinateSystem GeoCoordinateSys
		{
			get { return _geoCoordinateSys; }
			set { _geoCoordinateSys = value; }
		}

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="lat"></param>
        /// <param name="lon"></param>
        /// <param name="alt"></param>
        /// <param name="inDegrees">if true, lat and lon are in degrees, otherwise radians</param>
		public LatLonAltCoord_t(double lat, double lon, 
                                double alt = 0, bool inDegrees = false,
                                GeoCoordinateSystem geoCS = null)
		{
			_latRad = 0.0;
			_lonRad = 0.0;
			_altitude = alt;
            _geoCoordinateSys = geoCS;
            if (inDegrees)
            {
                LatitudeDegrees = lat;
                LongitudeDegrees = lon;
            }
            else
            {
                LatitudeRadians = lat;
                LongitudeRadians = lon;
            }
		}

		public LatLonAltCoord_t(LatLonAltCoord_t a)
		{
			_latRad = a._latRad;
			_lonRad = a._lonRad;
			_altitude = a.Altitude;
            _geoCoordinateSys = a.GeoCoordinateSys;
		}

		public LatLonAltCoord_t(LatLonCoord_t a)
		{
			_latRad = a.LatitudeRadians;
			_lonRad = a.LongitudeRadians;
			_altitude = 0.0;
            _geoCoordinateSys = a.GeoCoordinateSys;
		}

        /// <summary>
        /// Implicit conversion from LatLonCoord_t to LatLonAltCoord_t
        /// The altitude value is set to zero.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static implicit operator LatLonAltCoord_t(LatLonCoord_t latLat)
        {
            return new LatLonAltCoord_t(latLat);
        }


		public void clear()
		{
			_latRad = 0.0;
			_lonRad = 0.0;
			_altitude = 0.0;
		}


        public static bool operator ==(LatLonAltCoord_t a, LatLonAltCoord_t b)
        {
            LatLonAltCoord_t delLatLon = a - b;
            bool equal = Math.Abs(delLatLon._latRad) < EqualEpslon;
            equal &= Math.Abs(delLatLon._lonRad) < EqualEpslon;
            equal &= Math.Abs(delLatLon._altitude) < EqualEpslon;
            return equal;
        }

        public static bool operator !=(LatLonAltCoord_t a, LatLonAltCoord_t b)
        {
            LatLonAltCoord_t delLatLon = a - b;
            bool equal = Math.Abs(delLatLon._latRad) > EqualEpslon;
            equal |= Math.Abs(delLatLon._lonRad) > EqualEpslon;
            equal |= Math.Abs(delLatLon._altitude) > EqualEpslon;
            return equal;
        }

        public override bool Equals(object o)
        {
            try
            {
                return (bool)(this == (LatLonAltCoord_t)o);
            }
            catch
            {
                return false;
            }
        }

        // Override the Object.GetHashCode() method:
        public override int GetHashCode()
        {
            return (int)(_latRad + _lonRad + _altitude);
        }

		public static LatLonAltCoord_t operator +(LatLonAltCoord_t a, LatLonAltCoord_t b)
		{
            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
			return new LatLonAltCoord_t(a.LatitudeRadians + b.LatitudeRadians, 
				a.LongitudeRadians + b.LongitudeRadians,
				a.Altitude + b.Altitude, false, gcs);
		}

		public static LatLonAltCoord_t operator -(LatLonAltCoord_t a, LatLonAltCoord_t b)
		{
            //Near the -180 or +180 degree Longitude point we must be careful so
            //that the difference gives us the distance in degrees between the locations
            //Note:  there are no wrap-around the +90 or -90 degree locations for LatitudeRadians.
            double delLon = a.LongitudeRadians - b.LongitudeRadians;
            if (delLon < -PI)
                delLon += TwoPi;
            else if(delLon >= PI) 
                delLon -= TwoPi;

            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
			return new LatLonAltCoord_t(a.LatitudeRadians - b.LatitudeRadians, 
				                            delLon, a.Altitude - b.Altitude, false, gcs);
		}

		/// <summary>
		/// Scale Lat and Lon by the scale factor "a"
		/// The altitude is not scaled as it does not normally make sence to 
		/// scale the altitude by the same factor as lat and lon 
		/// </summary>
		/// <param name="r">The red component.</param>
		/// <param name="a">The alpha component.</param>
		public static LatLonAltCoord_t operator *(LatLonAltCoord_t r, double a)
		{
			return new LatLonAltCoord_t(a * r.LatitudeRadians, a * r.LongitudeRadians, 
                r.Altitude, false, r.GeoCoordinateSys);
		}

		/// <summary>
		/// Scale Lat and Lon by the scale factor "a"
		/// The altitude is not scaled as it does not normally make sence to 
		/// scale the altitude by the same factor as lat and lon 
		/// </summary>
		/// <param name="r">The red component.</param>
		/// <param name="a">The alpha component.</param>
		public static LatLonAltCoord_t operator *(double a, LatLonAltCoord_t r)
		{
			return new LatLonAltCoord_t(a * r.LatitudeRadians, a * r.LongitudeRadians, 
                r.Altitude, false, r.GeoCoordinateSys);
		}

		/// <summary>
		/// Divide Lat and Lon by the factor "c"
		/// The altitude is not scaled as it does not normally make sence to 
		/// scale the altitude by the same factor as lat and lon 
		/// </summary>
		/// <param name="r">The red component.</param>
		/// <param name="a">The alpha component.</param>
		public static LatLonAltCoord_t operator /(LatLonAltCoord_t r, double c)
		{
			double a = 1.0 / c;
			return new LatLonAltCoord_t(r.LatitudeRadians * a, r.LongitudeRadians * a, 
                r.Altitude, false, r.GeoCoordinateSys);
		}


        /// <summary>
        /// Convert to X-Y Coordinates.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <returns></returns>
        public xyzCoord_t ToXYZCoordinate()
        {
            xyzCoord_t xy = new xyCoord_t();
            if (_geoCoordinateSys != null)
            {
                xy = _geoCoordinateSys.LatLonAltToXYZ(this);
                xy.GeoCoordinateSys = _geoCoordinateSys;
            }
            return xy;
        }

        /// <summary>
        /// The Euclidian Distance between this Lat/Lon point and 
        /// the given Lat/Lon point.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <param name="llPt"></param>
        /// <returns></returns>
        public double DistanceEuclidian(LatLonAltCoord_t llPt)
        {
            double dist = 0;
            if (_geoCoordinateSys != null)
            {
                dist = _geoCoordinateSys.DistanceBetweenLatLonAltPointsEuclidian(this, llPt);
            }
            return dist;
        }

        /// <summary>
        /// Add a offset in meters to the Lat/Lon Position.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <param name="xyOffset"></param>
        /// <returns></returns>
        public LatLonAltCoord_t AddXYZOffset(xyzCoord_t xyzOffset)
        {
            LatLonAltCoord_t latLonPos = new LatLonCoord_t(this);
            if( _geoCoordinateSys != null)
            {
                xyzCoord_t xyzPos = _geoCoordinateSys.LatLonAltToXYZ(this);
                xyzPos = xyzPos + xyzOffset;
                latLonPos = _geoCoordinateSys.xyzToLatLonAlt(xyzPos);
            }
            return latLonPos;
        }

		public override string ToString()
		{
			return string.Format("Latitude(Deg)={0}, Longitude(Deg)={1}, Altitude={2}", 
				LatitudeDegrees, LongitudeDegrees, Altitude);
		}

		public string ToCSV_String()
		{
			return string.Concat(LatitudeDegrees.ToString(), ",", LongitudeDegrees.ToString(), ",", Altitude.ToString());
		}
	}


    public struct LatLonCoord_t
    {
		public const double RadiansToDegreesScaleFactor = (180.0 / Math.PI);
		public const double DegreesToRadiansScaleFactor = (Math.PI / 180.0);
		public const double PI = Math.PI;
		public const double HalfPi = Math.PI / 2.0; 
		public const double TwoPi = 2.0 * Math.PI;
        public const double EqualEpslon = 1.0e-10;

		private double _latRad;

		/// <summary>
		/// Latitude in Radians [-PI/2, PI/2]
		/// </summary>
		/// <value>The latitude radians.</value>
		public double LatitudeRadians
		{
			get { return _latRad; }
			set { _latRad = value < -HalfPi ? -HalfPi : value > HalfPi ? HalfPi : value; }
		}

		/// <summary>
		/// Latitude in Degrees [-90.0, 90.0]
		/// </summary>
		/// <value>The latitude degrees.</value>
		public double LatitudeDegrees
		{
			get { return RadiansToDegreesScaleFactor * _latRad; }
			set { LatitudeRadians = DegreesToRadiansScaleFactor * value; }
		}

		private double _lonRad;

		/// <summary>
		/// Latitude in Radians [-PI, PI)
		/// </summary>
		/// <value>The latitude radians.</value>
		public double LongitudeRadians
		{
			get { return _lonRad; }
			set 
			{ 
				if (value < -PI || value >= PI)
				{
                    value = value % (TwoPi);
                    if (value >= PI)
                        value -= TwoPi;
                    else if (value < -PI)
                        value += TwoPi;
				} 
				_lonRad = value;
			}
		}

		/// <summary>
		/// Latitude in Degrees [-180.0, 180.0)
		/// </summary>
		/// <value>The latitude degrees.</value>
		public double LongitudeDegrees
		{
			get { return RadiansToDegreesScaleFactor * _lonRad; }
			set 
			{ 
				if (value < -180.0 || value >= 180.0)
				{
                    value = value % 360.0;
                    if (value >= 180.0)
                        value -= 360.0;
                    else if (value < -180.0)
                        value += 360.0;
				}
				_lonRad = DegreesToRadiansScaleFactor * value;
			}
		}

        private GeoCoordinateSystem _geoCoordinateSys;
        /// <summary>
        /// GeoCoordinate System is used to conversion of Lat/Lon to 
        /// xy coordinates and for determining distance between points.
        /// </summary>
		public GeoCoordinateSystem GeoCoordinateSys
		{
			get { return _geoCoordinateSys; }
			set { _geoCoordinateSys = value; }
		}


        public LatLonCoord_t(double lat, double lon, 
                            bool inDegrees = false,
                            GeoCoordinateSystem geoCS = null)
		{
			_latRad = 0.0;
			_lonRad = 0.0;
            _geoCoordinateSys = geoCS;
            if (inDegrees)
            {
                LatitudeDegrees = lat;
                LongitudeDegrees = lon;
            }
            else
            {
                LatitudeRadians = lat;
                LongitudeRadians = lon;
            }
		}

        public LatLonCoord_t(LatLonCoord_t a)
        {
            _latRad = a._latRad;
            _lonRad = a._lonRad;
            _geoCoordinateSys = a.GeoCoordinateSys;
        }

        public LatLonCoord_t(LatLonAltCoord_t a)
        {
            _latRad = a.LatitudeRadians;
            _lonRad = a.LongitudeRadians;
            _geoCoordinateSys = a.GeoCoordinateSys;
        }

        /// <summary>
        /// Explicit conversion from LatLonAltCoord_t to LatLonCoord_t
        /// Allows a cast:  LatLonCoord_t latLon = (LatLonCoord_t)latLonAlt
        /// The altitude value is lost.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static explicit operator LatLonCoord_t(LatLonAltCoord_t latLonAlt)
        {
            return new LatLonCoord_t(latLonAlt);
        }


        public void clear()
        {
            _latRad = 0.0;
            _lonRad = 0.0;
        }

        public static bool operator ==(LatLonCoord_t a, LatLonCoord_t b)
        {
            LatLonCoord_t delLatLon = a - b;
            bool equal = Math.Abs(delLatLon._latRad) < EqualEpslon;
            equal &= Math.Abs(delLatLon._lonRad) < EqualEpslon;
            return equal;
        }

        public static bool operator !=(LatLonCoord_t a, LatLonCoord_t b)
        {
            LatLonCoord_t delLatLon = a - b;
            bool equal = Math.Abs(delLatLon._latRad) > EqualEpslon;
            equal |= Math.Abs(delLatLon._lonRad) > EqualEpslon;
            return equal;
        }

        public static LatLonCoord_t operator +(LatLonCoord_t a, LatLonCoord_t b)
        {
            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
            return new LatLonCoord_t(a._latRad + b._latRad, a._lonRad + b._lonRad, false, gcs);
        }

		public static LatLonCoord_t operator -(LatLonCoord_t a, LatLonCoord_t b)
		{
            //Near the -180 or +180 degree Longitude point we must be careful so
            //that the difference gives us the distance in degrees between the locations
            //Note:  there are no wrap-around the +90 or -90 degree locations for LatitudeRadians.
            double delLon = a.LongitudeRadians - b.LongitudeRadians;
            if (delLon < -PI)
                delLon += TwoPi;
            else if(delLon >= PI) 
                delLon -= TwoPi;

            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
			return new LatLonCoord_t(a.LatitudeRadians - b.LatitudeRadians, delLon, false, gcs);
		}

        public static LatLonCoord_t operator *(LatLonCoord_t r, double a)
        {
            return new LatLonCoord_t(a * r._latRad, a * r._lonRad, false, r.GeoCoordinateSys);
        }

        public static LatLonCoord_t operator *(double a, LatLonCoord_t r)
        {
            return new LatLonCoord_t(a * r._latRad, a * r._lonRad, false, r.GeoCoordinateSys);
        }

        public static LatLonCoord_t operator /(LatLonCoord_t r, double c)
        {
            double a = 1.0 / c;
            return new LatLonCoord_t(r._latRad * a, r._lonRad * a, false, r.GeoCoordinateSys);
        }


        public override bool Equals(object o)
        {
            try
            {
                return (bool)(this == (LatLonCoord_t)o);
            }
            catch
            {
                return false;
            }
        }

        // Override the Object.GetHashCode() method:
        public override int GetHashCode()
        {
            return (int)(_latRad + _lonRad);
        }


        /// <summary>
        /// Find the Maximum North-East Corner of a set of Latitude-Longitude values.
        /// It is assumed  Latitude values are in the range: [-Pi/2, Pi/2] or [-90.0, 90.0]
        /// and Longitude values are in the range: [-Pi, Pi) or [-180.0, 180.0)
        /// </summary>
        /// <param name="latLonList"></param>
        /// <returns></returns>
        public static LatLonCoord_t FindMaxNortEastCornerOfSetOfLatLonPoints(List<LatLonCoord_t> latLonList)
        {
            LatLonCoord_t maxLatLon = new LatLonCoord_t();
            double maxLatRad = -double.MaxValue;
            double maxLonRad = -double.MaxValue;
            bool posLon = false;
            bool negLon = false;
            GeoCoordinateSystem gcs = null;
            foreach (LatLonCoord_t latLon in latLonList)            
            {
                if (latLon.GeoCoordinateSys != null) gcs = latLon.GeoCoordinateSys;
                maxLatRad = Math.Max(latLon.LatitudeRadians, maxLatRad);
                maxLonRad = Math.Max(latLon.LongitudeRadians, maxLonRad);
                if (latLon.LongitudeRadians < 0)
                    negLon = true;
                else
                    posLon = true;
            }
            if (posLon && negLon && maxLonRad > 0.5 * Math.PI)
            {
                //We have points on both sides of the 180 degree Longitude...
                //We must go back and choose the smalled largest negative Longitude.
                maxLonRad = -double.MaxValue;
                foreach (LatLonCoord_t latLon in latLonList)
                {
                    if (latLon.LongitudeRadians < 0)
                        maxLonRad = Math.Max(latLon.LongitudeRadians, maxLonRad);
                }
            }
            maxLatLon.LatitudeRadians = maxLatRad;
            maxLatLon.LongitudeRadians = maxLonRad >= Math.PI ? Math.PI - 1.0e-12 : maxLonRad;
            maxLatLon.GeoCoordinateSys = gcs;
            return maxLatLon;
        }

        /// <summary>
        /// Find the Minimum South-West Corner of a set of Latitude-Longitude values.
        /// It is assumed  Latitude values are in the range: [-Pi/2, Pi/2] or [-90.0, 90.0]
        /// and Longitude values are in the range: [-Pi, Pi) or [-180.0, 180.0)
        /// </summary>
        /// <param name="latLonList"></param>
        /// <returns></returns>
        public static LatLonCoord_t FindMinSouthWestCornerOfSetOfLatLonPoints(List<LatLonCoord_t> latLonList)
        {
            LatLonCoord_t minLatLon = new LatLonCoord_t();
            double minLatRad = double.MaxValue;
            double minLonRad = double.MaxValue;
            bool posLon = false;
            bool negLon = false;
            GeoCoordinateSystem gcs = null;
            foreach (LatLonCoord_t latLon in latLonList)            
            {
                if (latLon.GeoCoordinateSys != null) gcs = latLon.GeoCoordinateSys;
                minLatRad = Math.Min(latLon.LatitudeRadians, minLatRad);
                minLonRad = Math.Min(latLon.LongitudeRadians, minLonRad);
                if (latLon.LongitudeRadians < 0)
                    negLon = true;
                else
                    posLon = true;
            }
            if (posLon && negLon && minLonRad < -0.5 * Math.PI)
            {
                //We have points on both sides of the 180 degree Longitude...
                //We must go back and choose the smalled positive Longitude.
                minLonRad = double.MaxValue;
                foreach (LatLonCoord_t latLon in latLonList)
                {
                    if (latLon.LongitudeRadians > 0)
                        minLonRad = Math.Min(latLon.LongitudeRadians, minLonRad);
                }
            }
            minLatLon.LatitudeRadians = minLatRad;
            minLatLon.LongitudeRadians = minLonRad < -Math.PI ? -Math.PI : minLonRad;
            minLatLon.GeoCoordinateSys = gcs;
            return minLatLon;
        }


        /// <summary>
        /// Find the Center of a set of Latitude-Longitude values.
        /// It is assumed  Latitude values are in the range: [-Pi/2, Pi/2] or [-90.0, 90.0]
        /// and Longitude values are in the range: [-Pi, Pi) or [-180.0, 180.0)
        /// The method handles cases where there is a cross over at -180 degress Longitude
        /// </summary>
        /// <param name="latLonList"></param>
        /// <returns></returns>
        public static LatLonCoord_t FindCenterOfSetOfLatLonPoints(List<LatLonCoord_t> latLonList)
        {
            LatLonCoord_t maxLatLon = FindMaxNortEastCornerOfSetOfLatLonPoints(latLonList);
            LatLonCoord_t minLatLon = FindMinSouthWestCornerOfSetOfLatLonPoints(latLonList);
            LatLonCoord_t delLatLon = 0.5 * (maxLatLon - minLatLon);
            LatLonCoord_t centerLatLon = minLatLon + delLatLon;
            return centerLatLon;
        }

        /// <summary>
        /// Convert to X-Y Coordinates.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <returns></returns>
        public xyCoord_t ToXYCoordinate()
        {
            xyCoord_t xy = new xyCoord_t();
            if (_geoCoordinateSys != null)
            {
                xy = _geoCoordinateSys.LatLonToXY(this);
                xy.GeoCoordinateSys = _geoCoordinateSys;
            }
            return xy;
        }

        /// <summary>
        /// The Euclidian Distance between this Lat/Lon point and 
        /// the given Lat/Lon point.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <param name="llPt"></param>
        /// <returns></returns>
        public double DistanceEuclidian(LatLonCoord_t llPt)
        {
            double dist = 0;
            if (_geoCoordinateSys != null)
            {
                dist = _geoCoordinateSys.DistanceBetweenLatLonPointsEuclidian(this, llPt);
            }
            return dist;
        }

        /// <summary>
        /// The Great Arc Distance between this Lat/Lon point and 
        /// the given Lat/Lon point.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <param name="llPt"></param>
        /// <returns></returns>
        public double DistanceGreatArc(LatLonCoord_t llPt)
        {
            double dist = 0;
            if (_geoCoordinateSys != null)
            {
                dist = _geoCoordinateSys.DistanceBetweenLatLonPointsGreatArc(this, llPt);
            }
            return dist;
        }

        /// <summary>
        /// Add a offset in meters to the Lat/Lon Position.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <param name="xyOffset"></param>
        /// <returns></returns>
        public LatLonCoord_t AddXYOffset(xyCoord_t xyOffset)
        {
            LatLonCoord_t latLonPos = new LatLonCoord_t(this);
            if( _geoCoordinateSys != null)
            {
                xyCoord_t xyPos = _geoCoordinateSys.LatLonToXY(this);
                xyPos = xyPos + xyOffset;
                latLonPos = _geoCoordinateSys.xyToLatLon(xyPos);
            }
            return latLonPos;
        }

        public override string ToString()
        {
            return string.Format("Latitude(Deg)={0}, Longitude(Deg)={1}", LatitudeDegrees, LongitudeDegrees);
        }

        public string ToCSV_String()
        {
            return string.Concat(LatitudeDegrees.ToString(), ",", LongitudeDegrees.ToString());
        }
    }


}

