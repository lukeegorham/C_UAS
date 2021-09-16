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
namespace GeoCoordinateSystemNS
{
    /// <summary>
    /// X-Y-Z Coordinate structure.
    /// The z-coordinate is typically used for altitude,
    /// and is optional... it will be set to zero if not used
    /// which keeps it from affecting the X-Y (North/East)
    /// coordinates.
    /// This is a general purpose coordinate structure for doing
    /// math on these items.
    /// </summary>
    public struct xyzCoord_t
    {
        public const double EqualEpslon = 1.0e-10;

        public double x;
        public double y;
        public double z;

        private GeoCoordinateSystem _geoCoordinateSys;
        /// <summary>
        /// GeoCoordinate System is used to conversion of xy to Lat/Lon coordinates.
        /// The GeoCoordinate will be carried through to Lat/Lon Coordinates
        /// for general purpose calculations..
        /// </summary>
		public GeoCoordinateSystem GeoCoordinateSys
		{
			get { return _geoCoordinateSys; }
			set { _geoCoordinateSys = value; }
		}


        public xyzCoord_t(double x_val, double y_val, double z_val = 0.0, GeoCoordinateSystem geoCS = null)
        {
            x = x_val;
            y = y_val;
            z = z_val;
            _geoCoordinateSys = geoCS;
        }

        public xyzCoord_t(xyzCoord_t a)
        {
            x = a.x;
            y = a.y;
            z = a.z;
            _geoCoordinateSys = a.GeoCoordinateSys;
        }

        public xyzCoord_t(xyCoord_t a)
        {
            x = a.x;
            y = a.y;
            z = 0.0;
            _geoCoordinateSys = a.GeoCoordinateSys;
        }

        /// <summary>
        /// Implicit conversion from xyCoord_t to xyzCoord_t
        /// The z-axis value is set to zero.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static implicit operator xyzCoord_t(xyCoord_t xyVec)
        {
            return new xyzCoord_t(xyVec);
        }

        /// <summary>
        /// Implicit conversion from nmCoord_t to xyzCoord_t
        /// The z-axis value is set to zero.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static implicit operator xyzCoord_t(nmCoord_t nmVec)
        {
            return new xyzCoord_t(nmVec.n, nmVec.m);
        }


        public void Clear()
        {
            x = 0.0;
            y = 0.0;
            z = 0.0;
        }

        /// <summary>
        /// Convert the XYZ Coordinate to a Lat/Lon Alt Coordinate.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <returns></returns>
        public LatLonAltCoord_t ToLatLonAltCoordinate()
        {
            LatLonAltCoord_t latLon = new LatLonAltCoord_t();
            if (GeoCoordinateSys != null)
            {
                latLon = GeoCoordinateSys.xyzToLatLonAlt(this);
                latLon.GeoCoordinateSys = GeoCoordinateSys;
            }
            return latLon;
        }


        public static bool operator ==(xyzCoord_t a, xyzCoord_t b)
        {
            xyzCoord_t delxyz = a - b;
            bool equal = Math.Abs(delxyz.x) < EqualEpslon;
            equal &= Math.Abs(delxyz.y) < EqualEpslon;
            equal &= Math.Abs(delxyz.z) < EqualEpslon;
            return equal;
        }

        public static bool operator !=(xyzCoord_t a, xyzCoord_t b)
        {
            xyzCoord_t delxyz = a - b;
            bool equal = Math.Abs(delxyz.x) > EqualEpslon;
            equal |= Math.Abs(delxyz.y) > EqualEpslon;
            equal |= Math.Abs(delxyz.z) > EqualEpslon;
            return equal;
        }

        public override bool Equals(object o)
        {
            try
            {
                return (bool)(this == (xyzCoord_t)o);
            }
            catch
            {
                return false;
            }
        }

        // Override the Object.GetHashCode() method:
        public override int GetHashCode()
        {
            return (int)(x + y + z);
        }


        public static xyzCoord_t operator +(xyzCoord_t a, xyzCoord_t b)
        {
            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
            return new xyzCoord_t(a.x + b.x, a.y + b.y, a.z + b.z, gcs); 
        }

        public static xyzCoord_t operator -(xyzCoord_t a, xyzCoord_t b)
        {
            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
            return new xyzCoord_t(a.x - b.x, a.y - b.y, a.z - b.z, gcs);
        }

        public static xyzCoord_t operator *(xyzCoord_t r, double a)
        {
            return new xyzCoord_t(a * r.x, a * r.y, a * r.z, r.GeoCoordinateSys);
        }

        public static xyzCoord_t operator *(double a, xyzCoord_t r)
        {
            return new xyzCoord_t(a * r.x, a * r.y, a * r.z, r.GeoCoordinateSys);
        }

        public static xyzCoord_t operator /(xyzCoord_t r, double c)
        {
            double a = 1.0;
            try
            {
                a = 1.0 / c;
            }
            catch
            {
                //do something reasonable so we don't have a divide by 
                //zero error.
                if (c >= 0)
                    a = 1.0 / EqualEpslon;
                else
                    a = -1.0 / EqualEpslon;
            }
            return new xyzCoord_t(r.x * a, r.y * a, r.z * a, r.GeoCoordinateSys);
        }


        /// <summary>
        /// Magnitude of this.
        /// </summary>
        /// <returns></returns>
        public double Magnitude()
        {
            return Math.Sqrt(x * x + y * y + z * z);
        }

        /// <summary>
        /// City-Block magnitude.
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double MagnitudeCityBlock()
        {
            double dx = Math.Abs(x);
            double dy = Math.Abs(y);
            double dz = Math.Abs(z);
            double mxy = dx > dy ? dx : dy;
            return mxy > dz ? mxy : dz;
        }

        /// <summary>
        /// Distance between this xyCoord and "a".
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double Distance(xyzCoord_t a)
        {
            double dx = x - a.x;
            double dy = y - a.y;
            double dz = z - a.z;
            return Math.Sqrt(dx * dx + dy * dy + dz * dz);
        }

        /// <summary>
        /// City-Block Distance between this xyCoord and "a".
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double DistanceCityBlock(xyzCoord_t a)
        {
            double dx = Math.Abs(x - a.x);
            double dy = Math.Abs(y - a.y);
            double dz = Math.Abs(z - a.z);
            double mxy = dx > dy ? dx : dy;
            return mxy > dz ? mxy : dz;
        }


        /// <summary>
        /// Calculate dot or inner product
        /// </summary>
        public double InnerProduct(xyzCoord_t b)
        {
            double c= x * b.x + y * b.y + z * b.z;
            return c;
        }

        /// <summary>
        /// The Outer Product of the XY Vector with the a.XYVec.
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double OuterProdXY(xyzCoord_t a)
        {
            return x * a.y - y * a.x;
        }

        //Return a unit vector in the same direction 
        //as this vector.  If the mag of the vector is near zero
        //return a zeroed vector.
        public xyzCoord_t NormalizedVector()
        {
            double d = Magnitude();
            if (d > EqualEpslon)
            {
                return (1.0 / d) * this;
            }
            else
            {
                return new xyzCoord_t(0, 0, 0, GeoCoordinateSys);
            }
        }

        /// <summary>
        /// Rotate the X-Y vector (around the z-axis by theta)
        /// The z-axis is left un-changed.
        /// </summary>
        /// <param name="theta">angle to rotate</param>
        /// <param name="inDegrees">if true, theta is in degrees, otherwize theta is in radians</param>
        /// <returns></returns>
        public xyzCoord_t RotateXYVec(double theta, bool inDegrees = false)
        {
            xyzCoord_t rVec = new xyzCoord_t(this);
            if (inDegrees)
            {
                theta = (Math.PI / 180.0) * theta;
            }
            double a = Math.Cos(theta);
            double b = Math.Sin(theta);
            rVec.x = a * x - b * y;
            rVec.y = b * x + a * y;
            return rVec;
        }

        /// <summary>
        /// Return an angle(radians) relative to the y-axis or North.
        /// </summary>
        /// <returns></returns>
        public double HeadingRadians()
        {
            return Math.Atan2(x, y);
        }

        /// <summary>
        /// Return an angle(radians) relative to the y-axis or North.
        /// </summary>
        /// <returns></returns>
        public double HeadingDegrees()
        {
            return (180.0 / Math.PI) * Math.Atan2(x, y);
        }

        public override string ToString()
        {
            string str = string.Concat("(", x.ToString(), ", ", y.ToString(), ", ", z.ToString(), ")");
            return str;
        }

        public string ToCSV_String()
        {
            return string.Concat(x.ToString(), ",", y.ToString(), ",", z.ToString());
        }

    }


    /// <summary>
    /// The X-Y Coordinate System is in Meters.
    /// Same as the X-Y-Z Coordinate System only for two dimentions.
    /// </summary>
    public struct xyCoord_t
    {
        public const double EqualEpslon = 1.0e-10;

        public double x;
        public double y;

        private GeoCoordinateSystem _geoCoordinateSys;
        /// <summary>
        /// GeoCoordinate System is used to conversion of xy to Lat/Lon coordinates.
        /// The GeoCoordinate will be carried through to Lat/Lon Coordinates
        /// for general purpose calculations..
        /// </summary>
		public GeoCoordinateSystem GeoCoordinateSys
		{
			get { return _geoCoordinateSys; }
			set { _geoCoordinateSys = value; }
		}


        public xyCoord_t(double X, double Y, GeoCoordinateSystem geoCS = null )
        {
            x = X;
            y = Y;
            _geoCoordinateSys = geoCS;
        }

        public xyCoord_t(xyCoord_t a)
        {
            x = a.x;
            y = a.y;
            _geoCoordinateSys = a.GeoCoordinateSys;
        }

        public xyCoord_t(xyzCoord_t a)
        {
            x = a.x;
            y = a.y;
            _geoCoordinateSys = a.GeoCoordinateSys;
        }

        /// <summary>
        /// Implicit conversion from nmCoord_t to xyCoord_t
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static implicit operator xyCoord_t(nmCoord_t nmVec)
        {
            return new xyCoord_t(nmVec.n, nmVec.m);
        }

        /// <summary>
        /// Explicit conversion from xyzCoord_t to xyCoord_t
        /// Allows a cast:  xyCoord_t xyVec = (xyCoord_t)xyzVec
        /// The z-axis value is lost.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static explicit operator xyCoord_t(xyzCoord_t xyzVec)
        {
            return new xyCoord_t(xyzVec);
        }

        public void clear()
        {
            x = 0.0;
            y = 0.0;
        }

        /// <summary>
        /// Convert the XY Coordinate to a Lat/Lon Coordinate.
        /// The GeoCoordinate System must be set for this to work.
        /// </summary>
        /// <returns></returns>
        public LatLonCoord_t ToLatLonCoordinate()
        {
            LatLonCoord_t latLon = new LatLonCoord_t();
            if (GeoCoordinateSys != null)
            {
                latLon = GeoCoordinateSys.xyToLatLon(this);
                latLon.GeoCoordinateSys = GeoCoordinateSys;
            }
            return latLon;
        }

        public static bool operator ==(xyCoord_t a, xyCoord_t b)
        {
            xyCoord_t delxy = a - b;
            bool equal = Math.Abs(delxy.x) < EqualEpslon;
            equal &= Math.Abs(delxy.y) < EqualEpslon;
            return equal;
        }

        public static bool operator !=(xyCoord_t a, xyCoord_t b)
        {
            xyCoord_t delxyz = a - b;
            bool equal = Math.Abs(delxyz.x) > EqualEpslon;
            equal |= Math.Abs(delxyz.y) > EqualEpslon;
            return equal;
        }

        public override bool Equals(object o)
        {
            try
            {
                return (bool)(this == (xyCoord_t)o);
            }
            catch
            {
                return false;
            }
        }

        // Override the Object.GetHashCode() method:
        public override int GetHashCode()
        {
            return (int)(x + y);
        }

        public static xyCoord_t operator +(xyCoord_t a, xyCoord_t b)
        {
            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
            return new xyCoord_t(a.x + b.x, a.y + b.y, gcs); 
        }

        public static xyCoord_t operator -(xyCoord_t a, xyCoord_t b)
        {
            GeoCoordinateSystem gcs = a.GeoCoordinateSys != null ? a.GeoCoordinateSys : b.GeoCoordinateSys;
            return new xyCoord_t(a.x - b.x, a.y - b.y, gcs);
        }

        public static xyCoord_t operator *(xyCoord_t r, double a)
        {
            return new xyCoord_t(a * r.x, a * r.y, r.GeoCoordinateSys);
        }

        public static xyCoord_t operator *(double a, xyCoord_t r)
        {
            return new xyCoord_t(a * r.x, a * r.y, r.GeoCoordinateSys);
        }

        public static xyCoord_t operator /(xyCoord_t r, double c)
        {
            double a = 1.0 / c;
            return new xyCoord_t(r.x * a, r.y * a, r.GeoCoordinateSys);
        }

        /// <summary>
        /// Calculate dot or inner product
        /// </summary>
        public double InnerProduct(xyCoord_t b)
        {
            double c= x * b.x + y * b.y;
            return c;
        }

        /// <summary>
        /// Outer Product value of the two vectors.
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double OuterProd(xyCoord_t a)
        {
            return x * a.y - y * a.x;
        }

        /// <summary>
        /// Magnitude of this.
        /// </summary>
        /// <returns></returns>
        public double Magnitude()
        {
            return Math.Sqrt(x * x + y * y);
        }

        /// <summary>
        /// City-Block magnitude.
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double MagnitudeCityBlock()
        {
            double dx = Math.Abs(x);
            double dy = Math.Abs(y);
            return dx > dy ? dx : dy;
        }

        /// <summary>
        /// Angle in radians from the x-axis
        /// </summary>
        /// <returns></returns>
        public double AngleRadians()
        {
            return Math.Atan2(y, x);
        }

        /// <summary>
        /// Angle in degrees from the x-axis
        /// </summary>
        /// <returns></returns>
        public double AngleDegrees()
        {
            return (180.0 / Math.PI) * Math.Atan2(y, x);
        }

        /// <summary>
        /// Return an angle(radians) relative to the y-axis or North.
        /// Headings are positive angles, clockwise from North [0, 2Pi)
        /// </summary>
        /// <returns></returns>
        public double HeadingRadians()
        {
            double angle = Math.Atan2(x, y);
            if (angle < 0)
                angle += 2.0 * Math.PI;
            return angle;
        }

        /// <summary>
        /// Return an angle(radians) relative to the y-axis or North.
        /// Headings are positive angles, clockwise from North [0, 360.0)
        /// </summary>
        /// <returns></returns>
        public double HeadingDegrees()
        {
            double angle = (180.0 / Math.PI) * Math.Atan2(x, y);
            if (angle < 0)
                angle += 360.0;
            return angle;
        }


        /// <summary>
        /// Distance between this xyCoord and "a".
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double Distance(xyCoord_t a)
        {
            double dx = x - a.x;
            double dy = y - a.y;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// City-Block Distance between this xyCoord and "a".
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double DistanceCityBlock(xyCoord_t a)
        {
            double dx = Math.Abs(x - a.x);
            double dy = Math.Abs(y - a.y);
            return dx > dy ? dx : dy;
        }


        //Return a unit vector in the same direction 
        //as this vector.  If the mag of the vector is near zero
        //return a zeroed vector.
        public xyCoord_t NormalizedVector()
        {
            double d = Magnitude();
            if (d > 0.0000001)
            {
                return (1.0 / d) * this;
            }
            else
            {
                return new xyCoord_t(0, 0, this.GeoCoordinateSys);
            }
        }

        /// <summary>
        /// Rotate the vector by angle theta.
        /// </summary>
        /// <param name="theta">angle to rotate</param>
        /// <param name="inDegrees">if true, theta is in degrees, otherwize theta is in radians</param>
        /// <returns></returns>
        public xyCoord_t RotateXYVec(double theta, bool inDegrees = false)
        {
            xyCoord_t rVec = new xyCoord_t(this);
            if (inDegrees)
            {
                theta = (Math.PI / 180.0) * theta;
            }
            double a = Math.Cos(theta);
            double b = Math.Sin(theta);
            rVec.x = a * x - b * y;
            rVec.y = b * x + a * y;
            return rVec;
        }

        /// <summary>
        /// Returns a number between 1 and 4 for the quadrant the 
        /// vector lies in:
        ///   1 --> x >= 0 && y >= 0
        ///   2 --> x < 0  && y >= 0
        ///   3 --> x < 0 && y < 0
        ///   4 --> x >= 0 && y < 0
        /// </summary>
        /// <returns></returns>
        public int Quadrant()
        {
            if (x >= 0 && y >= 0) return 1;
            if (x < 0 && y >= 0) return 2;
            if (x < 0 && y < 0) return 3;
            return 4;
        }


        public override string ToString()
        {
            string str = string.Concat("(", x.ToString(), ", ", y.ToString(), ")");
            return str;
        }

        public string ToCSV_String()
        {
            return string.Concat(x.ToString(), ",", y.ToString());
        }
    }


    /// <summary>
    /// The Interger version of the X-Y Coordinate System.
    /// This is used by Mission Control Mapping code.
    /// </summary>
    public struct nmCoord_t
    {
        public int n;
        public int m;

        public nmCoord_t(int N, int M)
        {
            n = N;
            m = M;
        }

        public nmCoord_t(nmCoord_t a)
        {
            n = a.n;
            m = a.m;
        }

        /// <summary>
        /// Explicit conversion from xyzCoord_t to nmCoord_t
        /// Allows a cast:  nmCoord_t xyVec = (nmCoord_t)xyzVec
        /// The x-y values are rounded up.
        /// The z-axis value is lost.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static explicit operator nmCoord_t(xyzCoord_t xyzVec)
        {
            return new nmCoord_t((int)(xyzVec.x + 0.5), (int)(xyzVec.y + 0.5));
        }

        /// <summary>
        /// Explicit conversion from xyCoord_t to nmCoord_t
        /// Allows a cast:  nmCoord_t xyVec = (nmCoord_t)xyVec
        /// The x-y values are rounded up.
        /// </summary>
        /// <param name="xyzVec"></param>
        /// <returns></returns>
        public static explicit operator nmCoord_t(xyCoord_t xyVec)
        {
            return new nmCoord_t((int)(xyVec.x + 0.5), (int)(xyVec.y + 0.5));
        }

        public void clear()
        {
            n = 0;
            m = 0;
        }

        public static bool operator ==(nmCoord_t a, nmCoord_t b)
        {
            if (a.n == b.n && a.m == b.m)
                return true;
            else
                return false;
        }

        public static bool operator !=(nmCoord_t a, nmCoord_t b)
        {
            if (a.n != b.n || a.m != b.m)
                return true;
            else
                return false;
        }

        public static nmCoord_t operator +(nmCoord_t a, nmCoord_t b)
        {
            return new nmCoord_t(a.n + b.n, a.m + b.m);
        }

        public static nmCoord_t operator -(nmCoord_t a, nmCoord_t b)
        {
            return new nmCoord_t(a.n - b.n, a.m - b.m);
        }

        public static nmCoord_t operator *(nmCoord_t r, int a)
        {
            return new nmCoord_t(a * r.n, a * r.m);
        }

        public static nmCoord_t operator *(int a, nmCoord_t r)
        {
            return new nmCoord_t(a * r.n, a * r.m);
        }

        public static nmCoord_t operator /(nmCoord_t r, int c)
        {
            return new nmCoord_t(r.n / c, r.m / c);
        }

        /// <summary>
        /// Magnitude of this.
        /// </summary>
        /// <returns></returns>
        public double Magnitude()
        {
            return Math.Sqrt(n * n + m * m);
        }

        /// <summary>
        /// City-Block magnitude.
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public int MagnitudeCityBlock()
        {
            int dx = Math.Abs(n);
            int dy = Math.Abs(m);
            return dx > dy ? dx : dy;
        }

        /// <summary>
        /// Distance between this xyCoord and "a".
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public double Distance(nmCoord_t a)
        {
            double dx = n - a.n;
            double dy = m - a.m;
            return Math.Sqrt(dx * dx + dy * dy);
        }

        /// <summary>
        /// City-Block Distance between this xyCoord and "a".
        /// </summary>
        /// <param name="a"></param>
        /// <returns></returns>
        public int DistanceCityBlock(nmCoord_t a)
        {
            int dx = Math.Abs(n - a.n);
            int dy = Math.Abs(m - a.m);
            return dx > dy ? dx : dy;
        }


        public override bool Equals(object o)
        {
            try
            {
                return (bool)(this == (nmCoord_t)o);
            }
            catch
            {
                return false;
            }
        }

        // Override the Object.GetHashCode() method:
        public override int GetHashCode()
        {
            return (int)(n + m);
        }


        public override string ToString()
        {
            return string.Concat("n=", n.ToString(), " m=", m.ToString());
        }

    }


}
