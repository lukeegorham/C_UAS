/**********************************************************************
 * Athr: Harry Direen, PhD, PE
 * DireenTech Inc.   (www.DireenTech.com)
 * Original Date: July 21, 2010
 * Updated:  Jan. 25, 2016
 * 
 * Academy Center for UAS Research 
 * Department of Electrical and Computer Engineering 
 * HQ USAFA/DFEC 
 * 2354 Fairchild Drive 
 * USAF Academy, CO 80840-6236 
 * 
 * Desc: Convert Latitude and Longitude to UTM coordinates
 * and UTM to Lat/Lon
 * uses the WGS-84 datum.
 * 
 * Source
 * Defense Mapping Agency. 1987b. DMA Technical Report:  
 * Supplement to Department of Defense World Geodetic System
 * 1984 Technical Report. Part I and II. Washington, DC: Defense Mapping Agency
 * 
 * Reference ellipsoids derived from Peter H. Dana's website- 
 * http://www.utexas.edu/depts/grg/gcraft/notes/datum/elist.html
 * Department of Geography, University of Texas at Austin
 * Internet: pdana@mail.utexas.edu
 * 3/22/95
 *  
 * ********************************************************************/
using System;
using System.Collections.Generic;
using System.Text;


namespace GeoCoordinateSystemNS
{
    /// <summary>
    /// This is the UTM X-Y Coordinate
    /// X-Y Coordinate values are in meters.
    /// The Standard UTM Coordinates are based upon center Longitude Zones
    /// that are 6 degrees appart going around the world and positive Northing
    /// starting at zero going north from the equator... but in the southern
    /// hemisphere the South Pole is zero and goes positive to the equator.
    /// This gives a big discontinuity at the equator.
    /// 
    /// If UTMRelativeToCenterLongitude is true, a non-standard Center 
	/// Longitude reference is used.
    /// </summary>
    public struct utmCoord_t
    {
        /// <summary>
        /// If true, the UTMEasting is relative to the CenterLongitude
        /// and UTMNorthing is positive in the Northern Hemisphere and 
        /// Negative in the Southern Hemisphere.
        /// </summary>
        public bool UTMRelativeToCenterLongitude;   
        public double UTMNorthing;          //Meters
        public double UTMEasting;           //Meters
        public int UTMZoneNumber;
        public char UTMZoneLatDes;
		/// <summary>
		/// The altitude in meters as Height Above Earth or the
		/// reference ellipsoid.   Normally this value is zero
		/// and currently is not used except for a pass-through parameter.
		/// </summary>
		public double AltitudeHAE;
        /// <summary>
        /// UTMRelativeToCenterLongitude = true.. this is set by the user to the 
        /// desired Center Longitude.  If UTMRelativeToCenterLongitude = true, 
        /// this is the Center Longitude of the UTMZoneNumber 
        /// </summary>
        public double CenterLongitudeDegrees;

		public void Clear()
		{
			UTMRelativeToCenterLongitude = false;
			UTMNorthing = 0.0;
			UTMEasting = 0.0;
			UTMZoneNumber = 0;
			UTMZoneLatDes = 'N';
			AltitudeHAE = 0.0;
			CenterLongitudeDegrees = 0.0;
		}
    };

    /// <summary>
    ///This is a static class that converts Lat/Lon to UTM - WGS84
    ///coordinates and UTM coordinates to Lat/Lon.  
    ///The WGS84 system is assumed.  Other systems can be used,
    ///simply change the EquatorialRadius and the eccentricitySquared 
    ///constants.
    /// The Standard UTM Coordinates are based upon center Longitude Zones
    /// that are 6 degrees appart going around the world and positive.
    /// Because of this, there are large discontinuities at the boundary
    /// of each UTMZoneNumber.
    /// The Northing coordinate starts at zero at the equator going North,
    /// but in the southern hemisphere the UTMNorthing is 0 at the south-pole
    /// and 10,000,000.00 at the equator.
    /// ... which gives a large discontinuity at the equator.
    /// 
    /// This class with work with the Standar UTM system, but is also 
    /// designed to work relative to a provided Center Longitude.  For the
    /// relative Center Longitude, the center longitude is provided by the user
    /// and Easting is positive for latitudes positive relative to the center long
    /// and negative if (long - center longitude is negative.  Also, the Northing
    /// will be positive going north from the equator and negative going south from
    /// the equator.  This avoids discontinuitys at boundaries.  
    /// Note:  for AbsValues of (Longitude - CenterLongitude) > 3 degrees, the error
    /// may be larger than the standard UTM.  This should not be an issue for any 
    /// reasonalble UAV mission.
    ///  
    /// </summary>
    public class LatLonUtmWgs84Conv
    {
        const double PI = Math.PI;
        const double FOURTHPI = PI / 4;
        const double deg2rad = PI / 180;
        const double rad2deg = 180.0 / PI;
        const double equatorialRadius = 6378137.0;   //WGS84
        const double eccentricitySquared = 0.00669438;  //WGS84
        const double maxWGS84Val = 10000000;    //10 million meters.

        static public char UTMLetterDesignator(double Lat)
        {
            //This routine determines the correct UTM letter designator for the given latitude
            //returns 'Z' if latitude is outside the UTM limits of 84N to 80S
            //Written by Chuck Gantz- chuck.gantz@globalstar.com
            char LetterDesignator;

            if ((84 >= Lat) && (Lat >= 72)) LetterDesignator = 'X';
            else if ((72 > Lat) && (Lat >= 64)) LetterDesignator = 'W';
            else if ((64 > Lat) && (Lat >= 56)) LetterDesignator = 'V';
            else if ((56 > Lat) && (Lat >= 48)) LetterDesignator = 'U';
            else if ((48 > Lat) && (Lat >= 40)) LetterDesignator = 'T';
            else if ((40 > Lat) && (Lat >= 32)) LetterDesignator = 'S';
            else if ((32 > Lat) && (Lat >= 24)) LetterDesignator = 'R';
            else if ((24 > Lat) && (Lat >= 16)) LetterDesignator = 'Q';
            else if ((16 > Lat) && (Lat >= 8)) LetterDesignator = 'P';
            else if ((8 > Lat) && (Lat >= 0)) LetterDesignator = 'N';
            else if ((0 > Lat) && (Lat >= -8)) LetterDesignator = 'M';
            else if ((-8 > Lat) && (Lat >= -16)) LetterDesignator = 'L';
            else if ((-16 > Lat) && (Lat >= -24)) LetterDesignator = 'K';
            else if ((-24 > Lat) && (Lat >= -32)) LetterDesignator = 'J';
            else if ((-32 > Lat) && (Lat >= -40)) LetterDesignator = 'H';
            else if ((-40 > Lat) && (Lat >= -48)) LetterDesignator = 'G';
            else if ((-48 > Lat) && (Lat >= -56)) LetterDesignator = 'F';
            else if ((-56 > Lat) && (Lat >= -64)) LetterDesignator = 'E';
            else if ((-64 > Lat) && (Lat >= -72)) LetterDesignator = 'D';
            else if ((-72 > Lat) && (Lat >= -80)) LetterDesignator = 'C';
            else LetterDesignator = 'Z'; //This is here as an error flag to show that the Latitude is outside the UTM limits

            return LetterDesignator;
        }

        /// <summary>
        /// Compute the UTM Zone number base on the Longitude.
        /// </summary>
        /// <param name="latDeg">The latitude in Degrees</param>
        /// <param name="longDeg">The longitude in Degrees</param>
        /// <returns></returns>
        public static int utmZoneNumber(double latDeg, double longDeg)
        {
            int ZoneNumber;
            //Make sure the longitude is between -180.00 .. 179.9
            double LongTemp = (longDeg + 180) - (int)((longDeg + 180) / 360) * 360 - 180; // -180.00 .. 179.9;
            ZoneNumber = (int)((LongTemp + 180) / 6) + 1;

            if (latDeg >= 56.0 && latDeg < 64.0 && LongTemp >= 3.0 && LongTemp < 12.0)
                ZoneNumber = 32;

            // Special zones for Svalbard
            if (latDeg >= 72.0 && latDeg < 84.0)
            {
                if (LongTemp >= 0.0 && LongTemp < 9.0) ZoneNumber = 31;
                else if (LongTemp >= 9.0 && LongTemp < 21.0) ZoneNumber = 33;
                else if (LongTemp >= 21.0 && LongTemp < 33.0) ZoneNumber = 35;
                else if (LongTemp >= 33.0 && LongTemp < 42.0) ZoneNumber = 37;
            }

            return ZoneNumber;
        }

        //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
        //East Longitudes are positive, West longitudes are negative. 
        //North latitudes are positive, South latitudes are negative
        //Lat and Long are in radians
        //Written by Chuck Gantz- chuck.gantz@globalstar.com
        /// <summary>
        /// Convert Latitude, Longitude to UTM coordinates.
        /// </summary>
        /// <param name="latRadians">The latitude</param>
        /// <param name="longRadians">The longitude</param>
        /// <returns></returns>
		public static utmCoord_t LLtoUTM_Radians(double latRadians, double longRadians, double altHAE = 0.0,
			bool setUTMRelativeToCenterLongitude = false, double centerLongDegrees = 0.0)
        {
			return LLtoUTM_Degrees(latRadians * rad2deg, longRadians * rad2deg, altHAE, setUTMRelativeToCenterLongitude, centerLongDegrees);
        }

        //converts lat/long to UTM coords.  Equations from USGS Bulletin 1532 
        //East Longitudes are positive, West longitudes are negative. 
        //North latitudes are positive, South latitudes are negative
        //Lat and Long are in decimal degrees
        //Written by Chuck Gantz- chuck.gantz@globalstar.com
        /// <summary>
        /// Convert Latitude, Longitude to UTM coordinates.
        /// </summary>
        /// <param name="latDegrees">The latitude</param>
        /// <param name="longDegrees">The longitude</param>
        /// <returns></returns>
		public static utmCoord_t LLtoUTM_Degrees(double latDegrees, double longDegrees, double altHAE = 0.0,
                bool setUTMRelativeToCenterLongitude = false, double centerLongDegrees = 0.0) 
        {
            utmCoord_t utm = new utmCoord_t();
			utm.Clear ();
			utm.AltitudeHAE = altHAE;
	        double a = equatorialRadius;
	        double eccSquared = eccentricitySquared;
	        double k0 = 0.9996;

	        double LongOrigin;
	        double eccPrimeSquared;
	        double N, T, C, A, M;
            double eastingOffset = setUTMRelativeToCenterLongitude ? 0.0 : 500000.0;
            double northingOffset = setUTMRelativeToCenterLongitude ? 0.0 : 10000000.0;

            utm.UTMRelativeToCenterLongitude = setUTMRelativeToCenterLongitude;

            //Make sure the longitude is between -180.00 .. 179.9
            double LongTemp = (longDegrees+180)- (int)((longDegrees+180)/360)*360-180; // -180.00 .. 179.9;

	        double LatRad = latDegrees*deg2rad;
	        double LongRad = LongTemp*deg2rad;
	        double LongOriginRad;
            int ZoneNumber = utmZoneNumber(latDegrees, longDegrees);

            if(setUTMRelativeToCenterLongitude)
            {
                LongOrigin = (centerLongDegrees + 180) - (int)((centerLongDegrees + 180) / 360) * 360 - 180;
            }
            else
            {
                LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone
            }
            utm.CenterLongitudeDegrees = LongOrigin;
            LongOriginRad = LongOrigin * deg2rad;

	        //compute the UTM Zone from the latitude and longitude
            utm.UTMZoneNumber = ZoneNumber;
            utm.UTMZoneLatDes = UTMLetterDesignator(latDegrees);

	        eccPrimeSquared = (eccSquared)/(1-eccSquared);

            N = a / Math.Sqrt(1 - eccSquared * Math.Sin(LatRad) * Math.Sin(LatRad));
            T = Math.Tan(LatRad) * Math.Tan(LatRad);
            C = eccPrimeSquared * Math.Cos(LatRad) * Math.Cos(LatRad);
            A = Math.Cos(LatRad) * (LongRad - LongOriginRad);

	        M = a*((1	- eccSquared/4		- 3*eccSquared*eccSquared/64	- 5*eccSquared*eccSquared*eccSquared/256)*LatRad 
				        - (3*eccSquared/8	+ 3*eccSquared*eccSquared/32	+ 45*eccSquared*eccSquared*eccSquared/1024) * Math.Sin(2*LatRad)
                                            + (15 * eccSquared * eccSquared / 256 + 45 * eccSquared * eccSquared * eccSquared / 1024) * Math.Sin(4 * LatRad)
                                            - (35 * eccSquared * eccSquared * eccSquared / 3072) * Math.Sin(6 * LatRad));
        	
	        utm.UTMEasting = (double)(k0*N*(A+(1-T+C)*A*A*A/6
					        + (5-18*T+T*T+72*C-58*eccPrimeSquared)*A*A*A*A*A/120)
					        + eastingOffset);

	        utm.UTMNorthing = (double)(k0*(M+N*Math.Tan(LatRad)*(A*A/2+(5-T+9*C+4*C*C)*A*A*A*A/24
				         + (61-58*T+T*T+600*C-330*eccPrimeSquared)*A*A*A*A*A*A/720)));

	        if(latDegrees < 0 && !setUTMRelativeToCenterLongitude)
                utm.UTMNorthing += northingOffset; //10000000 meter offset for southern hemisphere
    
            return utm;
        }


        //converts UTM coords to lat/long.  Equations from USGS Bulletin 1532 
        //East Longitudes are positive, West longitudes are negative. 
        //North latitudes are positive, South latitudes are negative
        //Lat and Long are in decimal degrees. 
	    //Written by Chuck Gantz- chuck.gantz@globalstar.com
		public static LatLonAltCoord_t UTMtoLL_Degrees(utmCoord_t utm)
        {
			LatLonAltCoord_t latLon = new LatLonAltCoord_t();
			latLon.Altitude = utm.AltitudeHAE;
	        double k0 = 0.9996;
            double a = equatorialRadius;
            double eccSquared = eccentricitySquared;
	        double eccPrimeSquared;
	        double e1 = (1-Math.Sqrt(1-eccSquared))/(1+Math.Sqrt(1-eccSquared));
	        double N1, T1, C1, R1, D, M;
	        double LongOrigin;
	        double mu, phi1, phi1Rad;
	        double x, y;
	        int ZoneNumber;

            x = utm.UTMEasting;
            y = utm.UTMNorthing;
            LongOrigin = utm.CenterLongitudeDegrees;
            ZoneNumber = utm.UTMZoneNumber;
            if ( !utm.UTMRelativeToCenterLongitude )
            {
                x = utm.UTMEasting - 500000.0; //remove 500,000 meter offset for longitude}
                if (utm.UTMZoneLatDes < 'N')
                {
                    y -= 10000000.0;//remove 10,000,000 meter offset used for southern hemisphere
                }

                LongOrigin = (ZoneNumber - 1) * 6 - 180 + 3;  //+3 puts origin in middle of zone
            }

            eccPrimeSquared = (eccSquared)/(1-eccSquared);

	        M = y / k0;
	        mu = M/(a*(1-eccSquared/4-3*eccSquared*eccSquared/64-5*eccSquared*eccSquared*eccSquared/256));

	        phi1Rad = mu	+ (3*e1/2-27*e1*e1*e1/32)*Math.Sin(2*mu) 
				        + (21*e1*e1/16-55*e1*e1*e1*e1/32)*Math.Sin(4*mu)
				        +(151*e1*e1*e1/96)*Math.Sin(6*mu);
	        phi1 = phi1Rad*rad2deg;

	        N1 = a/Math.Sqrt(1-eccSquared*Math.Sin(phi1Rad)*Math.Sin(phi1Rad));
	        T1 = Math.Tan(phi1Rad)*Math.Tan(phi1Rad);
	        C1 = eccPrimeSquared*Math.Cos(phi1Rad)*Math.Cos(phi1Rad);
	        R1 = a*(1-eccSquared)/Math.Pow(1-eccSquared*Math.Sin(phi1Rad)*Math.Sin(phi1Rad), 1.5);
	        D = x/(N1*k0);

	        latLon.LatitudeRadians = phi1Rad - (N1*Math.Tan(phi1Rad)/R1)*(D*D/2-(5+3*T1+10*C1-4*C1*C1-9*eccPrimeSquared)*D*D*D*D/24
					        +(61+90*T1+298*C1+45*T1*T1-252*eccPrimeSquared-3*C1*C1)*D*D*D*D*D*D/720);

	        latLon.LongitudeRadians = (D-(1+2*T1+C1)*D*D*D/6+(5-2*C1+28*T1-3*C1*C1+8*eccPrimeSquared+24*T1*T1)
					        *D*D*D*D*D/120)/Math.Cos(phi1Rad);

            latLon.LongitudeDegrees += LongOrigin;
            return latLon;
        }

    }
}
