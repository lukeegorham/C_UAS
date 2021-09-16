using NUnit.Framework;
using System;
using GeoCoordinateSystemNS;

namespace GeoCoordinateSystemUnitTests
{
    [TestFixture ()]
    class LatLonUtmWgs84ConvTests
    {

        [Test()]
        public void LLtoUTM_DegreesTests()
        {
            //Compare Lat/Lon to UTM based on an on-line calculator
            //http://www.earthpoint.us/Convert.aspx

            //40.0, -85.0 = 16T 670725mE 4429672mN
            utmCoord_t xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Degrees(40.0, -85.0);

            Assert.AreEqual((int)xyCoord1.UTMEasting, 670725);
            Assert.AreEqual((int)xyCoord1.UTMNorthing, 4429672);
            Assert.AreEqual(xyCoord1.UTMZoneLatDes, 'T');
            Assert.AreEqual(xyCoord1.UTMZoneNumber, 16);

            xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Degrees(60.0, -75.0);
            //18V 500000mE 6651411mN
            Assert.AreEqual((int)xyCoord1.UTMEasting, 500000);
            Assert.AreEqual((int)xyCoord1.UTMNorthing, 6651411);
            Assert.AreEqual(xyCoord1.UTMZoneLatDes, 'V');
            Assert.AreEqual(xyCoord1.UTMZoneNumber, 18);

            xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Degrees(-30.0, 45.0);
            //38J 500000mE 6681214mN
            Assert.AreEqual((int)xyCoord1.UTMEasting, 500000);
            Assert.AreEqual((int)xyCoord1.UTMNorthing, 6681214);
            Assert.AreEqual(xyCoord1.UTMZoneLatDes, 'J');
            Assert.AreEqual(xyCoord1.UTMZoneNumber, 38);

            xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Radians(10.0 * (Math.PI / 180.0), -66.0* (Math.PI / 180.0));
            //	20P 171071mE 1106908mN
            Assert.AreEqual((int)xyCoord1.UTMEasting, 171071);
            Assert.AreEqual((int)xyCoord1.UTMNorthing, 1106908);
            Assert.AreEqual(xyCoord1.UTMZoneLatDes, 'P');
            Assert.AreEqual(xyCoord1.UTMZoneNumber, 20);
        }

        [Test()]
        public void UTMtoLL_DegreesTests()
        {
            utmCoord_t xyCoord1 = new utmCoord_t();
            xyCoord1.UTMEasting = 670725;
            xyCoord1.UTMNorthing = 4429672;
            xyCoord1.UTMZoneLatDes = 'T';
            xyCoord1.UTMZoneNumber = 16;
            LatLonAltCoord_t llvals = LatLonUtmWgs84Conv.UTMtoLL_Degrees(xyCoord1);

            Assert.AreEqual(llvals.LatitudeDegrees, 40.0, 0.0001);
            Assert.AreEqual(llvals.LongitudeDegrees, -85.0, 0.0001);

            xyCoord1 = new utmCoord_t();
            xyCoord1.UTMEasting = 500000;
            xyCoord1.UTMNorthing = 6681214;
            xyCoord1.UTMZoneLatDes = 'J';
            xyCoord1.UTMZoneNumber = 38;
            llvals = LatLonUtmWgs84Conv.UTMtoLL_Degrees(xyCoord1);

            Assert.AreEqual(llvals.LatitudeDegrees, -30.0, 0.0001);
            Assert.AreEqual(llvals.LongitudeDegrees, 45.0, 0.0001);

            xyCoord1 = new utmCoord_t();
            xyCoord1.UTMEasting = 171071;
            xyCoord1.UTMNorthing = 1106908;
            xyCoord1.UTMZoneLatDes = 'P';
            xyCoord1.UTMZoneNumber = 20;
            llvals = LatLonUtmWgs84Conv.UTMtoLL_Degrees(xyCoord1);

            Assert.AreEqual(llvals.LatitudeDegrees, 10.0, 0.0001);
            Assert.AreEqual(llvals.LongitudeDegrees, -66.0, 0.0001);
        }

        [Test()]
        public void UTMRelativeCoordTests()
        {
            utmCoord_t xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Degrees(40.0, -85.0, 2200.0, true, -84.5);
            LatLonAltCoord_t llvals = LatLonUtmWgs84Conv.UTMtoLL_Degrees(xyCoord1);
            Assert.AreEqual(llvals.LatitudeDegrees, 40.0, 0.0001);
            Assert.AreEqual(llvals.LongitudeDegrees, -85.0, 0.0001);

            xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Degrees(1.5, 85.0, 2200.0, true, 84.5);
            llvals = LatLonUtmWgs84Conv.UTMtoLL_Degrees(xyCoord1);
            Assert.AreEqual(llvals.LatitudeDegrees, 1.5, 0.0001);
            Assert.AreEqual(llvals.LongitudeDegrees, 85.0, 0.0001);

            xyCoord1 = LatLonUtmWgs84Conv.LLtoUTM_Degrees(-1.5, 84.0, 2200.0, true, 84.5);
            llvals = LatLonUtmWgs84Conv.UTMtoLL_Degrees(xyCoord1);
            Assert.AreEqual(llvals.LatitudeDegrees, -1.5, 0.0001);
            Assert.AreEqual(llvals.LongitudeDegrees, 84.0, 0.0001);
        }


    }
}
