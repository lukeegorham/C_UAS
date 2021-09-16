using NUnit.Framework;
using System;
using System.Collections.Generic;
using GeoCoordinateSystemNS;

namespace GeoCoordinateSystemUnitTests
{
    [TestFixture ()]
    class GeoCoordinateSystemTests
    {

        [Test()]
        public void GeoCoordinateSystemWGS84RelativeTest1()
        {
            xyCoord_t xyPos1;
            xyCoord_t xyPos2;
            LatLonCoord_t latLonPos1;
            LatLonCoord_t latLonPos2;
            xyCoord_t delXY;
            LatLonCoord_t delLatLon;

            GeoCoordinateSystem gcs = new GeoCoordinateSystem(40.0, -85.0, 2200.0, 
                            true, GeoCoordinateSystemConversionType_e.WGS84_Relative);
            Assert.IsTrue(gcs.IsCoordinateSystemValid);
            Assert.AreEqual(gcs.ReferenceAltitudeMSL, 2200.0);
            Assert.AreEqual(gcs.ReferenceLatLonLocation.LatitudeDegrees, 40.0, 1.0e-9);
            Assert.AreEqual(gcs.ReferenceLatLonLocation.LongitudeDegrees, -85.0, 1.0e-9);
            Assert.AreEqual(gcs.UtmWgs84XYCenter.x, 0.0, 1.0e-9);
            Assert.AreEqual(gcs.UtmWgs84XYCenter.y, 4427757.2188447, 1.0e-6);
            Assert.AreEqual(gcs.UTMZoneNumberAtRefLatLon, 16);
            Assert.AreEqual(gcs.UTMZoneLatDesAtRefLatLon, 'T');

            latLonPos1 = new LatLonCoord_t(40.015, -85.025, true);
            xyPos1 = gcs.LatLonToXY(latLonPos1);
            //Expected Position:
            xyPos2 = new xyCoord_t(-2133.52548409202, 4429422.37357421 - 4427757.2188447);
            delXY = xyPos2 - xyPos1;
            double delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 0.0001);

            latLonPos2 = gcs.xyToLatLon(xyPos1);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);

            latLonPos1 = new LatLonCoord_t(39.75, -84.723, true);
            xyPos1 = gcs.LatLonToXY(latLonPos1);
            //Expected Position:
            xyPos2 = new xyCoord_t(23730.6556828957, 4400046.9448536 - 4427757.2188447);
            delXY = xyPos2 - xyPos1;
            delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 0.0001);

            latLonPos2 = gcs.xyToLatLon(xyPos1);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);
        }

        [Test()]
        public void GeoCoordinateSystemLinearTest1()
        {
            xyCoord_t xyPos1;
            xyCoord_t xyPos2;
            LatLonCoord_t latLonPos1;
            LatLonCoord_t latLonPos2;
            xyCoord_t delXY;
            LatLonCoord_t delLatLon;

            GeoCoordinateSystem gcsRef = new GeoCoordinateSystem(40.0, -85.0, 2200.0, 
                            true, GeoCoordinateSystemConversionType_e.WGS84_Relative);

            GeoCoordinateSystem gcsLin = new GeoCoordinateSystem(40.0, -85.0, 2200.0, 
                            true, GeoCoordinateSystemConversionType_e.Linear);

            Assert.IsTrue(gcsLin.IsCoordinateSystemValid);
            Assert.AreEqual(gcsLin.ReferenceAltitudeMSL, 2200.0);
            Assert.AreEqual(gcsLin.ReferenceLatLonLocation.LatitudeDegrees, 40.0, 1.0e-9);
            Assert.AreEqual(gcsLin.ReferenceLatLonLocation.LongitudeDegrees, -85.0, 1.0e-9);
            Assert.AreEqual(gcsLin.UtmWgs84XYCenter.x, 0.0, 1.0e-9);
            Assert.AreEqual(gcsLin.UtmWgs84XYCenter.y, 4427757.2188447, 1.0e-6);
            Assert.AreEqual(gcsLin.UTMZoneNumberAtRefLatLon, 16);
            Assert.AreEqual(gcsLin.UTMZoneLatDesAtRefLatLon, 'T');

            latLonPos1 = new LatLonCoord_t(40.015, -85.025, true);
            xyPos1 = gcsLin.LatLonToXY(latLonPos1);
            xyPos2 = gcsRef.LatLonToXY(latLonPos1);
            delXY = xyPos2 - xyPos1;
            double delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 1.0);

            latLonPos2 = gcsLin.xyToLatLon(xyPos1);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);

            latLonPos1 = new LatLonCoord_t(39.958, -84.967, true);
            xyPos1 = gcsLin.LatLonToXY(latLonPos1);
            xyPos2 = gcsRef.LatLonToXY(latLonPos1);
            delXY = xyPos2 - xyPos1;
            delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 2.5);

            latLonPos2 = gcsLin.xyToLatLon(xyPos1);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);
        }

        [Test()]
        public void GeoCoordinateSystemLinearTest2()
        {
            xyCoord_t xyPos1;
            xyCoord_t xyPos2;
            LatLonCoord_t latLonPos1;
            LatLonCoord_t latLonPos2;
            xyCoord_t delXY;
            LatLonCoord_t delLatLon;

            GeoCoordinateSystem gcs = new GeoCoordinateSystem();
            List<LatLonCoord_t> latLonList = new List<LatLonCoord_t>();
            latLonList.Add(new LatLonCoord_t(39.957, -84.975, true));
            latLonList.Add(new LatLonCoord_t(39.973, -84.987, true));
            latLonList.Add(new LatLonCoord_t(40.028, -84.973, true));
            latLonList.Add(new LatLonCoord_t(40.015, -85.0315, true));
            latLonList.Add(new LatLonCoord_t(39.987, -85.015, true));

            gcs.ReferenceAltitudeMSL = 2200.0;
            gcs.SetupGeoCoordinateSystem(latLonList, GeoCoordinateSystemConversionType_e.Linear);

            Assert.IsTrue(gcs.IsCoordinateSystemValid);
            Assert.AreEqual(gcs.ReferenceAltitudeMSL, 2200.0);
            Assert.AreEqual(gcs.ReferenceLatLonLocation.LatitudeDegrees, 39.9925, 1.0e-3);
            Assert.AreEqual(gcs.ReferenceLatLonLocation.LongitudeDegrees, -85.00225, 1.0e-3);
            Assert.AreEqual(gcs.UtmWgs84XYCenter.x, 0.0, 1.0e-9);
            Assert.AreEqual(gcs.UtmWgs84XYCenter.y, 4426924.792744, 1.0e-3);

            GeoCoordinateSystem gcsRef = new GeoCoordinateSystem(gcs.ReferenceLatLonAltLocation, 
                            GeoCoordinateSystemConversionType_e.WGS84_Relative);

            latLonPos1 = new LatLonCoord_t(40.015, -85.025, true);
            xyPos1 = gcsRef.LatLonToXY(latLonPos1);
            xyPos2 = gcs.LatLonToXY(latLonPos1);
            delXY = xyPos2 - xyPos1;
            double delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 1.0);

            latLonPos2 = gcs.xyToLatLon(xyPos2);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);

            latLonPos2 = new LatLonCoord_t(39.965, -84.975, true);
            double distLinE = gcs.DistanceBetweenLatLonAltPointsEuclidian(latLonPos1, latLonPos2);
            double distUTME = gcsRef.DistanceBetweenLatLonAltPointsEuclidian(latLonPos1, latLonPos2);
            double distHav = gcsRef.DistanceBetweenLatLonPointsGreatArc(latLonPos1, latLonPos2);

            Assert.AreEqual(distLinE, 7001.19062, 0.001);
            Assert.AreEqual(distLinE, distUTME, 0.25);
            Assert.AreEqual(distUTME, distHav, 5.0);

        }

        [Test()]
        public void GeoCoordinateSystemWGS84MapTest()
        {
            xyCoord_t xyPos1;
            xyCoord_t xyPos2;
            LatLonCoord_t latLonPos1;
            LatLonCoord_t latLonPos2;
            xyCoord_t delXY;
            LatLonCoord_t delLatLon;

            GeoCoordinateSystem gcs = new GeoCoordinateSystem(40.0, -85.0, 2200.0, 
                            true, GeoCoordinateSystemConversionType_e.WGS84_Map);
            Assert.IsTrue(gcs.IsCoordinateSystemValid);
            Assert.AreEqual(gcs.ReferenceAltitudeMSL, 2200.0);
            Assert.AreEqual(gcs.ReferenceLatLonLocation.LatitudeDegrees, 40.0, 1.0e-9);
            Assert.AreEqual(gcs.ReferenceLatLonLocation.LongitudeDegrees, -85.0, 1.0e-9);
            Assert.AreEqual(gcs.UtmWgs84XYCenter.x, 670725.49427421, 1.0e-3);
            Assert.AreEqual(gcs.UtmWgs84XYCenter.y, 4429672.97322188, 1.0e-3);
            Assert.AreEqual(gcs.UTMZoneNumberAtRefLatLon, 16);
            Assert.AreEqual(gcs.UTMZoneLatDesAtRefLatLon, 'T');

            latLonPos1 = new LatLonCoord_t(40.015, -85.025, true);
            xyPos1 = gcs.LatLonToXY(latLonPos1);
            //Expected Position:
            xyPos2 = new xyCoord_t(668554.363571232, 4431290.3957099);
            delXY = xyPos2 - xyPos1;
            double delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 0.0001);

            latLonPos2 = gcs.xyToLatLon(xyPos1);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);

            latLonPos1 = new LatLonCoord_t(39.75, -84.723, true);
            xyPos1 = gcs.LatLonToXY(latLonPos1);
            //Expected Position:
            xyPos2 = new xyCoord_t(695080.457955338, 4402489.67779715);
            delXY = xyPos2 - xyPos1;
            delErr = delXY.Magnitude();
            Assert.IsTrue(delErr < 0.0001);

            latLonPos2 = gcs.xyToLatLon(xyPos1);
            delLatLon = latLonPos2 - latLonPos1;
            Assert.IsTrue(delLatLon.LatitudeDegrees < 0.00001);
            Assert.IsTrue(delLatLon.LongitudeDegrees < 0.00001);
        }


    }
}
