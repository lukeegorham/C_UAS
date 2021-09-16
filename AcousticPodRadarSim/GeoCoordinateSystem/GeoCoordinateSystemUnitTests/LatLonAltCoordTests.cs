using NUnit.Framework;
using System;
using System.Collections.Generic;
using GeoCoordinateSystemNS;

namespace GeoCoordinateSystemUnitTests
{
    [TestFixture ()]
    class LatLonAltCoordTests
    {

		[Test ()]
		public void LLAConstructorTest()
		{
            LatLonAltCoord_t lla1 = new LatLonAltCoord_t(Math.PI / 4.0, Math.PI / 3.0, 1250.0);
            Assert.AreEqual(lla1.LatitudeDegrees, 180.0/4.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeDegrees, 180.0/3.0, 1e-10);
            Assert.AreEqual(lla1.Altitude, 1250.0, 1e-10);

            LatLonAltCoord_t lla2 = new LatLonAltCoord_t(lla1);
            Assert.AreEqual(lla1.LatitudeRadians, lla2.LatitudeRadians, 1e-10);
            Assert.AreEqual(lla1.LongitudeRadians, lla2.LongitudeRadians, 1e-10);
            Assert.AreEqual(lla1.Altitude, lla2.Altitude, 1e-10);

            lla1.LatitudeDegrees = 30.0;
            lla1.LongitudeDegrees = -60.0;
            Assert.AreEqual(lla1.LatitudeRadians, Math.PI / 6.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeRadians, -Math.PI / 3.0, 1e-10);

            lla1.LatitudeDegrees = 100.0;
            lla1.LongitudeDegrees = 190.0;

            Assert.AreEqual(lla1.LatitudeDegrees, 90.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeDegrees, -170.0, 1e-10);

            lla1.LatitudeDegrees = -100.0;
            lla1.LongitudeDegrees = -190.0;

            Assert.AreEqual(lla1.LatitudeDegrees, -90.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeDegrees, 170.0, 1e-10);

            lla1.LongitudeRadians = -Math.PI - Math.PI / 18.0;
            Assert.AreEqual(lla1.LongitudeDegrees, 170.0, 1e-10);

            lla1.LongitudeRadians = Math.PI + Math.PI / 18.0;
            Assert.AreEqual(lla1.LongitudeDegrees, -170.0, 1e-10);

            LatLonCoord_t latLon = (LatLonCoord_t)lla1;
            Assert.AreEqual(latLon.LatitudeDegrees, lla1.LatitudeDegrees, 1e-10);
            Assert.AreEqual(latLon.LongitudeDegrees, lla1.LongitudeDegrees, 1e-10);

            LatLonAltCoord_t llc = latLon;
            Assert.AreEqual(latLon.LatitudeDegrees, llc.LatitudeDegrees, 1e-10);
            Assert.AreEqual(latLon.LongitudeDegrees, llc.LongitudeDegrees, 1e-10);
            Assert.AreEqual(llc.Altitude, 0.0);
		}

        [Test()]
        public void LLAAddSubTest()
        {
            LatLonAltCoord_t lla1 = new LatLonAltCoord_t(30.0, 45.0, 1000.0, true);
            LatLonAltCoord_t lla2 = new LatLonAltCoord_t(10.0, 25.0, 250.0, true);
            
            LatLonAltCoord_t llaplus = lla1 + lla2;
            Assert.AreEqual(llaplus.LatitudeDegrees, 40.0, 1e-10);
            Assert.AreEqual(llaplus.LongitudeDegrees, 70.0, 1e-10);
            Assert.AreEqual(llaplus.Altitude, 1250.0, 1e-10);
 
            LatLonAltCoord_t llaminus = lla1 - lla2;
            Assert.AreEqual(llaminus.LatitudeDegrees, 20.0, 1e-10);
            Assert.AreEqual(llaminus.LongitudeDegrees, 20.0, 1e-10);
            Assert.AreEqual(llaminus.Altitude, 750.0, 1e-10);
       
            lla2 = new LatLonAltCoord_t(-15.0, 150.0, 333.3, true);
            llaplus = lla1 + lla2;
            Assert.AreEqual(llaplus.LatitudeDegrees, 15.0, 1e-10);
            Assert.AreEqual(llaplus.LongitudeDegrees, -165.0, 1e-10);
            Assert.AreEqual(llaplus.Altitude, 1333.3, 1e-10);

            lla1 = new LatLonAltCoord_t(30.0, -170.0, 1000.0, true);
            lla2 = new LatLonAltCoord_t(-15.0, 150.0, -355.5, true);
            llaplus = lla1 - lla2;
            Assert.AreEqual(llaplus.LatitudeDegrees, 45.0, 1e-10);
            Assert.AreEqual(llaplus.LongitudeDegrees, 40.0, 1e-10);
            Assert.AreEqual(llaplus.Altitude, 1355.5, 1e-10);
 
            lla1 = new LatLonAltCoord_t(30.0, 165.0, 1000.0, true);
            lla2 = new LatLonAltCoord_t(-15.0, -175.0, -355.5, true);
            llaplus = lla1 - lla2;
            Assert.AreEqual(llaplus.LongitudeDegrees, -20.0, 1e-10);
      
        }

        [Test()]
        public void LLAMultDivTest()
        {
            LatLonAltCoord_t lla1 = new LatLonAltCoord_t(30.0, 45.0, 1000.0, true);
            
            LatLonAltCoord_t llamult = 1.25 * lla1;
            Assert.AreEqual(llamult.LatitudeDegrees, 37.5, 1e-10);
            Assert.AreEqual(llamult.LongitudeDegrees, 56.25, 1e-10);
            Assert.AreEqual(llamult.Altitude, 1000.0, 1e-10);
 
            llamult = lla1 * -1.25;
            Assert.AreEqual(llamult.LatitudeDegrees, -37.5, 1e-10);
            Assert.AreEqual(llamult.LongitudeDegrees, -56.25, 1e-10);
            Assert.AreEqual(llamult.Altitude, 1000.0, 1e-10);

            llamult = lla1 / 2.5;
            Assert.AreEqual(llamult.LatitudeDegrees, 12.0, 1e-10);
            Assert.AreEqual(llamult.LongitudeDegrees, 18.0, 1e-10);
            Assert.AreEqual(llamult.Altitude, 1000.0, 1e-10);
        }

		[Test ()]
		public void LLConstructorTest()
		{
            LatLonCoord_t lla1 = new LatLonCoord_t(Math.PI / 4.0, Math.PI / 3.0);
            Assert.AreEqual(lla1.LatitudeDegrees, 180.0/4.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeDegrees, 180.0/3.0, 1e-10);

            LatLonCoord_t lla2 = new LatLonCoord_t(lla1);
            Assert.AreEqual(lla1.LatitudeRadians, lla2.LatitudeRadians, 1e-10);
            Assert.AreEqual(lla1.LongitudeRadians, lla2.LongitudeRadians, 1e-10);

            lla1.LatitudeDegrees = 30.0;
            lla1.LongitudeDegrees = -60.0;
            Assert.AreEqual(lla1.LatitudeRadians, Math.PI / 6.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeRadians, -Math.PI / 3.0, 1e-10);

            lla1.LatitudeDegrees = 100.0;
            lla1.LongitudeDegrees = 190.0;

            Assert.AreEqual(lla1.LatitudeDegrees, 90.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeDegrees, -170.0, 1e-10);

            lla1.LatitudeDegrees = -100.0;
            lla1.LongitudeDegrees = -190.0;

            Assert.AreEqual(lla1.LatitudeDegrees, -90.0, 1e-10);
            Assert.AreEqual(lla1.LongitudeDegrees, 170.0, 1e-10);

            lla1.LongitudeRadians = -Math.PI - Math.PI / 18.0;
            Assert.AreEqual(lla1.LongitudeDegrees, 170.0, 1e-10);

            lla1.LongitudeRadians = Math.PI + Math.PI / 18.0;
            Assert.AreEqual(lla1.LongitudeDegrees, -170.0, 1e-10);

		}

        [Test()]
        public void LLAddSubTest()
        {
            LatLonCoord_t lla1 = new LatLonCoord_t(30.0, 45.0, true);
            LatLonCoord_t lla2 = new LatLonCoord_t(10.0, 25.0, true);
            
            LatLonCoord_t llaplus = lla1 + lla2;
            Assert.AreEqual(llaplus.LatitudeDegrees, 40.0, 1e-10);
            Assert.AreEqual(llaplus.LongitudeDegrees, 70.0, 1e-10);
 
            LatLonCoord_t llaminus = lla1 - lla2;
            Assert.AreEqual(llaminus.LatitudeDegrees, 20.0, 1e-10);
            Assert.AreEqual(llaminus.LongitudeDegrees, 20.0, 1e-10);
       
            lla2 = new LatLonCoord_t(-15.0, 150.0, true);
            llaplus = lla1 + lla2;
            Assert.AreEqual(llaplus.LatitudeDegrees, 15.0, 1e-10);
            Assert.AreEqual(llaplus.LongitudeDegrees, -165.0, 1e-10);

            lla1 = new LatLonCoord_t(30.0, -170.0, true);
            lla2 = new LatLonCoord_t(-15.0, 150.0, true);
            llaplus = lla1 - lla2;
            Assert.AreEqual(llaplus.LatitudeDegrees, 45.0, 1e-10);
            Assert.AreEqual(llaplus.LongitudeDegrees, 40.0, 1e-10);
 
            lla1 = new LatLonCoord_t(30.0, 165.0, true);
            lla2 = new LatLonCoord_t(-15.0, -175.0, true);
            llaplus = lla1 - lla2;
            Assert.AreEqual(llaplus.LongitudeDegrees, -20.0, 1e-10);
      
        }

        [Test()]
        public void LLMultDivTest()
        {
            LatLonCoord_t lla1 = new LatLonCoord_t(30.0, 45.0, true);
            
            LatLonCoord_t llamult = 1.25 * lla1;
            Assert.AreEqual(llamult.LatitudeDegrees, 37.5, 1e-10);
            Assert.AreEqual(llamult.LongitudeDegrees, 56.25, 1e-10);
 
            llamult = lla1 * -1.25;
            Assert.AreEqual(llamult.LatitudeDegrees, -37.5, 1e-10);
            Assert.AreEqual(llamult.LongitudeDegrees, -56.25, 1e-10);

            llamult = lla1 / 2.5;
            Assert.AreEqual(llamult.LatitudeDegrees, 12.0, 1e-10);
            Assert.AreEqual(llamult.LongitudeDegrees, 18.0, 1e-10);
        }


        [Test]
        public void FindMaxNortEastCornerOfSetOfLatLonPointsTests()
        {
            List<LatLonCoord_t> latLonList = new List<LatLonCoord_t>();
            latLonList.Add(new LatLonCoord_t(10.0, -85, true));
            latLonList.Add(new LatLonCoord_t(12.0, -92, true));
            latLonList.Add(new LatLonCoord_t(1.0, -73, true));
            latLonList.Add(new LatLonCoord_t(-5.0, -100, true));
            latLonList.Add(new LatLonCoord_t(-11.0, -50, true));

            LatLonCoord_t neCorner = LatLonCoord_t.FindMaxNortEastCornerOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, 12.0, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, -50.0, 1.0e-9);

            latLonList.Clear();
            latLonList.Add(new LatLonCoord_t(10.0, -5, true));
            latLonList.Add(new LatLonCoord_t(12.0, -10, true));
            latLonList.Add(new LatLonCoord_t(1.0, 1, true));
            latLonList.Add(new LatLonCoord_t(-5.0, 9, true));
            latLonList.Add(new LatLonCoord_t(-11.0, 13, true));
            neCorner = LatLonCoord_t.FindMaxNortEastCornerOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, 12.0, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, 13.0, 1.0e-9);

            latLonList.Clear();
            latLonList.Add(new LatLonCoord_t(10.0, -175, true));
            latLonList.Add(new LatLonCoord_t(12.0, -177, true));
            latLonList.Add(new LatLonCoord_t(1.0, -169, true));
            latLonList.Add(new LatLonCoord_t(-5.0, 173, true));
            latLonList.Add(new LatLonCoord_t(-11.0, 178, true));
            neCorner = LatLonCoord_t.FindMaxNortEastCornerOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, 12.0, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, -169, 1.0e-9);
        }

        [Test]
        public void FindMinSouthWestCornerOfSetOfLatLonPointsTests()
        {
            List<LatLonCoord_t> latLonList = new List<LatLonCoord_t>();
            latLonList.Add(new LatLonCoord_t(10.0, -85, true));
            latLonList.Add(new LatLonCoord_t(12.0, -92, true));
            latLonList.Add(new LatLonCoord_t(1.0, -73, true));
            latLonList.Add(new LatLonCoord_t(-5.0, -100, true));
            latLonList.Add(new LatLonCoord_t(-11.0, -50, true));

            LatLonCoord_t neCorner = LatLonCoord_t.FindMinSouthWestCornerOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, -11.0, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, -100.0, 1.0e-9);

            latLonList.Clear();
            latLonList.Add(new LatLonCoord_t(10.0, -5, true));
            latLonList.Add(new LatLonCoord_t(12.0, -10, true));
            latLonList.Add(new LatLonCoord_t(1.0, 1, true));
            latLonList.Add(new LatLonCoord_t(-5.0, 9, true));
            latLonList.Add(new LatLonCoord_t(-11.0, 13, true));
            neCorner = LatLonCoord_t.FindMinSouthWestCornerOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, -11.0, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, -10.0, 1.0e-9);

            latLonList.Clear();
            latLonList.Add(new LatLonCoord_t(10.0, -175, true));
            latLonList.Add(new LatLonCoord_t(12.0, -177, true));
            latLonList.Add(new LatLonCoord_t(1.0, 169, true));
            latLonList.Add(new LatLonCoord_t(-5.0, 173, true));
            latLonList.Add(new LatLonCoord_t(-11.0, 178, true));
            neCorner = LatLonCoord_t.FindMinSouthWestCornerOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, -11.0, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, 169, 1.0e-9);
        }

        [Test]
        public void FindCenterOfSetOfLatLonPointsTests()
        {
            List<LatLonCoord_t> latLonList = new List<LatLonCoord_t>();
            latLonList.Add(new LatLonCoord_t(10.0, -85, true));
            latLonList.Add(new LatLonCoord_t(12.0, -92, true));
            latLonList.Add(new LatLonCoord_t(1.0, -73, true));
            latLonList.Add(new LatLonCoord_t(-5.0, -100, true));
            latLonList.Add(new LatLonCoord_t(-11.0, -50, true));

            LatLonCoord_t neCorner = LatLonCoord_t.FindCenterOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, 0.5, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, -75.0, 1.0e-9);

            latLonList.Clear();
            latLonList.Add(new LatLonCoord_t(10.0, -5, true));
            latLonList.Add(new LatLonCoord_t(12.0, -10, true));
            latLonList.Add(new LatLonCoord_t(1.0, 1, true));
            latLonList.Add(new LatLonCoord_t(-5.0, 9, true));
            latLonList.Add(new LatLonCoord_t(-11.0, 13, true));
            neCorner = LatLonCoord_t.FindCenterOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, 0.5, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, 1.5, 1.0e-9);

            latLonList.Clear();
            latLonList.Add(new LatLonCoord_t(10.0, -175, true));
            latLonList.Add(new LatLonCoord_t(12.0, -177, true));
            latLonList.Add(new LatLonCoord_t(1.0, 169, true));
            latLonList.Add(new LatLonCoord_t(-5.0, 173, true));
            latLonList.Add(new LatLonCoord_t(-11.0, 178, true));
            neCorner = LatLonCoord_t.FindCenterOfSetOfLatLonPoints(latLonList);
            Assert.AreEqual(neCorner.LatitudeDegrees, 0.5, 1.0e-9);
            Assert.AreEqual(neCorner.LongitudeDegrees, 177, 1.0e-9);
        }


    }
}
