using NUnit.Framework;
using System;
using GeoCoordinateSystemNS;

namespace GeoCoordinateSystemUnitTests
{
    [TestFixture ()]
    class XYZCoordinateTests
    {

        [Test()]
        public void XYZEqualOperatorTest()
        {
            double delta = 0.5 * xyzCoord_t.EqualEpslon;
            xyzCoord_t A = new xyzCoord_t(10.0, 20.0, 30.0);
            xyzCoord_t B = new xyzCoord_t(10.0 + delta, 20.0 - delta, 30.0 + delta);
            Assert.IsTrue(A == B);
            Assert.IsFalse(A != B);

            delta = 2.0 * xyzCoord_t.EqualEpslon;
            B = new xyzCoord_t(10.0 + delta, 20.0 - delta, 30.0 + delta);
            Assert.IsFalse(A == B);
            Assert.IsTrue(A != B);

            //Implicit Convertion Test
            xyCoord_t xyVecA = new xyCoord_t(-15.5, 23.8);
            xyzCoord_t xyzVecA = xyVecA;
            Assert.AreEqual(xyzVecA.x, xyVecA.x);
            Assert.AreEqual(xyzVecA.y, xyVecA.y);
            Assert.AreEqual(xyzVecA.z, 0.0);

            ////Explicit Convertion Test
            xyCoord_t xyVecB = (xyCoord_t)B;
            Assert.AreEqual(xyVecB.x, B.x);
            Assert.AreEqual(xyVecB.y, B.y);
        }

        [Test()]
        public void XYZPlusMinusOperatorTests()
        {
            xyzCoord_t A = new xyzCoord_t(10.0, 20.0, 30.0);
            xyzCoord_t B = new xyzCoord_t(25.0, -10.0, 15.0);
            xyzCoord_t Cans = new xyzCoord_t(35.0, 10.0, 45.0);
            xyzCoord_t C = A + B;
            Assert.IsTrue(C == Cans);

            Cans = new xyzCoord_t(-15.0, 30.0, 15.0);
            C = A - B;
            Assert.IsTrue(C == Cans);
        }

        [Test()]
        public void XYZMultDivOperatorTests()
        {
            xyzCoord_t A = new xyzCoord_t(10.0, 20.0, 30.0);
            xyzCoord_t C = 2.5 * A;
            xyzCoord_t Cans = new xyzCoord_t(25.0, 50.0, 75.0);
            Assert.IsTrue(C == Cans);
            C = A * 2.5;
            Assert.IsTrue(C == Cans);

            C = A / 2.0;
            Cans = new xyzCoord_t(5.0, 10.0, 15.0);
            Assert.IsTrue(C == Cans);
        }

        [Test()]
        public void XYZDotProdMagInnerProdOperatorTests()
        {
            xyzCoord_t A = new xyzCoord_t(10.0, 20.0, 30.0);
            xyzCoord_t B = new xyzCoord_t(5.0, -45.0, -60.0);
            double dpVal = A.InnerProduct(B);
            Assert.AreEqual(dpVal, -2650.0, 0.00001);

            double opVal = A.OuterProdXY(B);
            Assert.AreEqual(opVal, -550.0, 0.00001);

            double magA = A.Magnitude();
            Assert.AreEqual(magA, 37.416573867739416, 0.00001);
            double magB = B.Magnitude();
            Assert.AreEqual(magB, 75.166481891864535, 0.00001);

            magA = A.MagnitudeCityBlock();
            Assert.AreEqual(magA, 30.0, 0.00001);
            magB = B.MagnitudeCityBlock();
            Assert.AreEqual(magB, 60.0, 0.00001);

            double headingA = A.HeadingDegrees();
            Assert.AreEqual(headingA, 26.56505117707799, 0.00001);

            headingA = A.HeadingRadians();
            Assert.AreEqual(headingA, 0.46364760900080609, 0.00001);

            double dist = A.Distance(B);
            Assert.AreEqual(dist, 111.13055385446434, 0.00001);

            dist = A.DistanceCityBlock(B);
            Assert.AreEqual(dist, 90.0, 0.00001);

            xyzCoord_t normA = A.NormalizedVector();
            xyzCoord_t Cans = new xyzCoord_t(0.26726124191242434, 0.53452248382484868, 0.80178372573727308);
            Assert.IsTrue(normA == Cans);
            magA = normA.Magnitude();
            Assert.AreEqual(magA, 1.0, 0.00001);

            xyzCoord_t rotA = A.RotateXYVec(90.0, true);
            Cans = new xyzCoord_t(-20.0, 10.0, 30.0);
            Assert.IsTrue(rotA == Cans);
        }


        [Test()]
        public void XYEqualOperatorTest()
        {
            double delta = 0.5 * xyCoord_t.EqualEpslon;
            xyCoord_t A = new xyCoord_t(10.0, 20.0);
            xyCoord_t B = new xyCoord_t(10.0 + delta, 20.0 - delta);
            Assert.IsTrue(A == B);
            Assert.IsFalse(A != B);

            delta = 2.0 * xyCoord_t.EqualEpslon;
            B = new xyCoord_t(10.0 + delta, 20.0 - delta);
            Assert.IsFalse(A == B);
            Assert.IsTrue(A != B);

            //Implicit Convertion Test
            xyCoord_t xyVecA = new xyCoord_t(-15.5, 23.8);
            xyzCoord_t xyzVecA = xyVecA;
            Assert.AreEqual(xyzVecA.x, xyVecA.x);
            Assert.AreEqual(xyzVecA.y, xyVecA.y);
            Assert.AreEqual(xyzVecA.z, 0.0);

            ////Explicit Convertion Test
            xyCoord_t xyVecB = (xyCoord_t)xyzVecA;
            Assert.AreEqual(xyVecB.x, xyzVecA.x);
            Assert.AreEqual(xyVecB.y, xyzVecA.y);
        }

        [Test()]
        public void XYPlusMinusOperatorTests()
        {
            xyCoord_t A = new xyCoord_t(10.0, 20.0);
            xyCoord_t B = new xyCoord_t(25.0, -10.0);
            xyCoord_t Cans = new xyCoord_t(35.0, 10.0);
            xyCoord_t C = A + B;
            Assert.IsTrue(C == Cans);

            Cans = new xyCoord_t(-15.0, 30.0);
            C = A - B;
            Assert.IsTrue(C == Cans);
        }

        [Test()]
        public void XYMultDivOperatorTests()
        {
            xyCoord_t A = new xyCoord_t(10.0, 20.0);
            xyCoord_t C = 2.5 * A;
            xyCoord_t Cans = new xyCoord_t(25.0, 50.0);
            Assert.IsTrue(C == Cans);
            C = A * 2.5;
            Assert.IsTrue(C == Cans);

            C = A / 2.0;
            Cans = new xyCoord_t(5.0, 10.0);
            Assert.IsTrue(C == Cans);
        }

        [Test()]
        public void XYDotProdMagInnerProdOperatorTests()
        {
            xyCoord_t A = new xyCoord_t(10.0, 20.0);
            xyCoord_t B = new xyCoord_t(5.0, -45.0);
            double dpVal = A.InnerProduct(B);
            Assert.AreEqual(dpVal, -850.0, 0.00001);

            double opVal = A.OuterProd(B);
            Assert.AreEqual(opVal, -550.0, 0.00001);

            double magA = A.Magnitude();
            Assert.AreEqual(magA, 22.360679774997898, 0.00001);
            double magB = B.Magnitude();
            Assert.AreEqual(magB, 45.276925690687087, 0.00001);

            magA = A.MagnitudeCityBlock();
            Assert.AreEqual(magA, 20.0, 0.00001);
            magB = B.MagnitudeCityBlock();
            Assert.AreEqual(magB, 45.0, 0.00001);

            double headingA = A.HeadingDegrees();
            Assert.AreEqual(headingA, 26.56505117707799, 0.00001);

            headingA = A.HeadingRadians();
            Assert.AreEqual(headingA, 0.46364760900080609, 0.00001);

            double dist = A.Distance(B);
            Assert.AreEqual(dist, 65.192024052026483, 0.00001);

            dist = A.DistanceCityBlock(B);
            Assert.AreEqual(dist, 65.0, 0.00001);

            xyCoord_t normA = A.NormalizedVector();
            xyCoord_t Cans = new xyCoord_t(0.44721359549995793, 0.89442719099991586);
            Assert.IsTrue(normA == Cans);
            magA = normA.Magnitude();
            Assert.AreEqual(magA, 1.0, 0.00001);

            xyCoord_t rotA = A.RotateXYVec(90.0, true);
            Cans = new xyCoord_t(-20.0, 10.0);
            Assert.IsTrue(rotA == Cans);
        }

    }
}
