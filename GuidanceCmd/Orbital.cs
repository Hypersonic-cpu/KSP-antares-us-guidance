using System;

namespace GuidanceCmd
{
    public class Orbital
    {
        public static readonly double
            G0 = 9.806,
            GM = 3986005e8,         // m^3/s^2
            R0 = 6371e3,
            R0A = 6378140,
            E2 = 0.00669438487525,
            OMEGA = 7292115e-11;    // self-rotation angular speed, in rad/s

        public double
            velocity,
            pathAngle,
            radius;

        public Orbital(double radius, double velocity, double pathAngle)
        {
            this.radius = radius;
            this.velocity = velocity;
            this.pathAngle = pathAngle;
        }
        public void Refresh(double radius, double velocity, double pathAngle)
        {
            this.radius = radius;
            this.velocity = velocity;
            this.pathAngle = pathAngle;
        }

        public Tuple<double, double, double, double>
            OrbitInfo()
        {
            double
                semiMaj = 1 / (2 / radius - velocity * velocity / GM),
                var1 = (radius * velocity * Math.Cos(pathAngle)),
                ecc = Math.Sqrt(1 - var1 * var1 / GM / semiMaj),
                ap = semiMaj * (1 + ecc),
                pe = semiMaj * (1 - ecc);
            return Tuple.Create(semiMaj, ecc, ap - R0, pe - R0);
        }

        public double GFieldStrength()
        {
            return GM / (radius * radius);
        }

    }
}
