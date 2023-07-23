using System;

namespace GuidanceMath
{
    public static class Equ
    {
        /*
        public static double NewtonMethod(double cur, double precision, int equNum) 
        {
            double value = DAEM_Equation_Theta2(cur);
            double deriv;

            while (Math.Abs(value) > precision)
            {
                deriv = DAEM_Derivative_Theta2(cur);
                cur -= value / deriv;
                value = DAEM_Equation_Theta2(cur);
            }

            return cur;
        }

        static double DAEM_Equation_Theta2(double theta2)
        {
            return theta2 * theta2 + theta2 - 1 + Math.Sin(theta2);
        }
        static double DAEM_Derivative_Theta2(double theta2) 
        {
            return 2 * theta2 + 1 + Math.Cos(theta2);
        }
        */

        static public double VectorLength(Tuple<double, double, double> vector)
        {
            return Math.Sqrt(vector.Item1 * vector.Item1 + vector.Item2 * vector.Item2
                + vector.Item3 * vector.Item3);
        }
        
        static public double VectorDotProduct
            (Tuple<double, double, double> vector1, Tuple<double, double, double> vector2)
        {
            return vector1.Item1 * vector2.Item1 + vector1.Item2 * vector2.Item2
                + vector1.Item3 * vector2.Item3;
        }

        static public double VectorAngle
            (Tuple<double, double, double> vector1, Tuple<double, double, double> vector2)
        {
            return Math.Acos(VectorDotProduct(vector1, vector2)
                            / (VectorLength(vector2) * VectorLength(vector1)));
        }

        /* v1 cross v2 = 
         * |i   j   k   |
         * |v1x v1y v1z |
         * |v2x v2y v2z |
         * = (v1y*v2z - v1z*v2y)i +(v1z*v2x - v1x*v2z)j + (v1x*v2y - v1y*v2x)i 
         */
        static public Tuple<double, double, double>
            VectorCrossProduct(Tuple<double, double, double> vector1, Tuple<double, double, double> vector2)
        {
            return Tuple.Create
                (
                    vector1.Item2 * vector2.Item3 - vector1.Item3 * vector2.Item2,
                    vector1.Item3 * vector2.Item1 - vector1.Item1 * vector2.Item3,
                    vector1.Item1 * vector2.Item2 - vector1.Item2 * vector2.Item1
                );
        }

        static public Tuple <double, double, double>
            UnitVector(Tuple<double, double, double> vector)
        {
            double len = VectorLength(vector);
            return Tuple.Create
                (
                    vector.Item1 / len,
                    vector.Item2 / len,
                    vector.Item3 / len
                );
        }

        static public double Deg(double rad)
        {
            return rad / Math.PI * 180d;
        }
        static public double Rad(double deg)
        {
            return deg * Math.PI / 180d;
        }
    }
}
