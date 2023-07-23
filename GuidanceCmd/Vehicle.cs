using System;
using GuidanceMath;

namespace GuidanceCmd
{
    public class Vehicle
    {
        public static readonly double
            MAXAOA = Equ.Rad(52),
            MAX_ENGM_AOA = Equ.Rad(40),
            MAX_ANGLE_RATE = Equ.Rad(7);        // per second

        public const double
            FAIRING_JET_HEIGHT = 104500,
            V_EXC_BEFORE_IGN = 480,

            T_ENGM = 115,
            T_TOT = 174.4,
            T_ENGM_RESV = 45,
            V_ENGM_END = 15,
            SIMULATE_DT = 0.02;

        public const int 
            LOOP_SLEEP_T = 200;                 // ms
        
        protected Vehicle() { }
        public Vehicle(double radius, double speed, double pathAngle,
            double pitch, double grossMass, double fltTime, Orbital final)
        {
            obt = new Orbital(radius, speed, pathAngle);
            this.pitch = pitch;
            this.fltTime = fltTime;
            lastTime = fltTime - LOOP_SLEEP_T / 1000d;
            this.orgMass = grossMass;
            this.final = final;
        }

        public Orbital obt { get; protected set; }
        protected Orbital final { get; set; }

        protected Tuple<double, double, double, double, double>
            guidanceCurve;
        protected double
            orgMass, // should only be set at T = 0 (or < 0)
            pitch,
            sideSlipAngle,
            lastTime;

        public double
            fltTime;

        /// <summary>
        /// Generate cubic curve with a time mark (Item5).
        /// </summary>
        public void GenerateCurve()
        {
            try
            {
                double
                    v0 = obt.velocity,
                    h0 = obt.radius - Orbital.R0,
                    a0 = obt.pathAngle,
                    rt = T_TOT - fltTime,
                    vt = final.velocity,
                    at = final.pathAngle,
                    ht = final.radius - Orbital.R0;

                guidanceCurve = Tuple.Create(
                    h0,
                    v0 * Math.Sin(a0),
                    -1 / rt * (2 * v0 * Math.Sin(a0) + vt * Math.Sin(at))
                        + 3 * (ht - h0) / (rt * rt),
                    (v0 * Math.Sin(a0) + vt * Math.Sin(at)) / (rt * rt)
                        + 2 * (h0 - ht) / (rt * rt * rt),
                    fltTime);
            }
            catch (Exception e)
            {
                Console.WriteLine(e.ToString());
            }
        } // CreateCurve

        protected double CalculateCurve(double time, short derivative)
        {
            time -= guidanceCurve.Item5;
            if (derivative == 0)
            {
                return guidanceCurve.Item1 + guidanceCurve.Item2 * time + guidanceCurve.Item3 * time * time
                    + guidanceCurve.Item4 * time * time * time;
            }
            if (derivative == 1)
            {
                return guidanceCurve.Item2 + 2 * guidanceCurve.Item3 * time
                + 3 * guidanceCurve.Item4 * time * time;
            }
            if (derivative == 2)
            {
                return 2 * guidanceCurve.Item3 + 6 * guidanceCurve.Item4 * time;
            }
            if (derivative == 3)
            {
                return guidanceCurve.Item4;
            }
            else
            {
                return 0d;
            }
        } // calc curve

        protected double GetGuidancePitch(double nowThrust, double mass, double time)
        {
            double slope = CalculateCurve(time, 2);
            double axisAccel = nowThrust / mass;
            // if (Math.Abs(slope + 9.81) > axisAccel) Console.WriteLine("HIGH ACC REQUIRED");

            double val = -Math.Pow(obt.velocity * Math.Cos(obt.pathAngle), 2) / obt.radius;
            double vari = (slope + obt.GFieldStrength() + val) / axisAccel;
            double newPitch;
            if (vari <= 1 && vari >= -1)
            {
                newPitch = Math.Asin(vari); // (?)
            }
            else
            {
                double aoa;
                if (slope - Math.Tan(obt.pathAngle) >= 0)
                {
                    aoa = MAXAOA;
                }
                else
                {
                    aoa = -MAXAOA;
                }
                newPitch = aoa + obt.pathAngle;
            }
            return newPitch;
        } // GuidancePitch
            
    } // class Vehicle

    public class VehicleS : Vehicle // for simulation
    {
        public VehicleS() { }
        public VehicleS(double radius, double speed, double pathAngle,
            double pitch, double grossMass, double fltTime, Orbital final)
        {
            obt = new Orbital(radius, speed, pathAngle);
            this.pitch = pitch;
            this.fltTime = fltTime;
            this.orgMass = grossMass;
            lastTime = fltTime - SIMULATE_DT; // to avoid delta_t == 0 in GetPitch()
            this.final = final;
        }

        public void RefreshGuidance(double radius, double speed, double pathAngle,
            double pitch, double fltTime)
        // Only for init simulator
        {
            obt.Refresh(radius, speed, pathAngle);
            this.pitch = pitch;
            lastTime = fltTime - SIMULATE_DT;
            this.fltTime = fltTime;
        }

        /// <summary>
        /// Main simulation process. fltTime will change.
        /// </summary>
        public void CommandInput(double deltaT=SIMULATE_DT)
        {
            if (fltTime <= T_TOT - 15)
            {
                GenerateCurve();
            }
            if (fltTime >= 0) { pitch = GetPitch(); }
            Predict(Prop.GetThrust(fltTime), deltaT);
        }

        /// <summary>
        /// Get guidance pitch for SIMULATION.
        /// </summary>
        /// <returns>(double) pitch</returns>
        public double GetPitch()
        {
            double newPitch = GetGuidancePitch
                (Prop.GetThrust(fltTime), orgMass - Prop.GetConsumedMass(fltTime), fltTime);
            
            double delta_t = fltTime - lastTime;
            double angleRate = (newPitch - pitch) / delta_t,
                absRate = Math.Abs(angleRate);
            if (absRate > MAX_ANGLE_RATE)
            {
                newPitch = pitch + angleRate / absRate * MAX_ANGLE_RATE * delta_t;
            }
            
            newPitch = Math.Max(newPitch, -MAXAOA + obt.pathAngle);
            newPitch = Math.Min(newPitch, MAXAOA + obt.pathAngle);
            return newPitch;
        } // GetPitch

        private void Predict(double thrust, double deltaT)
        // Change T & lastT
        {
            double g = obt.GFieldStrength();
            double aoa = pitch - obt.pathAngle;
            double grossMass = orgMass - Prop.GetConsumedMass(fltTime);
            obt.velocity += (thrust * Math.Cos(aoa) / grossMass
                - g * Math.Sin(obt.pathAngle)) * deltaT;

            obt.pathAngle += (thrust * Math.Sin(aoa) / grossMass / obt.velocity
                - g * Math.Cos(obt.pathAngle) / obt.velocity
                + obt.velocity * Math.Cos(obt.pathAngle) / obt.radius) * deltaT;
            
            obt.radius += obt.velocity * Math.Sin(obt.pathAngle) * deltaT;
            
            lastTime = fltTime;
            fltTime += deltaT;
        }
    } // VehicleS

    public class VehicleR : Vehicle
    {
        private double grossMass; // Must update in time.

        protected VehicleR() { }

        /// <summary>
        /// Set TARGET and origin parameter.
        /// </summary>
        public VehicleR(double radius, double speed, double pathAngle,
            double pitch, double grossMass, double fltTime, Orbital final)
        {
            obt = new Orbital(radius, speed, pathAngle);
            this.pitch = pitch;
            this.fltTime = fltTime;
            lastTime = fltTime - LOOP_SLEEP_T / 1000d; // To avoid delta_T == 0.
            this.orgMass = grossMass;
            this.final = final;
        }

        /// <summary>
        /// Updata flight parameter, including mass.
        /// </summary>
        public void RefreshGuidance(double radius, double speed, double pathAngle,
            double pitch, double grossMass, double fltTime)
        {
            obt.Refresh(radius, speed, pathAngle);
            this.pitch = pitch;
            lastTime = this.fltTime;
            this.fltTime = fltTime;
            this.grossMass = grossMass;
        }

        /// <summary>
        /// Get guidance pitch in CLOSE-LOOP PHASE.
        /// </summary>
        /// <param name="nowThrust">real-time data from ksp</param>
        /// <returns>(double) pitch</returns>
        public double GetPitch(double nowThrust)
        {
            double newPitch;
            if (fltTime > T_TOT - 8)
            {
                // offset 1 sec
                newPitch = GetGuidancePitch(nowThrust, grossMass, fltTime + 1);
            }
            else
            {
                newPitch = GetGuidancePitch(nowThrust, grossMass, fltTime);
            }

            double delta_t = fltTime - lastTime;
            double angleRate = (newPitch - pitch) / delta_t,
                absRate = Math.Abs(angleRate);
            if (absRate > MAX_ANGLE_RATE)
            {
                newPitch = pitch + angleRate / absRate * 2 * MAX_ANGLE_RATE * delta_t;
                Console.WriteLine("\t\t\t\t(Max angle rate limit)");
            }
            
            newPitch = Math.Max(newPitch, -MAXAOA + obt.pathAngle);
            newPitch = Math.Min(newPitch, MAXAOA + obt.pathAngle);
            return newPitch;
        } // GetPitch

        /// <summary>
        /// Get guidance pitch in ENERGY MANAGEMENT PHASE
        /// </summary>
        /// <param name="nowThrust">real-time thrust</param>
        /// <param name="excVelocity">total excess velocity</param>
        /// <returns></returns>
        public double GetPitch(double nowThrust, double excVelocity)
        {
            double
                newPitch = GetGuidancePitch(nowThrust, grossMass, fltTime),
                engmAoA = GetManagementAoA(nowThrust, excVelocity); // postive

            if (fltTime >= T_ENGM_RESV)
            {
                newPitch -= engmAoA;
            }
            else
            {
                newPitch += engmAoA;
            }
            
            double delta_t = fltTime - lastTime;
            double angleRate = (newPitch - pitch) / delta_t,
                absRate = Math.Abs(angleRate);
            
            if (absRate > MAX_ANGLE_RATE)
            {
                newPitch = pitch + angleRate / absRate * 2 * MAX_ANGLE_RATE * delta_t;
                // Console.Write("\tMax angle rate limit.");
            }
            
            newPitch = Math.Max(newPitch, -MAXAOA + obt.pathAngle);
            newPitch = Math.Min(newPitch, MAXAOA + obt.pathAngle);
            return newPitch;
        } // GetPitch (excV)


        private double GetManagementAoA(double nowThrust, double excV, double ek = 3)
        {
            double grossMass = orgMass - Prop.GetConsumedMass(fltTime);
            double dSc = -excV * ek / (T_ENGM - fltTime);
            double angle;
            double vari = dSc / (nowThrust / grossMass) + 1;
            if (vari <= 1 && vari >= -1)
            {
                angle = Math.Acos(vari);
                angle = Math.Min(MAX_ENGM_AOA, angle);
                Console.Write($"EM {Math.Round(Equ.Deg(angle), 3)}\t");
                return angle;
            }
            return MAX_ENGM_AOA;
        } // EMNG AOA
    }
}
