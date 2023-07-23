namespace GuidanceCmd
{
    public class Simulator
    {
        protected VehicleS flt {get; private set;}
        protected Orbital final { get; private set;}
        
        /// <summary>
        /// Set TARGET and origin parameters.
        /// </summary>
        /// <param name="final">Target radius, velocity and path angle.</param>
        public Simulator(double radius, double velocity, double pathAngle,
            double pitch, double grossMass, double flightTime, Orbital final)
        {
            flt = new VehicleS
                (radius, velocity, pathAngle, pitch, grossMass, flightTime, final);
            this.final = final;
        }

        /// <summary>
        /// Reset parameters except mass, which will be read from dictionary.
        /// </summary>
        public void RefreshSimulator(double radius, double velocity, double pathAngle,
            double pitch, double flightTime)
        {
            flt.RefreshGuidance(radius, velocity, pathAngle, pitch, flightTime);
        }

        /// <summary>
        /// Calculate terminal speed. [Caution] flight parameters will change.
        /// </summary>
        /// <returns>(double) Terminal speed</returns>
        public double TerminalVelocity()
        {
            if (flt.fltTime < Vehicle.T_TOT - 15)
                flt.GenerateCurve();
            while (flt.fltTime < Vehicle.T_TOT)
            {
                flt.CommandInput();
            }
            
            return flt.obt.velocity;
        }
    } // class Simulator
} // namespace GuidanceCmd