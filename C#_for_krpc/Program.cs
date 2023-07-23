using System;
using System.IO;
using System.Net;
using System.Threading;
using System.Diagnostics;

using KRPC.Client;
using KRPC.Client.Services.KRPC;
using KRPC.Client.Services.SpaceCenter;

using GuidanceCmd;
using GuidanceMath;
using KRPC.Client.Services.RemoteTech;
using KRPC.Schema.KRPC;
using System.Linq;

namespace C__for_krpc
{
    
    internal class Program
    {
        internal static void Main()
        {
            var F = new StreamReader("../../../TARGET.txt");
            String[] str = F.ReadToEnd().Split('\n');
            F.Close();
            double ap = double.Parse(str[0].Split('=').Last()) + Orbital.R0, 
                pe = double.Parse(str[1].Split('=').Last()) + Orbital.R0;
            double v = Math.Sqrt(Orbital.GM * (2.0 / pe - 2.0 / (pe + ap)) );
            var TARGET = new Orbital(pe, v, 0.0);
            Console.WriteLine($"\n[ TARGET ] = \nAPOGEE  ALTITUDE\t{(ap - Orbital.R0) / 1000.0} KM\nPERIGEE ALTITUDE\t{(pe - Orbital.R0) / 1000.0} KM\n");
            Console.WriteLine($"RADIUS\t\t\t{TARGET.radius/1000.0} KM\nVELOCITY\t\t{Math.Round(TARGET.velocity)} M/S\nFLIGHT PATH ANGLE\t{TARGET.pathAngle} RAD");
            // Console.ReadKey();

            Console.Write("krpc connecting : ");
            using (var connection = new Connection(
                   name: "KRPC1",
                   address: IPAddress.Parse("127.0.0.1"),
                   rpcPort: 50000,
                   streamPort: 50001))
            {

                var krpc = connection.KRPC();
                var spaceCenter = connection.SpaceCenter();
                var vessel = spaceCenter.ActiveVessel;
                Console.WriteLine(vessel.Name);

                var refFrame = vessel.Orbit.Body.NonRotatingReferenceFrame;
                var fixFrame = vessel.Orbit.Body.ReferenceFrame;
                var ctlFrame = vessel.SurfaceReferenceFrame;
                var posStream = connection.AddStream(() => vessel.Position(refFrame));
                var velStream = connection.AddStream(() => vessel.Velocity(refFrame));
                var grvStream = connection.AddStream(() => vessel.Velocity(fixFrame));
                var utStream = connection.AddStream(() => spaceCenter.UT);
                var dirStream = connection.AddStream(() => vessel.Direction(refFrame));
                var massStream = connection.AddStream(() => vessel.Mass);
                var thrustStream = connection.AddStream(() => vessel.Thrust);
                Func<double> speed = () => Equ.VectorLength(velStream.Get());
                Func<double> pathAngle = () => Math.PI / 2 - Equ.VectorAngle(posStream.Get(), velStream.Get());
                Func<double> radius = () => Equ.VectorLength(posStream.Get());
                Func<double> pitch = () => Math.PI / 2 - Equ.VectorAngle(posStream.Get(), dirStream.Get());
                //Func<double> incidence = () => Equ.VectorAngle(Tuple.Create(0d, 1d, 0d), 
                //    Equ.VectorCrossProduct(posStream.Get(), dirStream.Get()));
                
                // MECO
                while (vessel.Control.CurrentStage != 4)
                {
                    Thread.Sleep(100);
                }

                vessel.AutoPilot.Engage();
                vessel.AutoPilot.ReferenceFrame = ctlFrame;
                vessel.Control.RCS = true;
                
                do
                {
                    var vG = spaceCenter.TransformDirection(Equ.UnitVector(grvStream.Get()), fixFrame, ctlFrame);
                    double vHeadingAngle = Math.Acos(vG.Item2 / Math.Sqrt(vG.Item2 * vG.Item2 + vG.Item3 * vG.Item3));
                    vessel.AutoPilot.TargetDirection = System.Tuple.Create
                        (
                            Math.Sin(pathAngle()),
                            Math.Cos(pathAngle()) * Math.Cos(vHeadingAngle),
                            Math.Cos(pathAngle()) * Math.Sin(vHeadingAngle)
                        );
                } while (false);

                while (radius() - Orbital.R0 < Vehicle.FAIRING_JET_HEIGHT)
                {
                    Thread.Sleep(50);
                }
                // Payload fairing jettison
                vessel.Control.ActivateNextStage();
                Console.WriteLine("[Action] Payload Fairing Jettison.");
                Thread.Sleep(2000);
                // Interstage separation.
                Console.WriteLine("[Action] Interstage Separation.");
                vessel.Control.ActivateNextStage();

                Simulator sim_ = new Simulator(radius(), speed(), pathAngle(), pitch(), massStream.Get(), 0, TARGET);
                do
                {
                    double lastExcV = -1000;
                    do
                    {
                        Thread.Sleep(50);
                        sim_.RefreshSimulator(radius(), speed(), pathAngle(), pitch(), 0);
                        double excV = sim_.TerminalVelocity() - TARGET.velocity;
                        Console.Write($"\r[ Info ] Excess Velocity {Math.Round(excV, 1)}M/S    ");
                        // +75m/s offset (???)
                        // [60, LIMIT) and increasing.
                        if (60 < excV && excV < Vehicle.V_EXC_BEFORE_IGN && excV > lastExcV) break;
                        // <100 and decreasing.
                        if (excV < 60 && excV < lastExcV) break;
                        lastExcV = excV;
                    } while (true);
                } while (false);

                Console.WriteLine("\r[Action] Upper Stage Ignition.    ");
                // Stage2 Ignition
                double T_IGN = utStream.Get();
                vessel.Control.ActivateNextStage();
                short fltPhase = 0;
                vessel.Control.RCS = false;

                Simulator sim = new Simulator(radius(), speed(), pathAngle(), pitch(), massStream.Get(), 0, TARGET);
                VehicleR vehicle = new VehicleR
                    (radius(), speed(), pathAngle(), pitch(), massStream.Get(), 0d, TARGET);

                while (vehicle.fltTime <= Vehicle.T_TOT + 5)
                {
                    double
                        T = utStream.Get() - T_IGN,
                        radius_ = radius(),
                        speed_ = speed(),
                        pathAngle_ = pathAngle(),
                        pitch_ = pitch(),
                        mass_ = massStream.Get();
                        
                    vehicle.RefreshGuidance
                        (radius_, speed_, pathAngle_, pitch_, mass_, T);
                    double newPitch, excVelocity;

                    if (fltPhase == 0)
                    {
                        sim.RefreshSimulator(radius_, speed_, pathAngle_, pitch_, T);
                        excVelocity = sim.TerminalVelocity() - TARGET.velocity;
                        
                        vehicle.GenerateCurve();
                        newPitch = vehicle.GetPitch(thrustStream.Get(), excVelocity - Vehicle.V_ENGM_END);
                        
                        if (excVelocity <= Vehicle.V_ENGM_END || T > Vehicle.T_ENGM - 10)
                        {
                            Console.WriteLine("\r[Guide ] Energy Management Ceased.                             ");
                            fltPhase = 1;
                        }
                        Console.Write($"\rU+{Math.Round(T, 1)}s    Pitch {Math.Round(Equ.Deg(pitch_), 1)}DEG    " +
                            $"Height {Math.Round((radius_ - Orbital.R0)/1000.0, 3)}    Excess V {Math.Round(excVelocity)}M/S    ");
                    }
                    else if (fltPhase == 1)
                    {
                        vehicle.GenerateCurve();
                        newPitch = vehicle.GetPitch(thrustStream.Get());
                        if (vehicle.fltTime >= Vehicle.T_TOT - 8)
                        {
                            vessel.Control.RCS = true;
                            fltPhase = 2;
                        }
                        Console.Write($"\rU+{Math.Round(T, 1)}s    Pitch {Math.Round(Equ.Deg(pitch_), 1)}DEG    " +
                            $"Height {Math.Round((radius_ - Orbital.R0) / 1000.0, 3)}KM    |V| {Math.Round(speed_)}M/S    ");
                    }
                    else // fltPhase = 2.
                    {
                        newPitch = vehicle.GetPitch(thrustStream.Get());
                        var ApPe = vehicle.obt.OrbitInfo();
                        Console.Write($"\rU+{Math.Round(T, 1)}s    " +
                            $"Ap, Pe height ({Math.Round(ApPe.Item1/1000.0, 3)}, {Math.Round(ApPe.Item2/1000.0, 3)})KM    |V error| {Math.Round((speed_ - v), 3)}M/S    ");
                    }

                    var vG = spaceCenter.TransformDirection(Equ.UnitVector(grvStream.Get()), fixFrame, ctlFrame);
                    double vHeadingAngle = Math.Acos(vG.Item2 / Math.Sqrt(vG.Item2 * vG.Item2 + vG.Item3 * vG.Item3));
                    vessel.AutoPilot.TargetDirection = System.Tuple.Create
                        (
                            Math.Sin(newPitch), 
                            Math.Cos(newPitch) * Math.Cos(vHeadingAngle), 
                            Math.Cos(newPitch) * Math.Sin(vHeadingAngle)
                        );

                    Thread.Sleep(Vehicle.LOOP_SLEEP_T);
                } // while loop

                Console.WriteLine("\n[Action] Upper Stage Burn-out.");
                vessel.Control.RCS = true;
                do
                {
                    var vG = spaceCenter.TransformDirection(Equ.UnitVector(grvStream.Get()), fixFrame, ctlFrame);
                    double vHeadingAngle = Math.Acos(vG.Item2 / Math.Sqrt(vG.Item2 * vG.Item2 + vG.Item3 * vG.Item3));
                    vessel.AutoPilot.TargetDirection = System.Tuple.Create
                        (
                            Math.Sin(pathAngle()),
                            Math.Cos(pathAngle()) * Math.Cos(vHeadingAngle),
                            Math.Cos(pathAngle()) * Math.Sin(vHeadingAngle)
                        );
                } while (false);

                vessel.Control.Throttle = 0;
                Console.WriteLine("[Guide ] End.");
                vessel.AutoPilot.Wait();
                Thread.Sleep(6000);
                Console.WriteLine("[Action] Cygnus Separation.");
                vessel.Control.ActivateNextStage();
                vessel.AutoPilot.Disengage();
            } // using conn
        } // void Main
                
    } // class Program
} // namespace C__for_krpc
