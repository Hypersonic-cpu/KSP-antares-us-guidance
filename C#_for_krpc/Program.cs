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
            String[] str = { F.ReadLine(), F.ReadLine(), F.ReadLine() };
            double r = double.Parse(str[0].Split('=').Last());
            double v = double.Parse(str[1].Split('=').Last());
            double rad = double.Parse(str[2].Split('=').Last());
            var TARGET = new Orbital(r, v, rad);
            Console.WriteLine($"\n[TARGET] =\nRADIUS\t\t\t{TARGET.radius}\nVELOCITY\t\t{TARGET.velocity}\nFLIGHT PATH ANGLE\t{TARGET.pathAngle}");

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
                // ...

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
                Console.WriteLine("[Action] Payload Fairing Jettison.");
                // Payload fairing jettison
                vessel.Control.ActivateNextStage();
                Thread.Sleep(2000);

                Simulator sim_ = new Simulator(radius(), speed(), pathAngle(), pitch(), massStream.Get() - 2300, 0, TARGET);
                do
                {
                    double lastExcV = -1000;
                    do
                    {
                        Thread.Sleep(50);
                        sim_.RefreshSimulator(radius(), speed(), pathAngle(), pitch(), -4);
                        double excV = sim_.TerminalVelocity() - TARGET.velocity;
                        Console.WriteLine($"ExcV {Math.Round(excV, 1)}");
                        // +75m/s offset
                        if (100 < excV && excV < Vehicle.V_EXC_BEFORE_IGN && excV > lastExcV) break;
                        if (excV < 100 && excV < lastExcV) break; // low delta V
                        lastExcV = excV;
                    } while (true);
                } while (false);
                Console.WriteLine("[Action] Interstage Seperation.");
                // Interstage seperation
                vessel.Control.ActivateNextStage();
                Thread.Sleep(4000);

                Console.WriteLine("[Action] Upper Stage Ignition.");
                // Stage2 Ignition
                double T_IGN = utStream.Get();
                vessel.Control.ActivateNextStage();
                short fltPhase = 0;

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

                    vessel.Control.RCS = false;

                    if (fltPhase == 0)
                    {
                        sim.RefreshSimulator(radius_, speed_, pathAngle_, pitch_, T);
                        excVelocity = sim.TerminalVelocity() - TARGET.velocity;
                        
                        vehicle.GenerateCurve();
                        newPitch = vehicle.GetPitch(thrustStream.Get(), excVelocity - Vehicle.V_ENGM_END);
                        
                        if (excVelocity <= Vehicle.V_ENGM_END || T > Vehicle.T_ENGM - 10)
                        {
                            Console.WriteLine("[Guidance] Energy management ceased.");
                            fltPhase = 1;
                        }
                        Console.Write($"ExcV {Math.Round(excVelocity, 1)}\t");
                        //Console.WriteLine(vehicle.guidanceCurve);
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
                    }
                    else
                    {
                        newPitch = vehicle.GetPitch(thrustStream.Get());
                    }

                    Console.WriteLine($"PitchCmd {Math.Round(Equ.Deg(newPitch), 3)}");

                    var vG = spaceCenter.TransformDirection(Equ.UnitVector(grvStream.Get()), fixFrame, ctlFrame);
                    double vHeadingAngle = Math.Acos(vG.Item2 / Math.Sqrt(vG.Item2 * vG.Item2 + vG.Item3 * vG.Item3));
                    vessel.AutoPilot.TargetDirection = System.Tuple.Create
                        (
                            Math.Sin(newPitch), 
                            Math.Cos(newPitch) * Math.Cos(vHeadingAngle), 
                            Math.Cos(newPitch) * Math.Sin(vHeadingAngle)
                        );

                    Console.WriteLine($"T+{Math.Round(T, 1)}\tPitch{Math.Round(Equ.Deg(pitch_), 2)}\t" +
                        $"V{Math.Round(speed_, 1)},\t{Math.Round(Equ.Deg(pathAngle_), 2)}\t Mass{Math.Round(mass_, 0)}");
                    Thread.Sleep(Vehicle.LOOP_SLEEP_T);
                } // while loop

                Console.WriteLine("[Action] Upper Stage Burn-out.");
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
                Console.WriteLine("[Guidance] End.");
                vessel.AutoPilot.Wait();
                Console.WriteLine("[Action] Cygnus Seperation.");
                vessel.Control.ActivateNextStage();
                vessel.AutoPilot.Disengage();
            } // using conn
        } // void Main
                
    } // class Program
} // namespace C__for_krpc
