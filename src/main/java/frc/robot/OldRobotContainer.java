// package frc.robot;

// import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Joystick;
// import edu.wpi.first.wpilibj.DriverStation.Alliance;
// import edu.wpi.first.wpilibj2.command.Subsystem;

// import java.util.Optional;

// import frc.robot.Auto.Positioning.FieldPositioning;
// import frc.robot.Auto.Positioning.Position;
// import frc.robot.Auto.Positioning.ResettablePosition;
// import frc.robot.Components.Climb;
// import frc.robot.Components.Elevator;
// import frc.robot.Components.Shooter;
// import frc.robot.Core.RobotPolicy;
// import frc.robot.Devices.AbsoluteEncoder;
// import frc.robot.Devices.BetterPS4;
// import frc.robot.Devices.BinarySensor;
// import frc.robot.Devices.Imu;
// import frc.robot.Devices.LimeLight;
// import frc.robot.Devices.Motor.Falcon;
// import frc.robot.Drive.PositionedDrive;
// import frc.robot.Drive.SwerveModule;
// import frc.robot.Drive.SwerveModulePD;
// import frc.robot.Util.AngleMath;
// import frc.robot.Util.Container;
// import frc.robot.Util.PDConstant;
// import frc.robot.Util.PIDConstant;
// import frc.robot.Util.PIDController;
// import frc.robot.Util.PWIDConstant;
// import frc.robot.Util.PWIDController;
// import frc.robot.Util.Promise;
// import frc.robot.Util.ScaleInput;
// import frc.robot.Util.Vector2;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;

// public class RobotContainer {
//     public static boolean isDriveDisabled = false;
//     public static boolean shotDuringAuton = false;

//     public static boolean isRed() {
//         boolean isRed = true;
//         var alliance = DriverStation.getAlliance();
//         if (alliance.isPresent()) {
//             isRed = alliance.get() == DriverStation.Alliance.Red ? true : false;
//         }
//         return isRed;
//     }

//     public static Vector2 speakerPosition() {
//         return new Vector2(isRed() ? 327.87 : -327.87, 60.0);
//     }

//     static RobotPolicy init() {
//         // Controllers
//         BetterPS4 con = new BetterPS4(0);
//         Joystick joystick = new Joystick(1);

//         // Drive + Positioning
//         LimeLight limeLightA;
//         LimeLight limeLightB;
//         Imu imu = new Imu(18);
//         PIDController turnPD = new PIDController(new PDConstant(0.4, 0.4));

      

//         FieldPositioning fieldPositioning = new FieldPositioning(drive, imu, limeLightA,
//                 new Position(isRed() ? 180 : 0, new Vector2(0, 0)));

//         // Subsystems
//         Shooter shooter;
//         BinarySensor intakeSensor = new BinarySensor(2); // aka beamBreak
//         Elevator elevator;
//         Climb climber;
//         Falcon intake;
//         Falcon carriage;

//         { // Subsystem initialization

//             // shooter
//             shooter = new Shooter(
//                     new Falcon(12, false),
//                     new Falcon(10, true),
//                     new PDConstant(0.2, 0),
//                     con);

//             // elevator
//             BinarySensor elevatorDownSensor = new BinarySensor(0);
//             elevator = new Elevator(
//                     new Falcon(9, false).withMaxVoltage(4), // left
//                     new Falcon(13, true).withMaxVoltage(4), // right
//                     new PIDController(new PIDConstant(0.1, 0, 0.000)),
//                     elevatorDownSensor);

//             // climber
//             Falcon leftGrippy = new Falcon(19, false).withMaxVoltage(7);
//             Falcon rightGrippy = new Falcon(20, true).withMaxVoltage(7);
//             climber = new Climb(leftGrippy, rightGrippy, new PDConstant(0.01, 0));

//             // intake
//             intake = new Falcon(14, false);
//             intake.setVelocityPD(new PIDController(new PDConstant(0.1, 0.0)));

//             // carriage
//             carriage = new Falcon(11, true);
//             carriage.setVelocityPD(new PIDController(new PDConstant(0.1, 0.0)));
//         }

//         var resettablePos = new ResettablePosition(fieldPositioning);
//         { // Configures PathPlanner
//             AutoBuilder.configureHolonomic(
//                     () -> resettablePos.getPosition(), // Robot pose supplier
//                     (pos) -> resettablePos.reset(pos), // Method to reset odometry (will be
//                     // called if your auto has a
//                     // starting
//                     // pose)
//                     () -> fieldPositioning.getRobotRelativeSpeeds(), // ChassisSpeeds supplier.
//                     // MUST BE ROBOT RELATIVE
//                     (speeds) -> drive.fromChassisSpeeds(speeds), // Method that will drive the
//                     // robot given ROBOT RELATIVE
//                     // ChassisSpeeds
//                     new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should
//                             // likely live in your
//                             // Constants class
//                             new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
//                             new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
//                             4.5, // Max module speed, in m/s
//                             0.4, // Drive base radius in meters. Distance from robot center to furthest
//                             // module.
//                             new ReplanningConfig() // Default path replanning config. See the API for the
//                     // options here
//                     ),
//                     () -> {
//                         // Boolean supplier that controls when the path will be mirrored for the red
//                         // alliance
//                         // This will flip the path being followed to the red side of the field.
//                         // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

//                         Optional<Alliance> alliance = DriverStation.getAlliance();
//                         if (alliance.isPresent()) {
//                             return alliance.get() == DriverStation.Alliance.Red;
//                         }
//                         return false;
//                     },
//                     new Subsystem() {

//                     } // Reference to this subsystem to set requirements
//             );
//         }

//         return new RobotPolicy() {

//             public void teleop() {

//                 drive.setAlignmentThreshold(0.5);

//                 var constants = new PDConstant(-0.1, -0).withMagnitude(0.5);
//                 drive.setConstants(constants);

//                 final Container<Boolean> isAutoAimOn = new Container<>(true);
//                 final Container<Boolean> isShooting = new Container<>(false);
//                 final Container<Boolean> inCarrage = new Container<>(!shotDuringAuton ? true : false);
//                 final Container<Boolean> inIntake = new Container<>(false);
//                 final Container<Boolean> isLimelightFlashing = new Container<>(false);

//                 Scheduler.registerTick(() -> {

//                     if (!intakeSensor.get()) {
//                         inIntake.val = true;
//                     }

//                     if (intakeSensor.get() && inIntake.val) {
//                         inIntake.val = false;
//                         inCarrage.val = true;
//                     }

//                     if (inCarrage.val)
//                         System.out.println("ITS INSIDE OF ME");
//                     else
//                         System.out.println("OUTSIDE DADDY");

//                     if (isShooting.val) {
//                         // do nothing
//                     } else if (con.getL2Button() && !inCarrage.val && elevator.isDown()) {
//                         // intake
//                         carriage.setVelocity(0.3 * 360);
//                         intake.setVelocity(0.3 * 360);
//                     } else if (con.getCrossButton()) {
//                         // outtake
//                         inCarrage.val = false;
//                         carriage.setVoltage(-12);
//                         intake.setVoltage(-6);
//                     } else if (!elevator.isDown() && con.getR1Button()) {
//                         // outtake but into amp
//                         carriage.setVoltage(-12);
//                         intake.setVoltage(0);
//                         inCarrage.val = false;
//                     } else {
//                         // do nothing
//                         carriage.setVelocity(0);
//                         intake.setVoltage(0);
//                     }

//                     if (elevator.isDown() && con.getR2ButtonPressed()) {
//                         shooter.toggleSpinning();
//                         isLimelightFlashing.val = !isLimelightFlashing.val;
//                         if (isLimelightFlashing.val) {
//                             limeLightA.setLEDState(3);
//                         } else {
//                             limeLightA.setLEDState(1);
//                         }
//                     }

//                     if (con.getR1ButtonPressed() && elevator.isDown()) {
//                         isShooting.val = true;
//                         var startShooting = Promise.timeout(shooter.isSpinning() ? 0 : 1);
//                         shooter.spin();
//                         startShooting.then(() -> {
//                             // System.out.println("dist: " + shooter.distance + " vel: " + shooter.vel);
//                             carriage.setVoltage(8);
//                             Scheduler.setTimeout(() -> {
//                                 carriage.setVoltage(0);
//                                 isShooting.val = false;
//                                 inCarrage.val = false;
//                             }, 2);
//                         });
//                     }

//                     // Arrow buttons control
//                     if (con.povChanged()) {
//                         switch (con.getPOV()) {
//                             case 0: // Button Up
//                                 shooter.toggleSpinning();
//                                 break;
//                             case 90: // Button Right
//                                 // Disable Auto Aim
//                                 isAutoAimOn.val = !isAutoAimOn.val;
//                                 break;
//                             case 180: // Button Down
//                                 // Toggle auto fw
//                                 break;
//                             case 270: // Button Left
//                                 limeLightA.setCamMode(!limeLightA.getCamMode());
//                                 break;
//                         }
//                     }

//                     // climber, can be accessed by joystick or controller
//                     if (joystick.getRawButtonPressed(2) || con.getTriangleButtonPressed())
//                         if (climber.isDown())
//                             climber.moveUp();
//                         else
//                             climber.moveDown();

//                     // elevator
//                     if (con.getL1ButtonPressed())
//                         if (elevator.getTarget() == 0)
//                             elevator.moveUp();
//                         else
//                             elevator.moveDown();

//                     // 2 82.1238544369317, 32.670822887202704
//                     final var displacementFromTar = speakerPosition()
//                             .minus(fieldPositioning.getPosition());

//                     var distToTar = displacementFromTar.getMagnitude();
//                     shooter.setDistance(distToTar);

//                     var correction = -turnPD.solve(AngleMath.getDelta(displacementFromTar.getTurnAngleDeg() + 90,
//                             fieldPositioning.getTurnAngle()));

//                     var pointingTar = shooter.isSpinning() && elevator.isDown() && isAutoAimOn.val;

//                     if ((con.getLeftStick().getMagnitude() + Math.abs(con.getRightX()) > 0.1) || pointingTar)
//                         drive.power(ScaleInput.curve(con.getLeftStick().getMagnitude(), 1.5) * 12.0, // voltage
//                                 (con.getLeftStick().getAngleDeg()) - fieldPositioning.getTurnAngle()
//                                         + ((isRed()) ? 0 : 180), // go angle
//                                 (!pointingTar) ? con.getRightX() * -12.0 : correction, // change back to
//                                                                                        // correction
//                                 // turn voltage
//                                 // 0,
//                                 false);
//                     else
//                         drive.power(0, 0, 0);
//                 });
//             }

//             public void autonomous() {
//                 // drive.setAlignmentThreshold(0.5);

//                 // var constants = new PDConstant(-0.1, -0).withMagnitude(0.5);
//                 // drive.setConstants(constants);

//                 // double target = 107;

//                 // var distancePID = new PIDController(new PIDConstant(0.2, 0, 0, 1.0));

//                 // // limeLight.setCamMode(true);

//                 // Promise.timeout(0).then(() -> {
//                 // drive.power(1, -imu.getTurnAngle() + 180, -0.35);
//                 // return Promise.timeout(4);
//                 // })
//                 // .then(() -> {
//                 // var aimingTimer = Promise.timeout(1);
//                 // Scheduler.registerTick(() -> {
//                 // final var displacementFromTar = speakerPosition()
//                 // .minus(fieldPositioning.getPosition());

//                 // var correction = -turnPD
//                 // .solve(AngleMath.getDelta(displacementFromTar.getTurnAngleDeg() + 90,
//                 // fieldPositioning.getTurnAngle()));

//                 // drive.power(
//                 // aimingTimer.isResolved()
//                 // ? distancePID.solve(target - displacementFromTar.getMagnitude())
//                 // : 0, // voltage
//                 // 90, // go angle
//                 // correction,
//                 // false);

//                 // System.out.println(displacementFromTar.getMagnitude());
//                 // });

//                 // shooter.spin();
//                 // return Promise.timeout(5);
//                 // })
//                 // .then(() -> {
//                 // if (fieldPositioning.hasGottenLimeLightFrame())
//                 // carriage.setVoltage(12);
//                 // shotDuringAuton = true;
//                 // });

//                 var auto = new PathPlannerAuto("New Auto");
//                 auto.schedule();

//                 Scheduler.runTask(new Schedulable() {
//                     @Override
//                     protected void start() {
//                     }

//                     @Override
//                     protected void tick(double dTime) {
//                         CommandScheduler.getInstance().run();
//                     }

//                     @Override
//                     protected void end() {
                        
//                     }
//                 });
//             }

//             public void test() {
//                 // turns off drive for test
//                 drive.setAlignmentThreshold(0.1);
//                 var constants = new PDConstant(-0.01, -0);
//                 drive.setConstants(constants);

//                 climber.disablePD();
//                 Scheduler.registerTick(() -> {
//                     if (con.getTriangleButton())
//                         climber.unsafeAccessLeftMotor().setVoltage(7);
//                     else if (con.getCrossButton())
//                         climber.unsafeAccessLeftMotor().setVoltage(-7);
//                     else
//                         climber.unsafeAccessLeftMotor().setVoltage(0);

//                     if (con.getSquareButton())
//                         climber.unsafeAccessRightMotor().setVoltage(7);
//                     else if (con.getCircleButton())
//                         climber.unsafeAccessRightMotor().setVoltage(-7);
//                     else
//                         climber.unsafeAccessRightMotor().setVoltage(0);
//                 });
//             }

//             @Override
//             public void disabled() {

//             }

//         };
//     }

//     @Override
//     public String toString() {
//         return "RobotContainer [Michael Barr Sucks]";
//     }

//     public static boolean isDriveDisabled() {
//         return isDriveDisabled;
//     }

//     public static void setDriveDisabled(boolean isDriveDisabled) {
//         RobotContainer.isDriveDisabled = isDriveDisabled;
//     }
// }
