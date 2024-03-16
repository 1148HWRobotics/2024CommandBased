package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Auto.FieldPositioning;
import frc.robot.Auto.Position;
import frc.robot.Components.Carriage;
import frc.robot.Components.Elevator;
import frc.robot.Components.Shooter;
import frc.robot.Devices.AbsoluteEncoder;
import frc.robot.Devices.BetterPS4;
import frc.robot.Devices.BinarySensor;
import frc.robot.Devices.Imu;
import frc.robot.Devices.LimeLight;
import frc.robot.Devices.Motor.TalonFX;
import frc.robot.Drive.PositionedDrive;
import frc.robot.Drive.SwerveModule;
import frc.robot.Drive.SwerveModulePD;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PIDConstant;
import frc.robot.Util.PIDController;
import frc.robot.Util.PWIDConstant;
import frc.robot.Util.PWIDController;
import frc.robot.Util.Vector2;

public class SubsystemInit {
    public static Boolean isRed() {
        boolean isRed = false;
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            isRed = alliance.get() == DriverStation.Alliance.Red ? true : false;
        }
        return isRed;
    }

    static PositionedDrive drive() {
        var placeholderTurnPID = new PDConstant(0, 0);

        var moduleGoPID = new PWIDController(
                new PWIDConstant(0.1, 0, 0, 0));

        var leftBackEncoder = new AbsoluteEncoder(22, "drive", -44.64843, false).setOffset(-90);
        var leftBackTurn = new TalonFX(2, "drive", true);
        var leftBackGo = new TalonFX(1, "drive", false);
        var leftBackRaw = new SwerveModule(leftBackTurn, leftBackGo, moduleGoPID);
        var leftBack = new SwerveModulePD(leftBackRaw, placeholderTurnPID, leftBackEncoder);

        var rightBackEncoder = new AbsoluteEncoder(21, "drive", 9.93164, false).setOffset(-90);
        var rightBackTurn = new TalonFX(8, "drive", true);
        var rightBackGo = new TalonFX(7, "drive", false);
        var rightBackRaw = new SwerveModule(rightBackTurn, rightBackGo, moduleGoPID);
        var rightBack = new SwerveModulePD(rightBackRaw, placeholderTurnPID, rightBackEncoder);

        var leftFrontEncoder = new AbsoluteEncoder(23, "drive", 45.96679, false).setOffset(-90);
        var leftFrontTurn = new TalonFX(4, "drive", true);
        var leftFrontGo = new TalonFX(3, "drive", false);
        var leftFrontRaw = new SwerveModule(leftFrontTurn, leftFrontGo, moduleGoPID);
        var leftFront = new SwerveModulePD(leftFrontRaw, placeholderTurnPID, leftFrontEncoder);

        var rightFrontEncoder = new AbsoluteEncoder(24, "drive", -99.66796, false).setOffset(-90);
        var rightFrontTurn = new TalonFX(6, "drive", true);
        var rightFrontGo = new TalonFX(5, "drive", false);
        var rightFrontRaw = new SwerveModule(rightFrontTurn, rightFrontGo, moduleGoPID);
        var rightFront = new SwerveModulePD(rightFrontRaw, placeholderTurnPID, rightFrontEncoder);

        PositionedDrive drive = new PositionedDrive(leftFront, rightFront, leftBack, rightBack, 23.0, 23.0);

        drive.setAlignmentThreshold(0.5);

        var constants = new PDConstant(-0.1, -0).withMagnitude(0.5);
        drive.setConstants(constants);

        return drive;
    }

    static LimeLight shooterLimelight() {
        var limeLightA = new LimeLight("limelight-a");
        limeLightA.setCamMode(true);
        limeLightA.setLEDState(1);
        return limeLightA;
    }

    static LimeLight intakeLimelight() {
        var limeLightB = new LimeLight("limelight-b");
        limeLightB.setCamMode(false);
        limeLightB.setLEDState(1);
        return limeLightB;
    }

    static Imu imu() {
        Imu imu = new Imu(18);
        return imu;
    }

    static Shooter shooter() {
        Shooter shooter = new Shooter(
                new TalonFX(12, false),
                new TalonFX(10, true));
        return shooter;

    }

    static Elevator elevator() {
        BinarySensor elevatorDownSensor = new BinarySensor(0);
        TalonFX f1 = new TalonFX(9, false).withMaxVoltage(12);
        TalonFX f2 = new TalonFX(13, true).withMaxVoltage(12);
        var elevator = new Elevator(
                f1, // left
                f2, // right
                new PIDController(new PIDConstant(0.13, 0.0, 0.0)),
                elevatorDownSensor);
        return elevator;
    }

    static TalonFX intake() {
        var intake = new TalonFX(14, false);
        intake.setVelocityPD(new PIDController(new PDConstant(0.1, 0.0)));
        return intake;
    }

    static Carriage carriage(BinarySensor intakeSensor) {
        var motor = new TalonFX(11, true);
        motor.setVelocityPD(new PIDController(new PDConstant(0.1, 0.0)));
        return new Carriage(motor, intakeSensor);
    }

    static BinarySensor intakeSensor() {
        BinarySensor intakeSensor = new BinarySensor(2);
        return intakeSensor;
    }

    static FieldPositioning fieldPositioning(PositionedDrive drive, Imu imu, LimeLight limeLight, Vector2 startPos) {
        FieldPositioning fieldPositioning = new FieldPositioning(drive, imu, limeLight,
                new Position(isRed() ? 0 : 180, new Vector2(0, 0)));
        return fieldPositioning;
    }

    // input

    static BetterPS4 con() {
        BetterPS4 con = new BetterPS4(0);
        return con;
    }

    static Joystick joystick() {
        Joystick joystick = new Joystick(1);
        return joystick;
    }
}
