package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Devices.AbsoluteEncoder;
import frc.robot.Devices.Motor.Falcon;
import frc.robot.Drive.SwerveModule;
import frc.robot.Drive.SwerveModulePD;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PWIDConstant;
import frc.robot.Util.PWIDController;

public class TestRobot extends TimedRobot {
    SwerveModulePD[] modules;

    @Override
    public void robotInit() {
        var placeholderTurnPID = new PDConstant(0, 0);

        var moduleGoPID = new PWIDController(
                new PWIDConstant(1, 0, 0, 0));

        var leftBackEncoder = new AbsoluteEncoder(22, "drive", -44.64843, false).setOffset(-90);
        var leftBackTurn = new Falcon(2, "drive", true);
        var leftBackGo = new Falcon(1, "drive", false);
        var leftBackRaw = new SwerveModule(leftBackTurn, leftBackGo, moduleGoPID);
        var leftBack = new SwerveModulePD(leftBackRaw, placeholderTurnPID, leftBackEncoder);

        var rightBackEncoder = new AbsoluteEncoder(21, "drive", 9.93164, false).setOffset(-90);
        var rightBackTurn = new Falcon(8, "drive", true);
        var rightBackGo = new Falcon(7, "drive", false);
        var rightBackRaw = new SwerveModule(rightBackTurn, rightBackGo, moduleGoPID);
        var rightBack = new SwerveModulePD(rightBackRaw, placeholderTurnPID, rightBackEncoder);

        var leftFrontEncoder = new AbsoluteEncoder(23, "drive", 45.96679, false).setOffset(-90);
        var leftFrontTurn = new Falcon(4, "drive", true);
        var leftFrontGo = new Falcon(3, "drive", false);
        var leftFrontRaw = new SwerveModule(leftFrontTurn, leftFrontGo, moduleGoPID);
        var leftFront = new SwerveModulePD(leftFrontRaw, placeholderTurnPID, leftFrontEncoder);

        var rightFrontEncoder = new AbsoluteEncoder(24, "drive", -99.66796, false).setOffset(-90);
        var rightFrontTurn = new Falcon(6, "drive", true);
        var rightFrontGo = new Falcon(5, "drive", false);
        var rightFrontRaw = new SwerveModule(rightFrontTurn, rightFrontGo, moduleGoPID);
        var rightFront = new SwerveModulePD(rightFrontRaw, placeholderTurnPID, rightFrontEncoder);

        modules = new SwerveModulePD[] {rightFront, leftFront, leftBack, rightBack};
    }

    @Override
    public void teleopPeriodic() {
        
    }
}
