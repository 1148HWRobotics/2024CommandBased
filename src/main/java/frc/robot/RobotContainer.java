// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Auto.Positioning.FieldPositioning;
import frc.robot.Auto.Positioning.Position;
import frc.robot.Components.Carriage;
import frc.robot.Components.Climb;
import frc.robot.Components.Elevator;
import frc.robot.Components.Shooter;
import frc.robot.Core.Time;
import frc.robot.Devices.BetterPS4;
import frc.robot.Devices.BinarySensor;
import frc.robot.Devices.Imu;
import frc.robot.Devices.LimeLight;
import frc.robot.Devices.Motor.Falcon;
import frc.robot.Drive.PositionedDrive;
import frc.robot.Util.AngleMath;
import frc.robot.Util.Container;
import frc.robot.Util.DeSpam;
import frc.robot.Util.Lambda;
import frc.robot.Util.MathPlus;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PIDConstant;
import frc.robot.Util.PIDController;
import frc.robot.Util.Promise;
import frc.robot.Util.ScaleInput;
import frc.robot.Util.Vector2;

public class RobotContainer {
  public final static boolean isDriveDisabled = false;
  final static boolean shotDuringAuton = false;

  BetterPS4 con = SubsystemInit.con();
  Joystick joystick = SubsystemInit.joystick();
  PositionedDrive drive = SubsystemInit.drive();
  LimeLight shooterLimeLight = SubsystemInit.shooterLimelight();
  LimeLight intakeLimeLight = SubsystemInit.intakeLimelight();
  Imu imu = SubsystemInit.imu();
  Shooter shooter = SubsystemInit.shooter();
  Climb climb = SubsystemInit.climb();
  Elevator elevator = SubsystemInit.elevator();
  Falcon intake = SubsystemInit.intake();
  BinarySensor intakeSensor = SubsystemInit.intakeSensor();
  Carriage carriage = SubsystemInit.carriage(intakeSensor);
  FieldPositioning fieldPositioning = SubsystemInit.fieldPositioning(drive, imu, shooterLimeLight, new Vector2(0, 0));

  DeSpam dSpam = new DeSpam(0.5);

  static Vector2 speakerPosition() {
    Vector2 speakerPosition = new Vector2(SubsystemInit.isRed() ? 337.87 : -337.87, 53.58);
    return speakerPosition;
  }

  public RobotContainer() {
    SubsystemInit.initializeAutoBuilder(drive, fieldPositioning);
  }

  Lambda teleop() {
    PIDController turnPD = new PIDController(new PDConstant(0.4, 0.05));
    PIDController targetingPID = new PIDController(new PIDConstant(1, 0, 0));

    final Container<Boolean> isAutoAimOn = new Container<>(true);
    final Container<Boolean> isShooting = new Container<>(false);
    final Container<Boolean> inCarriage = new Container<>(!shotDuringAuton ? true : false);
    final Container<Boolean> inIntake = new Container<>(false);
    final Container<Boolean> isLimelightFlashing = new Container<>(false);

    return () -> {

      if (!intakeSensor.get()) {
        inIntake.val = true;
      }

      if (intakeSensor.get() && inIntake.val) {
        inIntake.val = false;
        inCarriage.val = true;
      }

      if (isShooting.val) {
        // do nothing
      } else if (con.getL2Button() && !inCarriage.val && elevator.isDown()) {
        // intake
        carriage.intake();
        intake.setVelocity(0.3 * 360);
      } else if (con.getCrossButton()) {
        // outtake
        inCarriage.val = false;
        carriage.outTake();
        intake.setVoltage(-6);
      } else if (!elevator.isDown() && con.getR1Button()) {
        // outtake but into amp
        carriage.outTake();
        intake.setVoltage(0);
        inCarriage.val = false;
      } else {
        // do nothing
        carriage.stop();
        intake.setVoltage(0);
      }

      if (elevator.isDown() && con.getR2ButtonPressed()) {
        shooter.toggleSpinning();
        if (shooter.isSpinning()) {
          carriage.prepShot();
        } else {
          carriage.unPrepShot();
        }
        isLimelightFlashing.val = !isLimelightFlashing.val;
        if (isLimelightFlashing.val) {
          shooterLimeLight.setLEDState(3);
        } else {
          shooterLimeLight.setLEDState(1);
        }
      }

      // Arrow buttons control
      if (con.povChanged()) {
        switch (con.getPOV()) {
          case 0: // Button Up
            shooter.toggleSpinning();
            break;
          case 90: // Button Right
            // Disable Auto Aim
            isAutoAimOn.val = !isAutoAimOn.val;
            break;
          case 180: // Button Down
            // Toggle auto fw
            break;
          case 270: // Button Left
            shooterLimeLight.setCamMode(!shooterLimeLight.getCamMode());
            break;
        }
      }

      dSpam.exec(() -> {
        System.out.println(fieldPositioning.getPosition() + " " + fieldPositioning.getTurnAngle());
      });

      // climber, can be accessed by joystick or controller
      if (joystick.getRawButtonPressed(2) || con.getTriangleButtonPressed())
        if (climb.isDown())
          climb.moveUp();
        else
          climb.moveDown();

      // elevator
      if (con.getL1ButtonPressed())
        if (elevator.getTarget() == 0)
          elevator.moveUp();
        else
          elevator.moveDown();

      var targetPos = speakerPosition();
      // .add(fieldPositioning.getFieldRelativeSpeed().multiply(0.4));

      final var displacementFromTar = targetPos
          .minus(fieldPositioning.getPosition());
        isShooting.val = false;
      

      var distToTar = displacementFromTar.getMagnitude();

      var correction = -turnPD.solve(AngleMath.getDelta(displacementFromTar.getTurnAngleDeg() - 90,
          fieldPositioning.getTurnAngle()));

      var pointingTar = shooter.isSpinning() && elevator.isDown() && isAutoAimOn.val;

      boolean canAutoShoot = MathPlus.withinBounds(displacementFromTar.getMagnitude(), 120.0, 98.0)
          && correction < 0.38;

      if (elevator.isDown() && shooter.isAtVelocity()
          && canAutoShoot) {
        carriage.shoot();
        inCarriage.val = false;
        isShooting.val = true;
      }

      // System.out.println("autoshoot: " + canAutoShoot + " correction: " +
      // correction);
      if (elevator.isDown() && shooter.isSpinning() && shooter.isAtVelocity()) {
        // if (canAutoShoot)
        // carriage.setVoltage(12);

        // if (con.getR1ButtonPressed() && !isShooting.val) {
        //   System.out.println("gonna shootingS");
        //   isShooting.val = true;
        //   shooter.spin();

        //   carriage.setVoltage(8);
        //   Time.timeout(() -> {
        //     System.out.println("shootingS");
        //     carriage.setVoltage(0);
        //     isShooting.val = false;
        //     inCarriage.val = false;
        //   }, 2);
        // }
      }

      if ((con.getLeftStick().getMagnitude() + Math.abs(con.getRightX()) > 0.1) || pointingTar)
        drive.power(ScaleInput.curve(con.getLeftStick().getMagnitude(), 1.5) * 12.0, // voltage
            (con.getLeftStick().getAngleDeg()) - fieldPositioning.getTurnAngle()
                + ((SubsystemInit.isRed()) ? 0 : 180), // go angle
            (!pointingTar) ? con.getRightX() * -12.0 : correction, // change back to
                                                                   // correction
            // turn voltage
            // 0,
            false);
      else
        drive.power(0, 0, 0);
    };
  }

  public Command getTeleopCommand() {
    var periodic = new Container<Lambda>();
    return new Command() {
      @Override
      public void initialize() {
        periodic.val = teleop();
      }

      @Override
      public void execute() {
        periodic.val.run();
      }
    };
  }

  public Command getAutonomousCommand() {
    var auto = new PathPlannerAuto("New Auto");
    // fieldPositioning.setStartPosition(new
    // Position(auto.getStaringPoseFromAutoFile("New
    // Auto").getRotation().getDegrees(), new
    // Vector2(auto.getStaringPoseFromAutoFile("New Auto").getX(),
    // auto.getStaringPoseFromAutoFile("New Auto").getY())));
    return auto;
  }
}