// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Devices.Motor.TalonFX;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private Command m_teleopCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    m_teleopCommand = m_robotContainer.getTeleopCommand();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (m_teleopCommand != null) {
      m_teleopCommand.cancel();
    }
    m_autonomousCommand = m_robotContainer
        .getAutonomousCommand(SmartDashboard.getString("Auto Selector", "pathplanner"));

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_teleopCommand.schedule();
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
    m_robotContainer.autoDrive.setStates(new SwerveModuleState[] { new SwerveModuleState(1.0, new Rotation2d(0)),
        new SwerveModuleState(1.0, new Rotation2d(0)), new SwerveModuleState(1.0, new Rotation2d(0)),
        new SwerveModuleState(1.0, new Rotation2d(0)) });
  }

  @Override
  public void testPeriodic() {
    m_robotContainer.autoDrive.setStates(new SwerveModuleState[] { new SwerveModuleState(1.0, new Rotation2d(0)),
        new SwerveModuleState(1.0, new Rotation2d(0)), new SwerveModuleState(1.0, new Rotation2d(0)),
        new SwerveModuleState(1.0, new Rotation2d(0)) });
  }

  @Override
  public void testExit() {
  }
}
