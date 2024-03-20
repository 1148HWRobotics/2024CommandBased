package frc.robot.Auto;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Devices.Imu;
import frc.robot.Drive.SwerveModulePD;
import frc.robot.Util.AngleMath;
import frc.robot.Util.DeSpam;
import frc.robot.Util.PDConstant;
import frc.robot.Util.Vector2;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PathPlannerDrive extends SubsystemBase {

    /**
     * Drive is a class representing the swerve drive system of a robot.
     * It manages the coordination of the swerve modules for driving and turning
     * movements.
     */

    // Swerve modules for each corner of the robot.
    public SwerveModule frontLeft;
    protected SwerveModule frontRight;
    public SwerveModule backLeft;
    protected SwerveModule backRight;

    // Dimensions of the robot.
    protected double widthInches;
    protected double lengthInches;
    protected double circumferenceInches; // Calculated circumference for turning calculations.

    // The minimum alignment before driving starts.
    private double alignmentThreshold = 1;
    private Imu gyro;

    /**
     * Sets the threshold for how closely aligned the modules need to be to their
     * target positions before driving begins.
     * 
     * @param newThreshold A value between 0 (exclusive) and 1 (inclusive)
     *                     representing the alignment threshold.
     */
    public void setAlignmentThreshold(double newThreshold) {
        if (newThreshold <= 0 || newThreshold > 1)
            throw new Error("Threshold must be in range (0, 1]");
        this.alignmentThreshold = newThreshold;
    }

    SwerveModule[] modules;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    private Field2d field = new Field2d();

    /**
     * Constructor for Drive that sets up the swerve modules and the robot's
     * dimensions.
     * 
     * @param frontLeft    The front-left swerve module.
     * @param frontRight   The front-right swerve module.
     * @param backLeft     The back-left swerve module.
     * @param backRight    The back-right swerve module.
     * @param widthInches  The width of the robot in inches.
     * @param lengthInches The length of the robot in inches.
     */

    public class Constants {
        public static final double maxModuleSpeed = 4.5; // M/S

        public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
                new PIDConstants(5.0, 0, 0), // Translation constants
                new PIDConstants(5.0, 0, 0), // Rotation constants
                maxModuleSpeed,
                0.7112, // Drive base radius (distance from center to furthest module)
                new ReplanningConfig());
    }

    public PathPlannerDrive(SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft,
            SwerveModule backRight, Imu imu) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        this.circumferenceInches = 2 * Math.PI
                * Math.sqrt((widthInches * widthInches + lengthInches * lengthInches) / 2);
        modules = new SwerveModule[] {
                frontLeft, frontRight, backLeft, backRight
        };
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d[] { new Translation2d(-0.3556, 0.3556), new Translation2d(0.3556, 0.3556),
                        new Translation2d(-0.3556, -0.3556), new Translation2d(0.3556, -0.3556) });

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::resetPose,
                this::getSpeeds,
                this::driveRobotRelative,
                Constants.pathFollowerConfig,
                () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red
                    // alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                this);
        PathPlannerLogging.setLogActivePathCallback((poses) -> field.getObject("path").setPoses(poses));

        SmartDashboard.putData("Field", field);
        this.gyro = imu;
        odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d(), getPositions());

    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getPosition(true);
        }
        return positions;
    }

    public void fromChassisSpeeds(ChassisSpeeds speeds) {
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
        for (int i = 0; i < modules.length; i++) {
            modules[i].apply(states[i], DriveRequestType.Velocity);
        }
    }

    @Override
    public void periodic() {

        odometry.update(gyro.getRotation2d(), getPositions());

        field.setRobotPose(getPose());
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }

    public ChassisSpeeds getSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        driveRobotRelative(ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getPose().getRotation()));
    }

    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);

        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
        setStates(targetStates);
    }

    public void setStates(SwerveModuleState[] targetStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.maxModuleSpeed);

        for (int i = 0; i < modules.length; i++) {
            modules[i].apply(targetStates[i], DriveRequestType.Velocity, SteerRequestType.MotionMagic);
        }
    }

    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getCurrentState();
        }
        return states;
    }

    /**
     * Basic simulation of a swerve module, will just hold its current state and not
     * use any hardware
     */
    class SimSwerveModule {
        private SwerveModulePosition currentPosition = new SwerveModulePosition();
        private SwerveModuleState currentState = new SwerveModuleState();

        public SwerveModulePosition getPosition() {
            return currentPosition;
        }

        public SwerveModuleState getState() {
            return currentState;
        }

        public void setTargetState(SwerveModuleState targetState) {
            // Optimize the state
            currentState = SwerveModuleState.optimize(targetState, currentState.angle);

            currentPosition = new SwerveModulePosition(
                    currentPosition.distanceMeters + (currentState.speedMetersPerSecond * 0.02), currentState.angle);
        }
    }

    /**
     * Basic simulation of a gyro, will just hold its current state and not use any
     * hardware
     */
}
