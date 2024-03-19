package frc.robot.Auto;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Core.Time;
import frc.robot.Devices.Imu;
import frc.robot.Devices.LimeLight;
import frc.robot.Drive.PositionedDrive;
import frc.robot.Util.Vector2;

public class AutonomousPositioning extends SubsystemBase implements PositioningSystem {
    PositionedDrive drive;
    Imu imu;
    final double correctionTime = 0.5;
    LinkedList<Position> positionHistory = new LinkedList<>();

    public AutonomousPositioning(PositionedDrive drive, Imu imu, Position startPos) {
        this.drive = drive;
        this.imu = imu;
        positionHistory.add(0, startPos);
    }

    public void setStartPosition(Position position) {
        positionHistory.set(0, position);
    }

    public void reset(Pose2d pose) {
        positionHistory.set(0, new Position(pose.getRotation().getDegrees(), new Vector2(pose.getX(), pose.getY())));
    }

    public ChassisSpeeds getRobotRelativeSpeeds() {
        double rotationSpeed = (positionHistory.get(0).angle - positionHistory.get(1).angle) / 0.02;
        Vector2 translationSpeed = (positionHistory.get(0).position.minus(positionHistory.get(1).position))
                .multiply(1 / 0.02).rotate(-getTurnAngle());

        return new ChassisSpeeds(translationSpeed.x, translationSpeed.y, rotationSpeed / 180 * Math.PI);
    }

    public double getTurnAngle() {
        return positionHistory.getFirst().angle;
    }

    public Pose2d getPose2d() {
        return new Pose2d(new Translation2d(getPosition().x, getPosition().y),
                new Rotation2d(Units.degreesToRadians(getTurnAngle())));
    }

    public Vector2 getPosition() {
        return positionHistory.getFirst().position;
    }

    public void periodic() {
        Position lastPosition = positionHistory.getFirst();
        final double currentAngle = lastPosition.angle + imu.getYawDeltaThisTick();
        positionHistory.add(0, new Position(
                currentAngle,
                lastPosition.position.add(drive.movementSinceLastTick.rotate(currentAngle - 90))));

        // makes sure position history doesn't get too long
        if (positionHistory.size() > 5 / 0.02) {
            positionHistory.removeLast();
        }

    }

}
