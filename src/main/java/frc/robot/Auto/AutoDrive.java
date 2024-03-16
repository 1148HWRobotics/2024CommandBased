package frc.robot.Auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SubsystemInit;
import frc.robot.Drive.PositionedDrive;
import frc.robot.Util.AngleMath;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PIDController;
import frc.robot.Util.Promise;
import frc.robot.Util.SimplePromise;
import frc.robot.Util.Vector2;

public class AutoDrive extends SubsystemBase {
    FieldPositioning positioning;
    Position targetPos = new Position(0, new Vector2(0, 0));
    PositionedDrive drive;

    PIDController xCon;
    PIDController yCon;
    PIDController turnCon;

    public AutoDrive(FieldPositioning positioning, Position targetPos, PositionedDrive drive, PDConstant transCon,
            PDConstant turnCon) {
        this.positioning = positioning;
        moveTo(targetPos.position);
        setAngleTar(targetPos.angle);
        this.drive = drive;

        this.xCon = new PIDController(transCon);
        this.yCon = new PIDController(transCon);
        this.turnCon = new PIDController(turnCon);
    }

    // these core methods use reversing

    public void pointTo(Vector2 position) {
        if (!SubsystemInit.isRed())
            position.x *= -1;
        targetPos.angle = position.minus(positioning.getPosition()).getAngleDeg();
    }

    public void setAngleTar(double tar) {
        if (SubsystemInit.isRed())
            targetPos.angle = tar;
        else
            targetPos.angle = 180 - tar;
    }

    public Promise moveTo(Vector2 position) {
        if (!SubsystemInit.isRed())
            position.x *= -1;

        var prom = new SimplePromise();
        var autoDrive = this;
        targetPos.position = position;
        CommandScheduler.getInstance().schedule(new Command() {
            @Override
            public void execute() {
                if (autoDrive.getPositionalError() < 4) {
                    prom.resolve();
                    cancel();
                }
            }
        });
        return prom;
    }

    // --

    public Promise pointAndThenMoveTo(Vector2 point) {
        final double initialTar = targetPos.angle;
        pointTo(point);
        final double deltaAngle = Math.abs(initialTar - targetPos.angle);
        // we wait for the turn to complete and then move to the point
        return Promise.timeout(deltaAngle / 180).then(() -> moveTo(point));
    }

    public double getPositionalError() {
        return positioning.getPosition().minus(targetPos.position).getMagnitude();
    }

    public Vector2 displacement;

    @Override
    public void periodic() {
        displacement = targetPos.position.minus(positioning.getPosition());
        var correct = new Vector2(
                xCon.solve(displacement.x, 0.02),
                yCon.solve(displacement.y, 0.02));
        var turnCorrect = turnCon.solve(AngleMath.getDelta(positioning.getTurnAngle(), targetPos.angle), 0.02);
        drive.power(correct.getMagnitude(), correct.getAngleDeg() + 90 - positioning.getTurnAngle(), turnCorrect,
                false);
    }
}
