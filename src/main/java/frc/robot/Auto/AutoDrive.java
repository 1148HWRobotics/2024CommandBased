package frc.robot.Auto;

import java.util.Vector;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Drive.PositionedDrive;
import frc.robot.Util.AngleMath;
import frc.robot.Util.MotionController;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PIDController;
import frc.robot.Util.Promise;
import frc.robot.Util.SimplePromise;
import frc.robot.Util.Vector2;

public class AutoDrive extends SubsystemBase {
    FieldPositioning positioning;
    public Position targetPos;
    PositionedDrive drive;

    PIDController xCon;
    PIDController yCon;
    PIDController turnCon;

    public AutoDrive(FieldPositioning positioning, Position targetPos, PositionedDrive drive, PDConstant transCon,
            PDConstant turnCon) {
        this.positioning = positioning;
        this.targetPos = targetPos;
        this.drive = drive;

        this.xCon = new PIDController(transCon);
        this.yCon = new PIDController(transCon);
        this.turnCon = new PIDController(turnCon);
    }

    public double getPositionalError() {
        return positioning.getPosition().minus(targetPos.position).getMagnitude();
    }

    public void pointTo(Vector2 position) {
        setAngleTar(position.minus(positioning.getPosition()).getAngleDeg());
    }

    public Promise moveTo(Vector2 position) {
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

    public void setAngleTar(double tar) {
        targetPos.angle = tar;
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
