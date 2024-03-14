package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Devices.Motor.Falcon;
import frc.robot.Util.DeSpam;
import frc.robot.Util.MathPlus;
import frc.robot.Util.MotionController;
import frc.robot.Devices.BinarySensor;

public class Elevator extends SubsystemBase {
    Falcon left;
    Falcon right;
    BinarySensor zero;

    public Elevator(Falcon left, Falcon right, MotionController constant, BinarySensor zero) {
        this.left = left;
        this.right = right;
        this.zero = zero;

        left.resetEncoder();

        // if (constant instanceof PIDController)
        // ((PIDController) constant).setDeadZone(0.5);

        left.setVelocityPD(constant.clone());
        right.setVelocityPD(constant.clone());

        left.setCurrentLimit(30);
        right.setCurrentLimit(30);
    }

    public boolean isDown() {
        return target != null && target <= 0;
    }

    public Double target = 0.0;

    public double getHeight() {
        return left.getDegrees();
    }

    public double getTarget() {
        if (target == null)
            return 0;
        return target;
    }

    final double upHeight = 360.0 * 23.1;
    final double downHeight = -360 * 0.1;

    public void moveUp() {
        target = upHeight;
    }

    public void moveDown() {
        target = 0.0;
    }

    public void climbDown() {
        target = null;
        left.setVelocity(-48);
        right.setVelocity(-48);
    }

    DeSpam dSpam = new DeSpam(0.5);

    @Override
    public void periodic() {
        if (target != null) {
            target = Math.max(Math.min(target, upHeight), downHeight);
            var goingDown = target == 0;
            final var height = getHeight() + 0;

            var slowFac = 20;
            var max = 48 * slowFac;
            if (!goingDown)
                max *= 1.6;
            var vel = MathPlus.clampAbsVal(target - height, max) / slowFac;
            dSpam.exec(() -> {
                System.out.println("target " + target + "curr: " + height + " vel: " + vel);
            });

            if (!zero.get() && goingDown) {
                left.resetEncoder();
            } else {
                left.setVelocity(vel);
                right.setVelocity(vel);
            }
        }
    }
}
