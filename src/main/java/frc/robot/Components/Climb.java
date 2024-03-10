package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Devices.Motor.Falcon;
import frc.robot.Util.DeSpam;
import frc.robot.Util.MotionController;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PIDController;

public class Climb extends SubsystemBase {
    Falcon left;
    Falcon right;
    PIDController leftCon;
    PIDController rightCon;

    public Climb(Falcon left, Falcon right, PDConstant constant) {
        this.left = left;
        this.right = right;
        left.setBrakeMode(true);
        right.setBrakeMode(true);

        // if (constant instanceof PIDController)
        // ((PIDController) constant).setDeadZone(0.5);

        // left.setVelocityPD(constant.clone());
        // right.setVelocityPD(constant.clone());

        leftCon = new PIDController(constant);
        rightCon = new PIDController(constant);
    }

    public boolean isDown() {
        return target == null || target <= 360.0;
    }

    private Double target = 0.0;

    DeSpam dSpam = new DeSpam(0.5);

    public void motorTick(Falcon motor, MotionController con) {
        if (target != null) {
            var correction = con.solve(target - motor.getDegrees(), 0.02);
            motor.setVoltage(correction);
            // dSpam.exec(() -> {
            //     System.out.println("target: " + target + " current: " + motor.getDegrees() + " correction: " + correction);
            // });
        }
    }

    public double getTarget() {
        return target;
    }

    public void moveUp() {
        target = 360.0 * 79;
    }

    public void moveDown() {
        target = 360*-20.3;
    }

    // use to reset the climbers when we disable the climbers and they are up
    public void disablePD() {
        target = null;
    }

    public Falcon unsafeAccessLeftMotor() {
        return left;
    }

    public Falcon unsafeAccessRightMotor() {
        return right;
    }

    @Override
    public void periodic() {
        motorTick(left, leftCon);
        motorTick(right, rightCon);
    }
}
