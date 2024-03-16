package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Devices.AnyMotor;
import frc.robot.Util.MathPlus;
import frc.robot.Util.MotionController;
import frc.robot.Util.PWIDConstant;
import frc.robot.Util.PWIDController;

public class Shooter extends SubsystemBase {
    AnyMotor left;
    AnyMotor right;

    boolean isSpinning = false;

    public Shooter(AnyMotor left, AnyMotor right) {

        this.left = left;
        this.right = right;
    }

    public void spin() {
        isSpinning = true;
    }

    public void stop() {
        isSpinning = false;
    }

    public void toggleSpinning() {
        isSpinning = !isSpinning;
    }

    public boolean isSpinning() {
        return isSpinning;
    }

    public boolean isAtVelocity() {
        return MathPlus.withinBounds(left.getVelocity(), vel + 3, vel - 3);
    }

    public double vel = 85;

    final MotionController con = new PWIDController(new PWIDConstant(0.1, 0.0, 0.035, 0.6));

    public double getVoltage(double velocity) {
        return vel * 0.1;
    }

    public void periodic() {
        if (isSpinning) {
            // adds the output of the controller to the predicted
            // voltage required to reach the velocity
            double correct = getVoltage(vel) + con.solve(vel - left.getVelocity(), 0.02);
            System.out.println("vel: " + left.getVelocity() + " target vel: " + vel + " voltage: " + correct);
            left.setVoltage(correct);
            right.setVoltage(correct);
        } else {
            left.setVoltage(0);
            right.setVoltage(0);
        }
    }
}
