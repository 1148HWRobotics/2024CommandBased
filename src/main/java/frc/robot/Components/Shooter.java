package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Devices.AnyMotor;
import frc.robot.Util.MathPlus;
import frc.robot.Util.PWIDConstant;
import frc.robot.Util.PWIDController;

public class Shooter extends SubsystemBase {
    AnyMotor left;
    AnyMotor right;

    boolean isSpinning = false;

    public Shooter(AnyMotor left, AnyMotor right) {

        this.left = left;
        this.right = right;

        var con = new PWIDController(new PWIDConstant(0.1, 0.0, 0.01, 2.5));
        left.setVelocityPD(con);
        right.setVelocityPD(con);
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
        return MathPlus.withinBounds(left.getVelocity(), vel + 16, vel - 8);
    }

    public double vel = 85;

    public void periodic() {
        if (isSpinning) {
            System.out.println("vel: " + left.getVelocity() + " target vel: " + vel);
            right.setVelocity(vel);
            left.setVoltage(right.getVoltage());
        } else {
            left.setVoltage(0);
            right.setVoltage(0);
        }
    }
}
