package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Devices.AnyMotor;
import frc.robot.Util.DeSpam;
import frc.robot.Util.MathPlus;
import frc.robot.Util.PIDConstant;
import frc.robot.Util.PIDController;
import frc.robot.Util.PWIDConstant;
import frc.robot.Util.PWIDController;

public class Shooter extends SubsystemBase {
    AnyMotor left;
    AnyMotor right;

    boolean isSpinning = false;

    DeSpam dSpam = new DeSpam(0.5);

    public Shooter(AnyMotor left, AnyMotor right) {

        this.left = left;
        this.right = right;

        var con = new PWIDController(new PWIDConstant(0.1, 0, 0.01, 3));
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
        // var angle = Math.toRadians(40);
        // var velocity = 3 * distance / Math.cos(angle) * Math.sqrt(9.81 / (distance *
        // Math.tan(angle) - 12));
        // checks if velocity is NaN

        // var vel = (-con.getRightY() + 1) * 150;

        // this.vel = vel;

        if (isSpinning) {
            dSpam.exec(() -> {
                System.out.println("tvel: " + vel + " vel: " + right.getVelocity());
            });

            right.setVelocity(vel);
            left.setVoltage(right.getVoltage());
        } else {
            left.setVoltage(0);
            right.setVoltage(0);
        }
    }
}
