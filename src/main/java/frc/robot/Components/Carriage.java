package frc.robot.Components;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Devices.BinarySensor;
import frc.robot.Devices.Motor.TalonFX;
import frc.robot.Util.MotionController;
import frc.robot.Util.PDConstant;
import frc.robot.Util.PIDController;

public class Carriage extends SubsystemBase {
    Double startPos = null;
    TalonFX motor;
    BinarySensor noteSensor;
    MotionController controller = new PIDController(new PDConstant(4, 0, 3.0));
    boolean prepShot = false;

    public Carriage(TalonFX motor, BinarySensor noteSensor) {
        this.motor = motor;
        this.noteSensor = noteSensor;
    }

    public void intake() {
        motor.setVelocity(0.3 * 360);
    }

    public void stop() {
        motor.setVelocity(0);
    }

    public void outTake() {
        motor.setVoltage(-5);
        startPos = null;
    }

    public void prepShot() {
        prepShot = true;
    }

    public void unPrepShot() {
        prepShot = false;
    }

    public void shoot() {
        motor.setVoltage(12);
        startPos = null;
    }

    public boolean hasNote() {
        return startPos != null;
    }

    @Override
    public void periodic() {
        if (noteSensor.justEnabled()) {
            startPos = motor.getRevs();
            prepShot = false;
        }

        if (startPos != null) {
            motor.setVoltage(controller.solve(-(motor.getRevs() - (prepShot ? startPos + 4 : startPos + 1.5)), 0.02));
        }
    }
}
