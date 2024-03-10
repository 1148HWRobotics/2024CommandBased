package frc.robot.Components;

import frc.robot.Devices.Motor.Falcon;

public class Lifter {
    Falcon left;
    Falcon right;
    
    public Lifter(Falcon left, Falcon right) {
        this.left = left;
        this.right = right;
    }
}
