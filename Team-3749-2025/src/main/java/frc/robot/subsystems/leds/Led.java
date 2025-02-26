package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Led extends SubsystemBase {

    private LedBase led;

    public Led() {
       if(Robot.isReal())
       {
        led = new LedReal();
        return;
       }
       led = new LedSim();
    }   

    @Override
    public void periodic() {
        led.loop();
    }

}