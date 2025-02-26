package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.leds.LEDConstants.LEDPattern;

public class Led extends SubsystemBase {

    private LedIO led;

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

    public void setLEDPattern(LEDPattern pattern) {
        led.setLEDPattern(pattern);
    }

}