package frc.robot.subsystems.leds.real;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.LEDPattern;
import frc.robot.subsystems.leds.LEDConstants;
import frc.robot.subsystems.leds.LEDIO;

public class LedReal implements LEDIO {
    AddressableLED leds;
    AddressableLEDBuffer buf;

    public LedReal(int port, AddressableLEDBuffer buf) {
        leds = new AddressableLED(port);
        this.buf = buf;
        leds.setLength(LEDConstants.length);
        leds.setData(buf);
        leds.start();
    }

    public void setData(LEDPattern pattern)
    {
        pattern.applyTo(buf);
        leds.setData(buf);
    }
}
