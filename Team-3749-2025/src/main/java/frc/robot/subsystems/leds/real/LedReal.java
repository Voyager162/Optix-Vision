package frc.robot.subsystems.leds.real;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.subsystems.leds.LEDConstants;

public class LedReal {
    AddressableLED leds;

    public LedReal(int port, AddressableLEDBuffer buf) {
        leds = new AddressableLED(port);

        leds.setLength(LEDConstants.length);
        leds.setData(buf);
        leds.start();
    }
}
