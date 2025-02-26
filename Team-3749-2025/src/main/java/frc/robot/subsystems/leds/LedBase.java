package frc.robot.subsystems.leds;

import frc.robot.subsystems.leds.LEDConstants.LEDPattern;

public interface LedBase {
    public default void loop() {}
    public default void setLEDPattern(LEDPattern pattern) {}
}
