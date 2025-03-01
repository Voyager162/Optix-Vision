package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {

    public static final int length = 18;
    public static final int ledPort = 0;

    public static enum StatusIndicator {
        BATTERY,
        PIECE,
        COLOR,
        TEAM
    }

    public static enum LEDColor {
        RED_ALLIANCE(Color.kRed),
        BLUE_ALLIANCE(Color.kBlue),
        CHUTE_HAS_PIECE(Color.kGreen),
        CORAL_ARM_HAS_PIECE(Color.kPurple),
        OFF(Color.kBlack),
        BATTERY_LOW(Color.kOrange),
        BATTERY_GOOD(Color.kWhite);

        Color color;

        LEDColor(Color color) {
            this.color = color;
        }
    }
}