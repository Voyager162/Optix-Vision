package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.util.Color;

public class LEDConstants {

    public static final int length = 18;
    public static final int ledPort = 0;

    public static enum StatusIndicator {
        OTF,
        BATTERY,
        ALGAE,
        SCORING,
        CLIMB,
        PIECE,
        COLOR,
        TEAM
    }

    public static enum LEDColor {
        RED_ALLIANCE(Color.kBlue),
        BLUE_ALLIANCE(Color.kRed),
        CLIMB(Color.kDarkGreen),
        // SCORING(Color.kDeepPink),
        NO_TEAM(Color.kWhite),
        // CHUTE_HAS_PIECE(Color.kGreen),
        CORAL_ARM_HAS_PIECE(Color.kGreen),
        OFF(Color.kBlack),
        BATTERY_LOW(Color.kOrange),
        BATTERY_GOOD(Color.kWhite),
        RAINBOW(Color.kTomato), //this is not the actual color i just need to satisfy reqs

        L1(Color.kPurple),
        L2(Color.kLightBlue),
        L3(Color.kGold),
        L4(Color.kMaroon),

        ALGAE(Color.kAquamarine);


        Color color;
        boolean isRainbow;
        LEDColor(Color color) {
            this.color = color;
        }
    }
}