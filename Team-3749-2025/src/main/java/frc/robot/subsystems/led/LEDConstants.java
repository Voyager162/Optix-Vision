package frc.robot.subsystems.led;

public class LEDConstants {

    public static final int length = 49;

    public static enum LEDPattern {
        RED(255, 0, 0), 
        BLUE(0, 0, 255), 
        GREEN(0, 255, 0), 
        YELLOW(255, 255, 0), 
        WHITE(255, 255, 255), 
        NOTHING(0, 0, 0);

        final int R;
        final int G;
        final int B;

        /**
         * 
         * @param R
         * @param G
         * @param B
         */
        LEDPattern (int R, int G, int B){
            this.R = R;
            this.G = G;
            this.B = B;
        }
    }

}