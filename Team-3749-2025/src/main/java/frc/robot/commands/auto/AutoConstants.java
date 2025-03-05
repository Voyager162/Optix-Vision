package frc.robot.commands.auto;

import edu.wpi.first.math.util.Units;

/**
 * Constants unique to autonomous actions
 * 
 * @author Noah Simon
 */
public final class AutoConstants {
        public final class OTF{}
        public static double kPDrive = 1.75; // 3
        public static double kIDrive = 1; // 0.1
        public static double kDDrive = 0.65; // 0.5
        public static double driveIZone = 0.1; //0.1
        public static double driveToleranceMeters = 0.01;

        public static double kPTurn = 7.5; // 4.75
        public static double kITurn = 1; // 3
        public static double kDTurn = 0.4;// 0.2
        public static double turnIZone = Units.degreesToRadians(5);
        public static double turnToleranceRad = Units.degreesToRadians(0.75);

}