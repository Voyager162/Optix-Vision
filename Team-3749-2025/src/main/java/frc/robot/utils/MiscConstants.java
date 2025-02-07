package frc.robot.utils;

import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;

/**
 * Constants not specific to any given subsystem or commadn
 * 
 * @author Noah Simon
 */
public class MiscConstants {
  public static final Mode simMode = Mode.REPLAY;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static final class SimConstants {
    public static final double loopPeriodSec = 0.02;
  }

  public static final class ControllerConstants {

    public static final double deadband = 0.05;
  }

}
