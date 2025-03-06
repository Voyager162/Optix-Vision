package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.arm.climb.ClimbArmConstants.ArmStates;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import frc.robot.subsystems.leds.LEDConstants.StatusIndicator;
import edu.wpi.first.wpilibj.LEDPattern;

import frc.robot.subsystems.leds.real.LedReal;
import frc.robot.subsystems.leds.sim.LedSim;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class Led extends SubsystemBase {
    private AddressableLEDBuffer ledBuffer;

    private LEDColor desiredPattern = LEDColor.RAINBOW;
    private LEDColor currentPattern = LEDColor.RAINBOW;

    private StatusIndicator statusIndicator = StatusIndicator.TEAM;

    private double brightness = 1;

    private final LEDPattern rainbowPattern = LEDPattern.rainbow(255, 128);
    private final Distance ledSpacing = Meters.of(Units.inchesToMeters(11.5) / 18);
    private final LEDPattern scrollingRainbow = rainbowPattern.scrollAtAbsoluteSpeed(MetersPerSecond.of(0.5),
            ledSpacing);
    private LEDIO ledBase;

    public Led() {
        ledBuffer = new AddressableLEDBuffer(LEDConstants.length);

        if (Robot.isReal()) {
            ledBase = new LedReal(LEDConstants.ledPort, ledBuffer);
        } else {
            ledBase = new LedSim(LEDConstants.ledPort, ledBuffer);
        }
        ledBase.setData(LEDPattern.kOff);
        LedTriggers.createLEDTriggers();
    }

    /**
     * Takes the parameter of brightness to set the brightness of the LEDs
     * 
     * @param brightness
     */
    public Led(double brightness) {
        this();
        this.brightness = brightness;
    }

    /**
     * Returns a LED pattern that matches the current alliance of the robot
     * 
     * @return
     */
    private LEDColor getTeamColorLED() {
        Optional<Alliance> team = DriverStation.getAlliance(); // i hate doing it this way but it throws an error
                                                               // without it
        if (!team.isPresent()) {
            return LEDColor.NO_TEAM;
        }

        return team.get() == Alliance.Blue ? LEDColor.BLUE_ALLIANCE : LEDColor.RED_ALLIANCE;
    }

    private void setStripColor() {
        if (this.desiredPattern == LEDColor.RAINBOW) {
            ledBase.setData(scrollingRainbow);
            currentPattern = desiredPattern;
            return;
        }
        if (desiredPattern == currentPattern) {
            return;
        }

        LEDPattern setPattern = LEDPattern.solid(desiredPattern.color).atBrightness(Percent.of(brightness * 100));
        ledBase.setData(setPattern);

        currentPattern = desiredPattern;
    }

    public void setLEDColor(LEDColor color) {
        this.desiredPattern = color;
    }

    public void setLEDStatusIndicator(StatusIndicator indicator) {
        statusIndicator = indicator;
    }

    /**
     * Returns the current LED pattern
     * 
     * @return
     */
    public LEDColor getCurrentPattern() {
        return currentPattern;
    }

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }

    @Override
    public void periodic() {
        // switch (statusIndicator) {
        //     case OTF:
        //         if(Robot.swerve.getIsOTF())
        //         {
        //             setLEDColor(LEDColor.RAINBOW);
        //             return;
        //         }
        //         setLEDColor(LEDColor.OFF);
        //     break;
        //     case BATTERY:
        //         if (RobotController.getBatteryVoltage() < 8) {
        //             setLEDColor(LEDColor.BATTERY_LOW);
        //             return;
        //         }
        //         setLEDColor(LEDColor.BATTERY_GOOD);
        //         break;
        //     case PIECE:
        //         if (Robot.coralRoller.hasPiece()) {
        //             setLEDColor(LEDColor.CORAL_ARM_HAS_PIECE);
        //             return;
        //         }
        //         if (Robot.scoringRoller.hasPiece()) {
        //             setLEDColor(LEDColor.CHUTE_HAS_PIECE);
        //             return;
        //         }
        //         setLEDColor(LEDColor.OFF);
        //         break;
        //     case COLOR:
        //         setLEDColor(getCurrentPattern());
        //         break;
        //     case TEAM:
        //         setLEDColor(getTeamColorLED());
        //         break;
        //     default:
        //         setLEDColor(getTeamColorLED());
        //         break;
        // }
        // setStripColor();



        // this is the way I (weston) would do it, this way it is a bit simpler and we can prioritize certain colors and is easier to manage

            //     switch (statusIndicator) {
    //         case COLOR:
    //             setLEDColor(getCurrentPattern());
    //             break;
    //         case TEAM:
    //             setLEDColor(getTeamColorLED());
    //             break;
    //         default:
    //             break;
    //     }

    //     if (RobotController.getBatteryVoltage() < 8) {
    //         setLEDColor(LEDColor.BATTERY_LOW);
    //         return;
    //     }

    //     else if(Robot.swerve.getIsOTF())
    //     {
    //         setLEDColor(LEDColor.RAINBOW);
    //         return;
    //     }

    //     else if (Robot.scoringRoller.hasPiece()) {
    //         setLEDColor(LEDColor.CHUTE_HAS_PIECE);
    //         return;
    //     }

    //     else if (Robot.coralRoller.hasPiece()) {
    //         setLEDColor(LEDColor.CORAL_ARM_HAS_PIECE);
    //         return;
    //     }

    //     else {
    //         setLEDColor(LEDColor.BATTERY_GOOD);
    //     }
    //     setStripColor();
    // }
    }

}