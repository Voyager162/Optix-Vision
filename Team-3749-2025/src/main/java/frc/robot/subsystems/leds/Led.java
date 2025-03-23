package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import edu.wpi.first.wpilibj.LEDPattern;

import frc.robot.subsystems.leds.real.LedReal;
import frc.robot.subsystems.leds.sim.LedSim;

public class Led extends SubsystemBase {
    private AddressableLEDBuffer ledBuffer;

    private LEDColor desiredColor = getTeamColorLED();
    private LEDColor currentColor = null;

    private boolean doRainbow = false;

    // private StatusIndicator statusIndicator = StatusIndicator.CORAL_PIECE;

    private double brightness = 1;

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
     * Returns the current LED pattern
     * 
     * @return
     */
    public LEDColor getCurrentColor() {
        return currentColor;
    }

    /**
     * Returns a LED pattern that matches the current alliance of the robot
     * 
     * @return
     */
    public LEDColor getTeamColorLED() {
        Optional<Alliance> team = DriverStation.getAlliance(); // i hate doing it this way but it throws an error
                                                               // without it
        if (!team.isPresent()) {
            return LEDColor.NO_TEAM;
        }

        return team.get() == Alliance.Blue ? LEDColor.BLUE_ALLIANCE : LEDColor.RED_ALLIANCE;
    }

    // private void setStripColor() {
    //     switch (statusIndicator) {
    //         case CORAL_PIECE:
    //             desiredColor = LEDColor.CORAL_ARM_HAS_PIECE;
    //             doRainbow = Robot.coralRoller.hasPiece();
    //             break;
    //         case OTF:
    //             desiredColor = LEDColor.RAINBOW;

    //         break;
    //         default:
    //             desiredColor = getTeamColorLED();
    //             doRainbow = false;
    //             break;
    //     }
    // }

    public void setLEDColor(LEDColor color) {
        this.desiredColor = color;
    }

    // public void setLEDStatusIndicator(StatusIndicator indicator) {
    //     statusIndicator = indicator;
    // }

    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }

    public void updateLEDs() {
        if (desiredColor == currentColor) {
            return;
        }

        LEDPattern setPattern = LEDPattern.solid(desiredColor.color).atBrightness(Percent.of(brightness * 100));

        if(doRainbow) {
            setPattern = LEDConstants.scrollingRainbow;
        }

        ledBase.setData(setPattern);
        currentColor = desiredColor;
    }

    public void logData() {
        Logger.recordOutput("LED/currentPattern", currentColor.toString());
        Logger.recordOutput("LED/desiredPattern", desiredColor.toString());
        Logger.recordOutput("LED/brightness", brightness);
    }

    @Override
    public void periodic() {
        // setStripColor();
        updateLEDs();
        logData();
    }

}