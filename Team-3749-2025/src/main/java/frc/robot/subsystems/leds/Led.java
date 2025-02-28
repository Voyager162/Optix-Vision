package frc.robot.subsystems.leds;

import static edu.wpi.first.units.Units.Percent;

import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;
import frc.robot.subsystems.leds.LEDConstants.StatusIndicator;
import edu.wpi.first.wpilibj.LEDPattern;

public class LED extends SubsystemBase {

    private AddressableLED[] leds;

    private AddressableLEDBuffer[] ledBuffers;

    private LEDColor desiredPattern = getTeamColorLED();
    private LEDColor currentPattern = LEDColor.OFF;

    private StatusIndicator statusIndicator = StatusIndicator.TEAM;

    private double brightness = 1;

    public LED() {
        for (int port : LEDConstants.ledPorts) {
            leds[port] = new AddressableLED(port);
            ledBuffers[port] = new AddressableLEDBuffer(LEDConstants.length);

            leds[port].setLength(ledBuffers[port].getLength());
            leds[port].setData(ledBuffers[port]);

            leds[port].start();
        }
    }

    /**
     * Takes the parameter of brightness to set the brightness of the LEDs
     * 
     * @param brightness
     */
    public LED(double brightness) {
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
        return team.get() == Alliance.Blue ? LEDColor.BLUE_ALLIANCE : LEDColor.RED_ALLIANCE;
    }

    private void setStripColor(int strip) {
        if (desiredPattern == currentPattern) {
            return;
        }

        if (DriverStation.isEnabled()) {
            brightness = 1;
        }

        LEDPattern setPattern = LEDPattern.solid(desiredPattern.color).atBrightness(Percent.of(brightness * 100));
        setPattern.applyTo(ledBuffers[strip]);

        currentPattern = desiredPattern;
    }

    private void setAllStrips() {
        for (int strip : LEDConstants.ledPorts) {
            setStripColor(strip);
        }
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

    // runs every 0.02 sec
    @Override
    public void periodic() {
        switch (statusIndicator) {
            case BATTERY:
                if (RobotController.getBatteryVoltage() < 8) {
                    setLEDColor(LEDColor.BATTERY_LOW);
                } else {
                    setLEDColor(LEDColor.BATTERY_GOOD);
                }
                break;
            case PIECE:
                if (Robot.coralRoller.hasPiece()) {
                    setLEDColor(LEDColor.CORAL_ARM_HAS_PIECE);
                } else if (Robot.scoringRoller.hasPiece()) {
                    setLEDColor(LEDColor.CHUTE_HAS_PIECE);
                } else {
                    setLEDColor(LEDColor.OFF);
                }
                break;
            case COLOR:
                break;
            case TEAM:
            default:
                setLEDColor(getTeamColorLED());
        }

        setAllStrips();
    }

}