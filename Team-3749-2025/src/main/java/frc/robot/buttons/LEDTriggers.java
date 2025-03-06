package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;

public class LEDTriggers {

    public static void createLEDTriggers() {

        Trigger coralRollerHasPiece = new Trigger(
                () -> Robot.scoringRoller.hasPiece()).onTrue(
                        Commands.runOnce(
                                () -> Robot.led.setLEDColor(LEDColor.CORAL_ARM_HAS_PIECE)));
    }
}
