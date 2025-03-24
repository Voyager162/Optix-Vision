package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.JoystickIO;
import frc.robot.subsystems.arm.climb.ClimbArmConstants.ArmStates;
import frc.robot.subsystems.leds.LEDConstants.LEDColor;

public class LedTriggers {

    public static void createLEDTriggers() {

        Trigger coralHasPieceTrigger = new Trigger(() -> Robot.coralRoller.hasPiece());
        coralHasPieceTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDColor(LEDColor.CORAL_ARM_HAS_PIECE)));

        Trigger scoringModeTrigger = new Trigger(() -> JoystickIO.getButtonBoard().getScoringMode() == JoystickIO
                .getButtonBoard().getPreviousScoringMode());
        scoringModeTrigger.onChange(Commands.runOnce(() -> {
            switch (JoystickIO.getButtonBoard().getScoringMode()) {
                case ALGAE:
                    Robot.led.setLEDColor(LEDColor.ALGAE);
                    break;
                case L1:
                    Robot.led.setLEDColor(LEDColor.L1);

                    break;
                case L2:
                    Robot.led.setLEDColor(LEDColor.L2);

                    break;
                case L3:
                    Robot.led.setLEDColor(LEDColor.L3);

                    break;
                case L4:
                    Robot.led.setLEDColor(LEDColor.L4);

                    break;
            }

        }));

        Trigger OTFTrigger = new Trigger(() -> Robot.swerve.getIsOTF());
        OTFTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDColor(LEDColor.RAINBOW)));
        OTFTrigger.onFalse(Commands.runOnce(() -> Robot.led.setLEDColor(Robot.led.getTeamColorLED())));

        Trigger climbTrigger = new Trigger(() -> Robot.climbArm.getState().equals(ArmStates.CLIMB));
        climbTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDColor(LEDColor.CLIMB)));
    }

}
