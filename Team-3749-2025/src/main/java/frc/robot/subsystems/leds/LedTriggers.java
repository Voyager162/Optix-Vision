package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.arm.climb.ClimbArmConstants.ArmStates;
import frc.robot.subsystems.leds.LEDConstants.StatusIndicator;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

public class LedTriggers {

    public static void createLEDTriggers() {
        Trigger coralHasPieceTrigger = new Trigger(() -> Robot.coralRoller.hasPiece());
        coralHasPieceTrigger.onTrue(Commands.runOnce(() -> 
        Robot.led.setLEDStatusIndicator(StatusIndicator.PIECE)));

        Trigger scoringHasPieceTrigger = new Trigger(() -> Robot.scoringRoller.hasPiece());
        scoringHasPieceTrigger.onTrue(Commands.runOnce(() -> 
        Robot.led.setLEDStatusIndicator(StatusIndicator.PIECE)));

        Trigger algaeKnockTrigger = new Trigger(() -> Robot.scoringRoller.getIsAlgaeMode());
        algaeKnockTrigger.onTrue(Commands.runOnce(() -> 
        Robot.led.setLEDStatusIndicator(StatusIndicator.ALGAE)));

        Trigger scoringTrigger = new Trigger(() -> Robot.scoringRoller.getState().equals(RollerStates.SCORE));
        scoringTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDStatusIndicator(StatusIndicator.SCORING)));

        Trigger OTFTrigger = new Trigger(() -> Robot.swerve.getIsOTF());
        OTFTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDStatusIndicator(StatusIndicator.OTF)
        ));

        Trigger climbTrigger = new Trigger(() -> Robot.climbArm.getState().equals(ArmStates.CLIMB));
        climbTrigger.onTrue(Commands.runOnce(() -> Robot.led.setLEDStatusIndicator(StatusIndicator.CLIMB)
        ));

    }
    
}
