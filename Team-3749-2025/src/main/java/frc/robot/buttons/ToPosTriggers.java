package frc.robot.buttons;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringLocation;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;
import frc.robot.utils.UtilityFunctions;

public class ToPosTriggers {

    private static boolean OTFWithinMargin() {
        return UtilityFunctions.withinMargin(ToPosConstants.Setpoints.approachPointDistance,
                Robot.swerve.getPose().getTranslation(),
                Robot.swerve.getPPSetpoint().setpoint.getTranslation());
    }

    public static void createOTFTriggers() {

        Trigger coralStation = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralStation = Robot.swerve.getPPSetpoint() == PPSetpoints.CORALLEFT
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.CORALRIGHT;
            return withinMargin && isCoralStation;
        });
        coralStation.onTrue(new PrintCommand("Coral Station"));

        Trigger coralReefL1 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = Robot.swerve.getPPSetpoint() == PPSetpoints.A
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.B; // put all reef locaitons here w boolean supplier
            Boolean isL1 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L1;
            return withinMargin && isCoralReef && isL1;
        });
        coralReefL1.onTrue(Commands.print("Score L1"));

        Trigger coralReefL2 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = Robot.swerve.getPPSetpoint() == PPSetpoints.A
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.B; // put all reef locaitons here w boolean supplier
            Boolean isL2 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L2;
            return withinMargin && isCoralReef && isL2;
        });
        coralReefL2.onTrue(Commands.print("Score L2"));

        Trigger coralReefL3 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = Robot.swerve.getPPSetpoint() == PPSetpoints.A
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.B; // put all reef locaitons here w boolean supplier
            Boolean isL3 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L3;
            return withinMargin && isCoralReef && isL3;
        });
        coralReefL3.onTrue(Commands.print("Score L3"));

        Trigger coralReefL4 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = Robot.swerve.getPPSetpoint() == PPSetpoints.A
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.B; // put all reef locaitons here w boolean supplier
            Boolean isL4 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L4;
            return withinMargin && isCoralReef && isL4;
        });
        coralReefL4.onTrue(Commands.print("Score L4"));

    }
}
