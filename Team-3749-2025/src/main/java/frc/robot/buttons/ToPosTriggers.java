package frc.robot.buttons;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringLocation;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;
import frc.robot.utils.UtilityFunctions;

public class ToPosTriggers {
    public static BooleanSupplier isCoralSetpoint = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.A ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.B ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.C ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.D ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.E ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.F ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.G ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.H ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.I ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.J ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.K ||
                Robot.swerve.getPPSetpoint() == PPSetpoints.L;
    
        private static boolean OTFWithinMargin() {
            return UtilityFunctions.withinMargin(ToPosConstants.Setpoints.approachPointDistance,
                    Robot.swerve.getPose().getTranslation(),
                    Robot.swerve.getPPSetpoint().setpoint.getTranslation());
        }
    
        public static void createOTFTriggers() {
    
            Trigger coralStation = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
                Boolean withinMargin = OTFWithinMargin();
                Boolean isCoralStation = Robot.swerve.getPPSetpoint() == PPSetpoints.CORALLEFT || Robot.swerve.getPPSetpoint() == PPSetpoints.CORALRIGHT;
                return withinMargin && isCoralStation;
            });
            coralStation.onTrue(new PrintCommand("Coral Station"));
    
            Trigger coralReefL1 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
                Boolean withinMargin = OTFWithinMargin();
                Boolean isCoralReef = isCoralSetpoint.getAsBoolean();
            Boolean isL1 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L1;
            return withinMargin && isCoralReef && isL1;
        });
        coralReefL1.onTrue(Commands.print("Score L1"));

        Trigger coralReefL2 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = isCoralSetpoint.getAsBoolean();
            Boolean isL2 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L2;
            return withinMargin && isCoralReef && isL2;
        });
        coralReefL2.onTrue(Commands.print("Score L2"));

        Trigger coralReefL3 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = isCoralSetpoint.getAsBoolean();
            Boolean isL3 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L3;
            return withinMargin && isCoralReef && isL3;
        });
        coralReefL3.onTrue(Commands.print("Score L3"));

        Trigger coralReefL4 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = isCoralSetpoint.getAsBoolean();
            Boolean isL4 = JoystickIO.buttonBoard.getScoringLocation() == ScoringLocation.L4;
            return withinMargin && isCoralReef && isL4;
        });
        coralReefL4.onTrue(Commands.print("Score L4"));

    }
}
