package frc.robot.buttons;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.commands.integration.ScoringModeConditionalHandoff;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.swerve.ToPosConstants; //dont remvoe these yet: read the commented stuff
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;

/**
 * The `ToPosTriggers` class manages automatic command triggers for robot
 * positioning.
 * It uses predefined setpoints from `PPSetpoints` and executes commands when
 * the robot
 * is within a certain distance from its target. This ensures smooth execution
 * of scoring
 * and intake actions.
 */
public class ToPosTriggers {

        // ======= Boolean Suppliers for Identifying Robot Position =======

        /**
         * Checks if the current PPSetpoint is at one of the reef scoring positions.
         */
        public static BooleanSupplier isReefSupplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.A ||
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

        /**
         * Checks if the current PPSetpoint is at one of the Level 1 reef scoring
         * positions.
         */
        public static BooleanSupplier isReefL1Supplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.AL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.BL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.CL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.DL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.EL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.FL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.GL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.HL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.IL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.JL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.KL1 ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.LL1;

        /**
         * Checks if the current PPSetpoint is at one of the coral station positions.
         */
        public static BooleanSupplier isCoralSupplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.CORALLEFT ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.CORALRIGHT;

        /**
         * Checks if the current PPSetpoint is at a high algae knocking position.
         */
        public static BooleanSupplier isHighAlgaeSupplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.REEFCLOSE
                        ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.REEFFARLEFT ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.REEFFARRIGHT;

        /**
         * Checks if the current PPSetpoint is at a low algae knocking position.
         */
        public static BooleanSupplier isLowAlgaeSupplier = () -> Robot.swerve.getPPSetpoint() == PPSetpoints.REEFFAR ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.REEFCLOSELEFT ||
                        Robot.swerve.getPPSetpoint() == PPSetpoints.REEFCLOSERIGHT;

        // ======= Utility Functions =======

        // /**
        // * Checks if the robot is within a certain margin of the target setpoint.
        // * Used to determine when to execute position-based triggers.
        // *
        // * @return true if the robot is within the defined distance of the target
        // * setpoint.
        // */
        public static boolean OTFWithinMargin() {
                //save a bit on processing power cause i assume trig calcs are a bit expensive
                double cos30 = Math.cos(Units.degreesToRadians(30));
                double sin30 = Math.sin(Units.degreesToRadians(30));

                // translate the point to the hexagon's center
                double dx = Robot.swerve.getPose().getX() - ToPosConstants.ReefDimensions.hexagonCenterMeters.getX();
                double dy = Robot.swerve.getPose().getY() - ToPosConstants.ReefDimensions.hexagonCenterMeters.getY();

                // Rotate the point back by -30 degrees cause the reefs rotated 30 from flat sides on top and bottom
                double xPrime = dx * cos30 + dy * sin30;
                double yPrime = -dx * sin30 + dy * cos30;

                // normalize coordinates for hexagon check
                double qx = (2.0 / 3.0) * xPrime / ToPosConstants.ReefDimensions.hexagonRadiusMeters;
                double qy = (-1.0 / 3.0) * xPrime / ToPosConstants.ReefDimensions.hexagonRadiusMeters + (2.0 / 3.0) * yPrime / (Math.sqrt(3) * ToPosConstants.ReefDimensions.hexagonRadiusMeters);

                // is it inside the reef
                return Math.abs(qx) <= 1 && Math.abs(qy) <= 1 && Math.abs(qx + qy) <= 1;

              
                // return
                // UtilityFunctions.withinMargin(ToPosConstants.Setpoints.approachPointDistance,
                // Robot.swerve.getPose().getTranslation(),
                // Robot.swerve.getPPSetpoint().setpoint.getTranslation());
        }

        // ======= Trigger Setup for Automatic Actions =======

        /**
         * Creates all On-The-Fly (OTF) triggers for automatic actions when the robot
         * reaches certain locations.
         */
        public static void createOTFTriggers() {

                // ======= Coral Station Trigger =======
                // Activates intake when the robot reaches the coral station.
                Trigger coralStation = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isCoralSupplier.getAsBoolean();
                });
                coralStation.onTrue(new CoralIntakeSource().andThen(new ScoringModeConditionalHandoff()));

                // ======= Reef Level 1 Scoring Trigger =======
                // Scores using the Level 1 elevator state when within range.
                Trigger coralReefL1 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isReefL1Supplier.getAsBoolean() &&
                                        JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L1;
                });
                coralReefL1.onTrue(new ScoreL1());

                // ======= Reef Level 2 Scoring Trigger =======
                Trigger coralReefL2 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isReefSupplier.getAsBoolean() &&
                                        JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L2;
                });
                coralReefL2.onTrue(new ScoreL234(ElevatorStates.L2));

                // ======= Reef Level 3 Scoring Trigger =======
                Trigger coralReefL3 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isReefSupplier.getAsBoolean() &&
                                        JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L3;
                });
                coralReefL3.onTrue(new ScoreL234(ElevatorStates.L3));

                // ======= Reef Level 4 Scoring Trigger =======
                Trigger coralReefL4 = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isReefSupplier.getAsBoolean() &&
                                        JoystickIO.getButtonBoard().getScoringMode() == ScoringMode.L4;
                });
                coralReefL4.onTrue(new ScoreL234(ElevatorStates.L4));

                // ======= High Algae Knocking Trigger =======
                Trigger highAlgaeTrigger = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isHighAlgaeSupplier.getAsBoolean();
                });
                highAlgaeTrigger.onTrue(new KnockAlgae(ElevatorStates.ALGAE_HIGH));

                // ======= Low Algae Knocking Trigger =======
                Trigger lowAlgaeTrigger = new Trigger(() -> Robot.swerve.getIsOTF()).and(() -> {
                        return OTFWithinMargin() &&
                                        isLowAlgaeSupplier.getAsBoolean();
                });
                lowAlgaeTrigger.onTrue(new KnockAlgae(ElevatorStates.ALGAE_LOW));

        }
}
