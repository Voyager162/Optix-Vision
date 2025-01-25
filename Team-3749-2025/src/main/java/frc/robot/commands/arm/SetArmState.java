package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.UtilityFunctions;

/**
 * Command to set the selected state for arm.
 * This command can either use a set point to move the arm to a specific position
 * or simply change the arm's state without considering the position.
 */
public class SetArmState<ArmState extends Enum<ArmState>> extends Command {

    // Instance variables
    private final Arm arm;            // The arm subsystem
    private final ArmState state;     // The state to set the arm to
    private final double setPoint;    // The target position for the arm (if applicable)
    private final boolean hasFinish;  // Flag to indicate if we should wait for the arm to reach the set point

    /**
     * Constructor for when the command requires a set point.
     *
     * @param arm The arm subsystem
     * @param state The arm state to set
     * @param setPoint The target position to move the arm to
     */
    public SetArmState(Arm arm, ArmState state, double setPoint) {
        this.arm = arm;
        this.state = state;
        this.setPoint = setPoint;
        this.hasFinish = true;  // We need to finish when the set point is reached
        addRequirements(arm);   // Ensure that no other command is using the arm during this command
    }

    /**
     * Constructor for when the command does not require a set point.
     * This is used when you just need to set the arm state without worrying about position.
     *
     * @param arm The arm subsystem
     * @param state The arm state to set
     */
    public SetArmState(Arm arm, ArmState state) {
        this.arm = arm;
        this.state = state;
        this.setPoint = 0;       // No position to check
        this.hasFinish = false;  // We don't need to finish based on position
        addRequirements(arm);    // Ensure that no other command is using the arm during this command
    }

    /**
     * Called once when the command is initialized. Sets the arm to the selected state.
     */
    @Override
    public void initialize() {
        arm.setState(state);  // Set the arm's state
    }

    /**
     * Called when the command ends (either successfully or due to interruption).
     * Stops the arm if we're not waiting for a set point.
     *
     * @param interrupted True if the command was interrupted, false otherwise
     */
    @Override
    public void end(boolean interrupted) {
        if (!hasFinish) {
            arm.stop();  // Stop the arm if we are not waiting for a position
        }
    }

    /**
     * Returns true when the command should end.
     * This checks if the arm has reached the set point (if applicable).
     *
     * @return True if the arm has reached the set point, false otherwise
     */
    @Override
    public boolean isFinished() {
        if (!hasFinish) {
            return false;  // If there's no set point, the command doesn't finish automatically
        } else {
            // Get the current position of the arm and check if it's within a small margin of the set point
            double position = arm.getPositionRad();
            return UtilityFunctions.withinMargin(0.0001, setPoint, position);
        }
    }
}
