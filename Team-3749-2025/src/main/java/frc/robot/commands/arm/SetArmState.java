package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.Arm;
import frc.robot.utils.UtilityFunctions;

/**
 * Sets the selected state for a given arm
 */
public class SetArmState<ArmState extends Enum<ArmState>> extends Command {

    private final Arm arm;
    private final ArmState state;
    private final double setPoint;

    public SetArmState(Arm arm, ArmState state, double setPoint) {
        this.arm = arm;
        this.state = state;
        this.setPoint = setPoint;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setState(state);
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        double position = arm.getPositionRad();
        return UtilityFunctions.withinMargin(0.0001, setPoint, position);
    }
}
