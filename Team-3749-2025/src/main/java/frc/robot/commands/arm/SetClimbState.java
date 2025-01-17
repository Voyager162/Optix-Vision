package frc.robot.commands.arm;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.climb.ClimbConstants.ArmStates;
import frc.robot.subsystems.arm.climb.ClimbArm;
import frc.robot.utils.UtilityFunctions;

public class SetClimbState extends Command {

    ClimbArm arm;
    ArmStates state;
    double setPoint;

    public SetClimbState(ClimbArm arm, ArmStates state, double setPoint) {
        this.setPoint = setPoint;
        this.state = state;
        this.arm = arm;
        super.addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.setState(state);
    }

    @Override
    public void end(boolean interupted) {
    }

    @Override
    public boolean isFinished() {
        double position = arm.getPositionRad();
        return UtilityFunctions.withinMargin(0.0001, setPoint, position);
    }
}
