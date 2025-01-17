package frc.robot.commands.arm.coral;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.arm.ArmConstants.coralArmConstants.ArmStates;
import frc.robot.subsystems.arm.CoralArm;
import frc.robot.utils.UtilityFunctions;

public class SetCoralState extends Command {

    CoralArm arm;
    ArmStates state;
    double setPoint;

    public SetCoralState(CoralArm arm, ArmStates state, double setPoint) {
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
