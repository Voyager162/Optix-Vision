package frc.robot.subsystems.arm;

import frc.robot.subsystems.arm.ArmConstants;

public class CoralArm extends ArmSubsystem {

    public CoralArm() {
        super();
    }

    @Override
    protected void runStateLogic() {
        switch (state) {
            case CORAL_PICKUP:
                setPoint = ArmConstants.coralPickUpSetPoint_Rad;
                setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
                break;
            case HAND_OFF:
                setPoint = ArmConstants.handOffSetPoint_rad;
                setVoltage(controller.calculate(data.positionUnits, setPoint));
                break;
            case STOWED:
                setPoint = ArmConstants.stowSetPoint_rad;
                setVoltage(controller.calculate(data.positionUnits, setPoint) + calculateFeedForward());
                break;
            case MOVING_UP:
                setVoltage(12 + calculateFeedForward());
                break;
            case MOVING_DOWN:
                setVoltage(-12 + calculateFeedForward());
                break;
            case STOPPED:
                setVoltage(0 + calculateFeedForward());
                break;
            default:
                break;
        }
    }
}
