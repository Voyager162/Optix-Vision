package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
public class CoralRoller extends Roller {

    public CoralRoller() {
        super(Implementations.CORAL, FF());
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity.get(), RollerConstants.Coral.kVVelocity.get(),
                RollerConstants.Coral.kAVelocity.get());
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.intakeVelocity.get());
    }

    @Override
    public void outtake() {
        setVelocity(RollerConstants.Coral.outtakeVelocity.get());
    }
}
