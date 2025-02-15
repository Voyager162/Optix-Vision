package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;

public class AlgaeRoller extends Roller {

    public AlgaeRoller() {
        super(Implementations.ALGAE, FF());
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity.get(), RollerConstants.Algae.kVVelocity.get(),
                RollerConstants.Algae.kAVelocity.get());
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }

    @Override
    public void outtake() {}

}
