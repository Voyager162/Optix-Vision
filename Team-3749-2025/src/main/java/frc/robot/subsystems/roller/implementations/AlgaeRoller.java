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
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity, RollerConstants.Algae.kVVelocity, RollerConstants.Algae.kAVelocity);
    }


    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }

    // does not score
    @Override
    public void score() {
    }
}
