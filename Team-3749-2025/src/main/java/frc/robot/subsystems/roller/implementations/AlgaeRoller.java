package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;

/**
 * Algae implementation of the roller subsystem
 *
 * @author Lilian Wu
 */
public class AlgaeRoller extends Roller {

    public AlgaeRoller() {
        super(Implementations.ALGAE, FF()); 
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity, RollerConstants.Algae.kVVelocity, RollerConstants.Algae.kAVelocity);
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }

    /**
     * Implemetation of score method
     */
     @Override
    public void score() {
    }
}
