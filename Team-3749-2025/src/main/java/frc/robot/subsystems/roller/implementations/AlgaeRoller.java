package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerConstants.RollerStates;

/**
 * Algae implementation of the roller subsystem
 *
 * @author Lilian Wu
 */
public class AlgaeRoller extends Roller {
    public AlgaeRoller() {
        super(Implementations.ALGAE, FF(), positionPID(), velocityPID());
    }

    public static PIDController positionPID() {
        return new PIDController(RollerConstants.Algae.kPPosition.get(), RollerConstants.Algae.kIPosition.get(),
                RollerConstants.Algae.kDPosition.get());
    }

    public static PIDController velocityPID() {
        return new PIDController(RollerConstants.Algae.kPVelocity.get(), RollerConstants.Algae.kIVelocity.get(),
                RollerConstants.Algae.kDVelocity.get());
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity.get(),
                RollerConstants.Algae.kVVelocity.get(),
                RollerConstants.Algae.kAVelocity.get());
    }

    @Override
    public void intake() {
        setVelocity(RollerStates.INTAKE.algaeVelocity);
    }

    @Override
    public void outtake() {
        setVelocity(RollerStates.OUTTAKE.algaeVelocity);
    }

    @Override
    public void maintain() {
        setPosition(getLastKnownPosition(), RollerConstants.Algae.kSVelocity.get());
    }
}
