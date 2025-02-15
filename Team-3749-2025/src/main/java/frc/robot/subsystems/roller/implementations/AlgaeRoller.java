package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;

public class AlgaeRoller extends Roller {

    public AlgaeRoller() {
        super(Implementations.ALGAE, FF(), positionPID(),velocityPID());
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity.get(), RollerConstants.Algae.kVVelocity.get(),
                RollerConstants.Algae.kAVelocity.get());
    }
    public static PIDController positionPID(){
        return new PIDController(RollerConstants.Algae.kPPosition.get(), RollerConstants.Algae.kIPosition.get(),RollerConstants.Algae.kDPosition.get());
    }

    public static PIDController velocityPID(){
        return new PIDController(RollerConstants.Algae.kPVelocity.get(), RollerConstants.Algae.kIVelocity.get(),RollerConstants.Algae.kDVelocity.get());
    }
    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }

    @Override
    public void outtake() {}

}
