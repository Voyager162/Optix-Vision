package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.utils.LoggedTunableNumber;

public class CoralRoller extends Roller {
    
    public CoralRoller() {
        super(Implementations.CORAL, velocityController(), FF(), positionController());
        kp = new LoggedTunableNumber(getName() + "/kP", RollerConstants.Coral.kPVelocity);
        ki = new LoggedTunableNumber(getName() + "/kI", RollerConstants.Coral.kIVelocity);
        kd = new LoggedTunableNumber(getName() + "/kD", RollerConstants.Coral.kDVelocity);
        kv = new LoggedTunableNumber(getName() + "/kV", RollerConstants.Coral.kVVelocity);
        ka = new LoggedTunableNumber(getName() + "/kA", RollerConstants.Coral.kAVelocity);
        ks = new LoggedTunableNumber(getName() + "/kS", RollerConstants.Coral.kSVelocity);
        maxVelocity = new LoggedTunableNumber(getName() + "/maxVelocity", RollerConstants.Coral.maxVelocity);
        maxAcceleration = new LoggedTunableNumber(getName() + "/maxAcceleration", RollerConstants.Coral.maxAcceleration);
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Coral.kPVelocity, RollerConstants.Coral.kIVelocity, RollerConstants.Coral.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity, RollerConstants.Coral.kVVelocity, RollerConstants.Coral.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Coral.kPPosition, RollerConstants.Coral.kIPosition, RollerConstants.Coral.kDPosition);
    }

    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.velocity);
    }
}
