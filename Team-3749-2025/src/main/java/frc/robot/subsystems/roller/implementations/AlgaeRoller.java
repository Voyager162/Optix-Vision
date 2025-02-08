package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.utils.LoggedTunableNumber;

public class AlgaeRoller extends Roller {

    public AlgaeRoller() {
        super(Implementations.ALGAE, FF()); 

        kp = new LoggedTunableNumber(getName() + "/kP", RollerConstants.Algae.kPVelocity);
        ki = new LoggedTunableNumber(getName() + "/kI", RollerConstants.Algae.kIVelocity);
        kd = new LoggedTunableNumber(getName() + "/kD", RollerConstants.Algae.kDVelocity);
        kv = new LoggedTunableNumber(getName() + "/kV", RollerConstants.Algae.kVVelocity);
        ka = new LoggedTunableNumber(getName() + "/kA", RollerConstants.Algae.kAVelocity);
        ks = new LoggedTunableNumber(getName() + "/kS", RollerConstants.Algae.kSVelocity);
        maxVelocity = new LoggedTunableNumber(getName() + "/maxVelocity", RollerConstants.Algae.maxVelocity);
        maxAcceleration = new LoggedTunableNumber(getName() + "/maxAcceleration", RollerConstants.Algae.maxAcceleration);
    }

 
    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Algae.kSVelocity, RollerConstants.Algae.kVVelocity, RollerConstants.Algae.kAVelocity);
    }


    @Override
    public void run() {
        setVelocity(RollerConstants.Algae.velocity);
    }
    
}
