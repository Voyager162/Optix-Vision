package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.utils.LoggedTunableNumber;

public class CoralRoller extends Roller {
    
    public CoralRoller() {
        super(Implementations.CORAL, FF());
    
        ks = new LoggedTunableNumber(getName() + "/kS", RollerConstants.Coral.kSVelocity);
        kv = new LoggedTunableNumber(getName() + "/kV", RollerConstants.Coral.kVVelocity);
        ka = new LoggedTunableNumber(getName() + "/kA", RollerConstants.Coral.kAVelocity);
        
        RollerConstants.Coral.kSVelocity = ks.get();
        RollerConstants.Coral.kVVelocity = kv.get();
        RollerConstants.Coral.kAVelocity = ka.get();

        maxVelocity = new LoggedTunableNumber(getName() + "/maxVelocity", RollerConstants.Coral.maxVelocity);
        maxAcceleration = new LoggedTunableNumber(getName() + "/maxAcceleration", RollerConstants.Coral.maxAcceleration);
    }


    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity, RollerConstants.Coral.kVVelocity, RollerConstants.Coral.kAVelocity);
    }



    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.velocity);
    }
}
