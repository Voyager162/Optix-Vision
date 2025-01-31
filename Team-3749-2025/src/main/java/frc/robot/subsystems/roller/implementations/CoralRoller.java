package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;

public class CoralRoller extends Roller {
    private double lastVelocity = 0.0;
    private boolean hasPiece = false;
    private RollerData data;
    
    public CoralRoller() {
        super(Implementations.CORAL, velocityController(), FF(), positionController());
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

    public boolean getIsStableState() {
        double currentVelocity = data.rollerVelocityRadPerSec;
        double velocityChange = Math.abs(currentVelocity - lastVelocity);

        double velocityChangeThreshold = 0.5; // placeholder
        double velocityDropThreshold = 0.1;

        if (currentVelocity < lastVelocity - velocityDropThreshold) {
            hasPiece = true;
        }
        boolean isStable = velocityChange < velocityChangeThreshold;
        
        return isStable;
    }

    public boolean hasPiece() {
        return hasPiece();
    }

    public void setHasPiece(boolean hasPiece) {
        this.hasPiece = hasPiece;
    }
    
    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.velocity);
    }

    @Override
    public void score() {
        setVelocity(RollerConstants.Coral.scoreVelocity);
    }
}
