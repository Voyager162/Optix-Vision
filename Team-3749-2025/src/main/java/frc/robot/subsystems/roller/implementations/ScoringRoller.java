package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Robot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;

public class ScoringRoller extends Roller {
    @SuppressWarnings("unused")
    private RollerData rollerData;

    public ScoringRoller() {
        super(Implementations.SCORING, FF(), positionPID(), velocityPID());
        this.rollerData = new RollerData();
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity.get(), RollerConstants.Scoring.kVVelocity.get(),
                RollerConstants.Scoring.kAVelocity.get());
    }

        public static PIDController positionPID(){
        return new PIDController(RollerConstants.Scoring.kPPosition.get(), RollerConstants.Scoring.kIPosition.get(),RollerConstants.Scoring.kDPosition.get());
    }

    public static PIDController velocityPID(){
        return new PIDController(RollerConstants.Scoring.kPVelocity.get(), RollerConstants.Scoring.kIVelocity.get(),RollerConstants.Scoring.kDVelocity.get());
    }
    @Override
    public void run() {
        // if (!rollerData.sensorTripped) {
        //     setVelocity(RollerConstants.Scoring.velocity);
        // } else {
            setVoltage(Robot.subsystemVoltageSetter.get());
        // }
    }

    @Override
    public void outtake() {}
}
