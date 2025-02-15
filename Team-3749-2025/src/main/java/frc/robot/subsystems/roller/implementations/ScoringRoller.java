package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Robot;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;

public class ScoringRoller extends Roller {
    private RollerData rollerData;

    public ScoringRoller() {
        super(Implementations.SCORING, FF());
        this.rollerData = new RollerData();
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity.get(), RollerConstants.Scoring.kVVelocity.get(),
                RollerConstants.Scoring.kAVelocity.get());
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
