package frc.robot.subsystems.roller.implementations;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.real.JTVisiSight;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;

public class ScoringRoller extends Roller {
    private RollerData rollerData;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private PhotoelectricIO photoelectricIO;

    private boolean hasPiece = false;
    // private double intakeStartTime = 100000;

    public ScoringRoller() {
        super(Implementations.SCORING, velocityController(), FF(), positionController());
        this.rollerData = new RollerData();
        if (Robot.isSimulation()) {
            this.photoelectricIO = new PhotoelectricSim();
        } else {
            this.photoelectricIO = new JTVisiSight(); 
        }
    }

    public static PIDController velocityController() {
        return new PIDController(RollerConstants.Scoring.kPVelocity, RollerConstants.Scoring.kIVelocity,
                RollerConstants.Scoring.kDVelocity);
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Scoring.kSVelocity, RollerConstants.Scoring.kVVelocity,
                RollerConstants.Scoring.kAVelocity);
    }

    public static PIDController positionController() {
        return new PIDController(RollerConstants.Scoring.kPPosition, RollerConstants.Scoring.kIPosition,
                RollerConstants.Scoring.kDPosition);
    }

    @Override
    public void run() {
        if (!rollerData.sensorTripped) {
            setVelocity(RollerConstants.Scoring.velocity);
        } else {
            setVoltage(0.0);
        }
    }

    @Override
    public void score() {
        setVelocity(RollerConstants.Scoring.scoreVelocity);
    }

    public boolean hasPiece() {
        return hasPiece;
    }

    public void setHasPiece(boolean hasPiece) {
        this.hasPiece = hasPiece;
    }

    @Override
    public void periodic() {
        super.periodic();
        photoelectricIO.updateData(photoelectricData);

        if (Robot.isSimulation()) {
            if (this.getCurrentCommand() != null) {

                photoelectricIO.setSensing(this.getCurrentCommand().getName());
                setHasPiece(photoelectricData.sensing);

                // if (photoelectricData.sensing) {
                //     setHasPiece(true);
                //     // System.out.println("correct");
                // } else {
                //     setHasPiece(false);
                // }
                // if (this.getCurrentCommand().getName() == "IntakeFloor" || this.getCurrentCommand().getName() == "IntakeSource") {
                    
                    // if (intakeStartTime > 10000) {
                    //     intakeStartTime = Timer.getFPGATimestamp();
                    // }
                    // if (Timer.getFPGATimestamp() - intakeStartTime > 2) {
                        // setHasPiece(true);
                    //     intakeStartTime = 100000;
                    // }
            }
        }

    }
}
