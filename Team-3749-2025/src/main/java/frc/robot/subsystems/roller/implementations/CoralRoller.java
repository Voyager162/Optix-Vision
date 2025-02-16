package frc.robot.subsystems.roller.implementations;

import edu.wpi.first.math.controller.PIDController;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.commands.auto.Autos;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.subsystems.roller.RollerConstants.Implementations;
import frc.robot.subsystems.roller.RollerIO.RollerData;
import frc.robot.subsystems.roller.sim.PhotoelectricSim;


/**
 * Coral implementation of the roller subsystem
 *
 * @author Lilian Wu
 */
public class CoralRoller extends Roller {
    private double lastVelocity = 0.0;
    private boolean hasPiece = true;
    private PhotoelectricIO photoelectricIO;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private RollerData rollerData = new RollerData();
    private boolean routineStarted = false;
    
    public CoralRoller() {
        super(Implementations.CORAL, FF(), positionPID(), velocityPID());
        photoelectricIO = new PhotoelectricSim();
    }

    public static SimpleMotorFeedforward FF() {
        return new SimpleMotorFeedforward(RollerConstants.Coral.kSVelocity.get(), RollerConstants.Coral.kVVelocity.get(),
                RollerConstants.Coral.kAVelocity.get());
    }
        public static PIDController positionPID(){
        return new PIDController(RollerConstants.Coral.kPPosition.get(), RollerConstants.Coral.kIPosition.get(),RollerConstants.Coral.kDPosition.get());
    }

    public static PIDController velocityPID(){
        return new PIDController(RollerConstants.Coral.kPVelocity.get(), RollerConstants.Coral.kIVelocity.get(),RollerConstants.Coral.kDVelocity.get());
    }

    public boolean getIsStableState() {
        double currentVelocity = rollerData.rollerVelocityRadPerSec;
        double velocityChange = Math.abs(currentVelocity - lastVelocity);
        double velocityChangeThreshold = 0.05; 

        switch(Robot.coralRoller.getState()) {
            case INTAKE:
                return velocityChange < velocityChangeThreshold;
            case SCORE:
                return velocityChange < velocityChangeThreshold;
            case MAINTAIN:
                return false;
            case STOP:
                return true;
            default:
                return false;
        }
    }

    public boolean hasPiece() {
        if (Robot.isSimulation()) {
           return hasPiece; 
        } else {
            if (!getIsStableState()) { 
                hasPiece = true; 
            }
            return hasPiece;
        }
    }

    public boolean getHasPiece() {
        return hasPiece;
    }

    /**
     * Implemetation of run method
     */
    @Override
    public void run() {
        setVelocity(RollerConstants.Coral.intakeVelocity.get());
    }
        /**
     * Implemetation of run method
     */
    @Override
    public void outtake() {
        setVelocity(RollerConstants.Coral.intakeVelocity.get());
    }
    /**
     * Implemetation of score method
     */
    public void score() {
        setVelocity(RollerConstants.Coral.scoreVelocity);
    }

    @Override
    public void periodic() {
        super.periodic();
        photoelectricIO.updateData(photoelectricData);
        hasPiece = photoelectricData.sensing;

        Logger.recordOutput("subsystems/rollers/coral/hasPiece", hasPiece);
        Logger.recordOutput("subsystems/rollers/coral/setInitalState", routineStarted);

        // routineStarted is true when the routine begins in Autos 
        if (Autos.isRoutineStarted() && !routineStarted) { 
            routineStarted = true; 
            // sets initial state at the start of each routine
            photoelectricIO.setInitialState(true);
        }
        
        // routineStarted is false when the routine ends in Autos 
        if (!Autos.isRoutineStarted() && routineStarted) {
            routineStarted = false;  
        }

        if (this.getCurrentCommand() != null) {
            SmartDashboard.putString("coral roller command", this.getCurrentCommand().getName());
        } else {
            SmartDashboard.putString("coral roller command", "null");
        }
    }
}
