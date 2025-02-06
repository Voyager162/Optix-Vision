package frc.robot.subsystems.arm.coral;

import frc.robot.Robot;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.arm.real.ArmSparkMax;
import frc.robot.subsystems.arm.sim.ArmSim;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.PhotoelectricIO.PhotoelectricData;
import frc.robot.utils.ShuffleData;
import frc.robot.utils.UtilityFunctions;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

/**
 * Subsystem class for the arm
 * 
 * @author Weston Gardner
 */

public class CoralArm extends Arm {

    private CoralConstants.ArmStates state = CoralConstants.ArmStates.STOPPED;
    // private boolean hasPiece;
    private PhotoelectricData photoelectricData = new PhotoelectricData();
    private PhotoelectricIO photoelectricIO;
    private PIDController controller = new PIDController(
            CoralConstants.kP,
            CoralConstants.kI,
            CoralConstants.kD);

    private ShuffleData<String> stateLog = new ShuffleData<String>(this.getName(), "state", state.name());

    private Mechanism2d mechanism2d = new Mechanism2d(3, 3);
    private MechanismRoot2d armRoot = mechanism2d.getRoot("ArmRoot", 1.8, .4);
    private MechanismLigament2d armLigament = armRoot.append(new MechanismLigament2d("Coral Arm", CoralConstants.armLength_meters, 0));
    // private Pose3d zeroedComponentPose = new Pose3d(0, 0, 0, new Rotation3d(roll,
    // pitch, yaw));

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
            .getStructTopic("CoralArm Pose", Pose3d.struct).publish();

    /**
     * Constructor for the CoralArm subsystem.
     * Determines if simulation or real hardware is used.
     */
    public CoralArm() {
        if (Robot.isSimulation()) {

            armIO = new ArmSim(
                    CoralConstants.numMotors,
                    CoralConstants.armGearing,
                    CoralConstants.momentOfInertia,
                    CoralConstants.armLength_meters,
                    CoralConstants.armMinAngle_degrees,
                    CoralConstants.armMaxAngle_degrees,
                    CoralConstants.simulateGravity,
                    CoralConstants.armStartingAngle_degrees);

        } else {
            armIO = new ArmSparkMax(CoralConstants.motorId);
            // this.photoelectricIO = new JTVisiSight();
        }
        SmartDashboard.putData("Coral Arm Mechanism", mechanism2d);
        // System.out.println(data.positionUnits);
    }

    /**
     * @return the current arm state.
     */
    public CoralConstants.ArmStates getState() {
        return state;
    }

    @Override
    public void stop() {
        setState(CoralConstants.ArmStates.STOPPED);
    }

    /**
     * @return whether the arm is in a stable state.
     */
    public boolean getIsStableState() {

        switch (state) {
            case STOWED:
                return UtilityFunctions.withinMargin(0.01, CoralConstants.stowSetPoint_rad,
                        data.positionUnits);
            case HAND_OFF:
                return UtilityFunctions.withinMargin(0.01, CoralConstants.handOffSetPoint_rad,
                        data.positionUnits);
            case CORAL_PICKUP:
                return UtilityFunctions.withinMargin(0.01, CoralConstants.coralPickUpSetPoint_rad,
                        data.positionUnits);
            case L1:
                return UtilityFunctions.withinMargin(0.01, CoralConstants.L1SetPoint_rad,
                        data.positionUnits);
            case SOURCE:
                return UtilityFunctions.withinMargin(0.01, CoralConstants.sourceSetPoint_rad,
                        data.positionUnits);
            case MOVING_DOWN:
                return data.velocityUnits < 0;
            case MOVING_UP:
                return data.velocityUnits > 0;
            case STOPPED:
                return UtilityFunctions.withinMargin(0.01, 0, data.velocityUnits);
            default:
                return false;
        }
    }

    /**
     * Sets the current state of the arm.
     * 
     * @param state The new state for the arm.
     */
    @Override
    public void setState(Enum<?> state) {
        this.state = (CoralConstants.ArmStates) state;
    }

    /**
     * Runs the logic for the current arm state.
     */
    private void runState() {
        switch (state) {
            case STOWED:
                setVoltage(controller.calculate(data.positionUnits, CoralConstants.stowSetPoint_rad)
                        + calculateFeedForward());
                break;
            case HAND_OFF:
                setVoltage(controller.calculate(data.positionUnits, CoralConstants.handOffSetPoint_rad)
                        + calculateFeedForward());
                break;
            case CORAL_PICKUP:
                setVoltage(controller.calculate(data.positionUnits, CoralConstants.coralPickUpSetPoint_rad)
                        + calculateFeedForward());

                break;
            case L1:
                setVoltage(controller.calculate(data.positionUnits, CoralConstants.L1SetPoint_rad)
                        + calculateFeedForward());
                break;
            case SOURCE:
                setVoltage(controller.calculate(data.positionUnits, CoralConstants.sourceSetPoint_rad)
                        + calculateFeedForward());
                break;
            case STOPPED:
                setVoltage(0 + calculateFeedForward());
                break;
            case MOVING_DOWN:
                setVoltage(-1 + calculateFeedForward());
                break;
            case MOVING_UP:
                setVoltage(1 + calculateFeedForward());
                break;
            default:
                break;
        }
    }

    /**
     * Logs data to Shuffleboard.
     */
    private void logData() {
        currentCommandLog.set(this.getCurrentCommand() == null ? "None" : this.getCurrentCommand().getName());
        positionUnitsLog.set(data.positionUnits);
        velocityUnitsLog.set(data.velocityUnits);
        inputVoltsLog.set(data.inputVolts);
        appliedVoltsLog.set(data.appliedVolts);
        currentAmpsLog.set(data.currentAmps);
        tempCelciusLog.set(data.tempCelcius);

        armLigament.setAngle(Math.toDegrees(data.positionUnits));

        stateLog.set(state.name());

        // Logger.recordOutput("zeropose", zeroedComponentPose);

        publisher.set(new Pose3d(getTransform3d().getTranslation(), getTransform3d().getRotation()));
        // elevatorPose3dLog.set(
        // new Double[] {
        // getTransform3d().getTranslation().getX(),
        // getTransform3d().getTranslation().getY(),
        // getTransform3d().getTranslation().getZ(),
        // getTransform3d().getRotation().getAngle()
        // }
        // )
    }

    private Angle getPitch() {
        // System.out.println(data.positionUnits);
        return Angle.ofBaseUnits(data.positionUnits + Units.degreesToRadians(-55), Radians); // remove offset once coral
                                                                                             // arm code is fixed
    }

    private Transform3d getTransform3d() {
        Transform3d transform = new Transform3d(0.285, 0.01, 0.4,
                new Rotation3d(Angle.ofBaseUnits(0, Radians), getPitch(), Angle.ofBaseUnits(0, Radians)));
        return transform;
    }

    private double calculateFeedForward() {
        double feedForward = CoralConstants.kG * Math.cos(data.positionUnits);
        return feedForward;
    }

    // public boolean hasPiece() {
    //     return photoelectricData.sensing;
    //     // return hasPiece;
    // }

    // public void setHasPiece(boolean hasPiece) {
    //     this.hasPiece = hasPiece;
    // }

    public PhotoelectricIO getPhotoElectricIO() {
        return photoelectricIO;
    }

    /**
     * Periodic method for updating arm behavior.
     */
    @Override
    public void periodic() {
        super.periodic();
        armIO.updateData(data);

        logData();
        runState();

        // Ensure photoelectricData is updated

        // photoelectricIO.updateData(photoelectricData);

        // if (Robot.isSimulation()) {
        //     if (this.getCurrentCommand() != null) {
        //         photoelectricIO.setSensing(this.getCurrentCommand().getName());
        //         // Update hasPiece based on sensing
        //         // setHasPiece(photoelectricData.sensing);

        //         // Debugging logs
        //         // System.out.println("Command: " + this.getCurrentCommand().getName());
        //         // System.out.println("Sensing: " + photoelectricData.sensing);
        //     }
        // }
    }
}
