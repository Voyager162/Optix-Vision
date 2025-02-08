package frc.robot.subsystems.swerve;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.subsystems.swerve.SwerveModuleIO.ModuleData;

/**
 * General class for swerve modules that interacts with the
 * interface. Handles all logic relating to individual modules
 * 
 * @author Noah Simon
 * @author Rohin Sood
 * @author Raadwan Masum
 */
public class SwerveModule {

    private String name;
    private SwerveModuleState desiredState = new SwerveModuleState();

    private final SimpleMotorFeedforward drivingFeedFordward;

    private ModuleData moduleData = new ModuleData();
    private SwerveModuleIO moduleIO;

    private double previousSetpointVelocity = 0;
    private int index;

    public SwerveModule(int index, SwerveModuleIO SwerveModule) {
        this.index = index;
        moduleIO = SwerveModule;

        if (index == 0) {
            name = "FL module";
        } else if (index == 1) {
            name = "FR module";
        } else if (index == 2) {
            name = "BL module";
        } else if (index == 3) {
            name = "BR module";
        }

        drivingFeedFordward = new SimpleMotorFeedforward(ModuleConstants.kSDriving,
                ModuleConstants.kVDriving, ModuleConstants.kADriving);
    }

    public String getName() {
        return name;
    }

    /**
     * State has a velocity (m/s) and an angle(0-2pi) component
     * 
     * @return The state of the module as a SwerveModuleState object
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                moduleData.driveVelocityMPerSec,
                new Rotation2d(moduleData.turnPositionRad));
    }

    /**
     * State has a distance (m) and an angle (0-2pi) component
     * 
     * @return The position of the module as a SwerveModuleState object
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(moduleData.drivePositionM, new Rotation2d(moduleData.turnPositionRad));
    }

    /**
     * 
     * @return SwerveModuleState - Angle and velocity point
     */
    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    /**
     * sets the module to a given setpoint state
     * 
     * @param state the SwerveModule state, velocity (m/s) and angle(0-2pi) that
     *              will become the module's setpoint
     */
    public void setDesiredState(SwerveModuleState state) {

        state.optimize(getState().angle);

        // prevent micromovements on the motor
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            state.speedMetersPerSecond = 0;
        }

        this.desiredState = state;

        setDriveSpeed(state.speedMetersPerSecond);
        setTurnPosition(state.angle.getRadians());

    }

    /**
     * 
     * @param speedMetersPerSecond - the drive speed setpoint for the module
     */
    public void setDriveSpeed(double speedMetersPerSecond) {

        double setpointAcceleration = (speedMetersPerSecond - previousSetpointVelocity) / 0.02;
        previousSetpointVelocity = speedMetersPerSecond;

        double feedforward = drivingFeedFordward.calculate(speedMetersPerSecond, setpointAcceleration);
        

        moduleIO.setDriveVelocity(speedMetersPerSecond, feedforward);

    }

    /**
     * 
     * @param positionRad - the angle setpoint (0-2pi) for the module
     */
    public void setTurnPosition(double positionRad) {
        moduleIO.setTurnPosition(positionRad, 0);
    }

    // public void setDriveVoltage(double volts) {
    //     moduleIO.setDriveVoltage(volts);

    // }

    // public void setTurnVoltage(double volts) {
    //     moduleIO.setTurnVoltage(volts);
    // }

    public void setBreakMode(boolean enabled) {
        moduleIO.setDriveBrakeMode(enabled);
        moduleIO.setTurningBrakeMode(enabled);

    }

    public void stop() {
        // setDriveVoltage(0);
        // setTurnVoltage(0);
    }

    public ModuleData getModuleData() {
        return moduleData;
    }

    // called within the swerve subsystem's periodic
    public void periodic() {
        moduleIO.updateData(moduleData);
        // // Logging
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/drive velocity", moduleData.driveVelocityMPerSec);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/drive position", moduleData.drivePositionM);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/drive temperature", moduleData.driveTempCelcius);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/drive applied volts", moduleData.driveAppliedVolts);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/drive current amps", moduleData.driveCurrentAmps);

        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/turn velocity", moduleData.turnVelocityRadPerSec);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/turn position", moduleData.turnPositionRad);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/turn temperature", moduleData.turnTempCelcius);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/turn applied volts", moduleData.turnAppliedVolts);
        Logger.recordOutput("subsystems/swerve/swerveModule " + index + "/turn current amps", moduleData.turnCurrentAmps);

    }
}
