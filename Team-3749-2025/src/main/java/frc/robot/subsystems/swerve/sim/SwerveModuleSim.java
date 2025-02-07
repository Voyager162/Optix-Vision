package frc.robot.subsystems.swerve.sim;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DriveConstants;
import frc.robot.subsystems.swerve.SwerveConstants.ModuleConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * Simulation implementation for swerve modules
 * Very closely inspired by 6328's Swerve Sim code,
 * 
 * @see https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/drive/ModuleIOSim.java
 * 
 * @author Noah Simon
 */
public class SwerveModuleSim implements SwerveModuleIO {

    LinearSystem<N1, N1, N1> drivePlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1), // Motor
            0.09, // J (moment of inertia)
            ModuleConstants.driveMotorGearRatio // Gear ratio
    );
    FlywheelSim driveSim = new FlywheelSim(
            drivePlant, // The linear system
            DCMotor.getNEO(1), // The motor (gearbox) model
            0.0 // Optional noise in sensor measurements
    );
    LinearSystem<N1, N1, N1> turnPlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1), // Motor
            0.04, // J (moment of inertia)
            ModuleConstants.turnMotorGearRatio // Gear ratio
    );
    FlywheelSim turnSim = new FlywheelSim(
            turnPlant, // The linear system
            DCMotor.getNEO(1), // The motor (gearbox) model
            0.0 // Optional noise in sensor measurements
    );

    private final PIDController turningPidController;
    private final PIDController drivingPidController;

    private double turnPositionRad = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;
    private int index;

    public SwerveModuleSim(int index) {
        this.index = index;
        System.out.println("[Init] Creating ModuleIOSim");

        drivingPidController = new PIDController(ModuleConstants.drivePID[2][0], ModuleConstants.drivePID[2][1],
                ModuleConstants.drivePID[2][2]);

        turningPidController = new PIDController(ModuleConstants.turnPID[0][0], ModuleConstants.turnPID[0][1],
                ModuleConstants.turnPID[0][2]);
        turningPidController.enableContinuousInput(0, 2 * Math.PI);

    }

    @Override
    public void updateData(ModuleData data) {
        // update sim values
        driveSim.update(SimConstants.loopPeriodSec);
        turnSim.update(SimConstants.loopPeriodSec);

        // how far have we turned in the previous loop?
        double angleDiffRad = turnSim.getAngularVelocityRadPerSec() * SimConstants.loopPeriodSec;
        // update our angle variables
        turnPositionRad += angleDiffRad;
        // keep our absolute position within 0-2 pi
        while (turnPositionRad < 0) {
            turnPositionRad += 2.0 * Math.PI;
        }
        while (turnPositionRad > 2.0 * Math.PI) {
            turnPositionRad -= 2.0 * Math.PI;
        }
        // distance traveled + Rad/Time * Time * diameter
        data.drivePositionM = data.drivePositionM
                + getDriveVelocityMetersPerSec() * 0.02;
        data.driveVelocityMPerSec = getDriveVelocityMetersPerSec();
        data.driveAppliedVolts = driveAppliedVolts;
        // System.out.println(Integer.toString(index) + ": " + driveAppliedVolts);

        data.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());
        data.driveTempCelcius = 0;

        data.turnPositionRad = turnPositionRad;
        data.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
        data.turnAppliedVolts = turnAppliedVolts;
        data.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());
        data.turnTempCelcius = 0;

    }

    @Override
    public void setDriveVelocity(double setpointVelocity, double feedforward) {
        double feedback = drivingPidController.calculate(getDriveVelocityMetersPerSec(), setpointVelocity);
        setDriveVoltage(feedback + feedforward);

    }

    @Override
    public void setTurnPosition(double setpointPosition, double feedforward) {
        double feedback = turningPidController.calculate(turnPositionRad, setpointPosition);
        setTurnVoltage(feedback + feedforward);

    }

    @Override
    public void setDriveVoltage(double volts) {

        driveAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        // System.out.println(Integer.toString(index) + ": " + driveAppliedVolts);
        // driveAppliedVolts = Math.copySign(12,driveAppliedVolts);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -DriveConstants.maxMotorVolts,
                DriveConstants.maxMotorVolts);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    private double getDriveVelocityMetersPerSec() {
        return (driveSim.getAngularVelocityRadPerSec() * ModuleConstants.wheelDiameterMeters) / 2;
    }
}