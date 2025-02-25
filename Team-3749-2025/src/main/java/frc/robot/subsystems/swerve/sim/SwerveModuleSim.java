package frc.robot.subsystems.swerve.sim;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.subsystems.swerve.SwerveModuleIO;
import frc.robot.subsystems.swerve.SwerveConstants.DrivetrainConstants;
import frc.robot.subsystems.swerve.SwerveConstants.MotorConstants;
import frc.robot.utils.MiscConstants.MotorControllerConstants;
import frc.robot.utils.MiscConstants.SimConstants;

/**
 * Simulation implementation for swerve modules
 * Very closely inspired by 6328's Swerve Sim code,
 * 
 * @see https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023 drive/ModuleIOSim.java
 * 
 * @author Noah Simon
 */
public class SwerveModuleSim implements SwerveModuleIO {

    LinearSystem<N1, N1, N1> drivePlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1), // Motor
            0.09, // J (moment of inertia)
            MotorConstants.driveMotorGearRatio // Gear ratio
    );
    FlywheelSim driveSim = new FlywheelSim(
            drivePlant, // The linear system
            DCMotor.getNEO(1), // The motor (gearbox) model
            0.0 // Optional noise in sensor measurements
    );
    LinearSystem<N1, N1, N1> turnPlant = LinearSystemId.createFlywheelSystem(
            DCMotor.getNEO(1), // Motor
            0.04, // J (moment of inertia)
            MotorConstants.turnMotorGearRatio // Gear ratio
    );
    FlywheelSim turnSim = new FlywheelSim(
            turnPlant, // The linear system
            DCMotor.getNEO(1), // The motor (gearbox) model
            0.0 // Optional noise in sensor measurements
    );


    private double turnPositionRad = 0;
    private double driveAppliedVolts = 0.0;
    private double turnAppliedVolts = 0.0;

    public SwerveModuleSim() {
        System.out.println("[Init] Creating ModuleIOSim");

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
    public void setDriveVoltage(double volts) {

        driveAppliedVolts = MathUtil.clamp(volts, -MotorControllerConstants.maxMotorVolts,
                MotorControllerConstants.maxMotorVolts);
        driveSim.setInputVoltage(driveAppliedVolts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnAppliedVolts = MathUtil.clamp(volts, -MotorControllerConstants.maxMotorVolts,
                MotorControllerConstants.maxMotorVolts);
        turnSim.setInputVoltage(turnAppliedVolts);
    }

    private double getDriveVelocityMetersPerSec() {
        return (driveSim.getAngularVelocityRadPerSec() * DrivetrainConstants.wheelDiameterMeters) / 2;
    }
}