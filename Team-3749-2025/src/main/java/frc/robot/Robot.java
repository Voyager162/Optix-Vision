// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import java.util.Arrays;
import java.util.List;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.example.ExampleSubsystem;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.ShuffleData;

public class Robot extends TimedRobot {
    private static final List<Translation2d> HEXAGON_VERTICES = List.of(
        new Translation2d(4.5 + 1.0, 4.0),
        new Translation2d(4.5 + 0.5, 4.0 + Math.sqrt(3) / 2),
        new Translation2d(4.5 - 0.5, 4.0 + Math.sqrt(3) / 2),
        new Translation2d(4.5 - 1.0, 4.0),
        new Translation2d(4.5 - 0.5, 4.0 - Math.sqrt(3) / 2),
        new Translation2d(4.5 + 0.5, 4.0 - Math.sqrt(3) / 2),
        new Translation2d(4.5 + 1.0, 4.0) // Close loop
    );
  private Command m_autonomousCommand;

  public static Swerve swerve = new Swerve();
  public static ExampleSubsystem subsystem = new ExampleSubsystem();

  private ShuffleData<Double> batteryVoltageLog = new ShuffleData<Double>("DS", "battery voltage", 0.0);
  private ShuffleData<Boolean> isBrownedOutLog = new ShuffleData<Boolean>("DS", "brownout", false);
  private ShuffleData<Double> cpuTempLog = new ShuffleData<Double>("DS", "cpu temp", 0.0);
  private ShuffleData<Double> CANUtilizationLog = new ShuffleData<Double>("DS", "CAN utilizaition", 0.0);
  private ShuffleData<String> radioStatusLog = new ShuffleData<String>("DS", "radio status", "kOff");
  private ShuffleData<String> allianceLog = new ShuffleData<String>("DS", "alliance", "Red");
  private ShuffleData<Boolean> FMSLog = new ShuffleData<Boolean>("DS", "FMS connected", false);
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    batteryVoltageLog.set(RobotController.getBatteryVoltage());
    cpuTempLog.set(RobotController.getCPUTemp());
    CANUtilizationLog.set(RobotController.getCANStatus().percentBusUtilization);
    radioStatusLog.set(RobotController.getRadioLEDState().name());
    isBrownedOutLog.set(RobotController.isBrownedOut());
    allianceLog.set(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get().name() : "None");
    FMSLog.set(DriverStation.isFMSAttached());
    // Publish hexagon points to NetworkTables
        double[][] hexagonArray = new double[HEXAGON_VERTICES.size()][2];
        for (int i = 0; i < HEXAGON_VERTICES.size(); i++) {
            hexagonArray[i][0] = HEXAGON_VERTICES.get(i).getX();
            hexagonArray[i][1] = HEXAGON_VERTICES.get(i).getY();
        }
        NetworkTableInstance.getDefault()
            .getTable("Field")
            .getEntry("HexagonPoints")
            .setDoubleArray(flattenArray(hexagonArray));
        
        // Optional: Display in SmartDashboard for debugging
        SmartDashboard.putString("Hexagon Points", HEXAGON_VERTICES.toString());
  }
   private double[] flattenArray(double[][] array) {
        return Arrays.stream(array).flatMapToDouble(Arrays::stream).toArray();
    }

  @Override
  public void disabledInit() {
    swerve.setBreakMode(false);
  }

  @Override
  public void disabledPeriodic() {

  }

  @Override
  public void disabledExit() {
    swerve.setBreakMode(true);
  }

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }
  @Override
  public void simulationInit(){
    DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
  }
}
