// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.auto.AutoUtils;
import frc.robot.commands.auto.Autos;
import frc.robot.commands.elevator.elevatorstop;
import frc.robot.commands.elevator.movetosetpoint;
import frc.robot.utils.JoystickIO;

public class RobotContainer {
  final CommandXboxController controller = new CommandXboxController(0);
  private final movetosetpoint test = new movetosetpoint(Units.feetToMeters(4));
  private final elevatorstop reset = new elevatorstop();
  public RobotContainer() {
    // DriverStation.silenceJoystickConnectionWarning(true);
    // DriverStation.removeRefreshedDataEventHandle(44000);

    // // data logs
    // DataLogManager.start();
    // DataLogManager.logNetworkTables(true);
    // DriverStation.startDataLog(DataLogManager.getLog(), true);

    // // When to go into brownout protection
    // RobotController.setBrownoutVoltage(7.0);

    // // robot setup
    // JoystickIO.getButtonBindings();
    // AutoUtils.initAuto();
    controller.a().onTrue(test);
    controller.b().onTrue(reset);
  }

  public Command getAutonomousCommand() {
    // return new DriveStraight();
    return Autos.getSelectedCommand();
  }
}
