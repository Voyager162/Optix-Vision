package frc.robot.buttons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;

import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.arm.SetCoralArmState;
import frc.robot.commands.elevator.SetElevatorState;


import frc.robot.commands.swerve.RotationialSysId;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
@SuppressWarnings("unused")
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);

    private static final RotationialSysId rotate1 = new RotationialSysId(
            Robot.swerve.getDriveSysIdTuner().sysIdQuasistatic(Direction.kForward), Robot.swerve);
    private static final RotationialSysId rotate2 = new RotationialSysId(
            Robot.swerve.getDriveSysIdTuner().sysIdQuasistatic(Direction.kReverse), Robot.swerve);
    private static final RotationialSysId rotate3 = new RotationialSysId(
            Robot.swerve.getDriveSysIdTuner().sysIdDynamic(Direction.kForward), Robot.swerve);
    private static final RotationialSysId rotate4 = new RotationialSysId(
            Robot.swerve.getDriveSysIdTuner().sysIdDynamic(Direction.kReverse), Robot.swerve);

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.STOWED,
        ClimbArmConstants.stowSetPoint_rad);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.CLIMB,
        ClimbArmConstants.climbSetPoint_rad);
    private static final Command coralHandOff = new SetCoralArmState(Robot.coralArm, CoralArmConstants.ArmStates.HAND_OFF,
        CoralArmConstants.handOffSetPoint_rad);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm,
        CoralArmConstants.ArmStates.CORAL_PICKUP,
        CoralArmConstants.coralPickUpSetPoint_rad);

    private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);
    private static ButtonBoard buttonBoard = new ButtonBoard();

    public JoystickIO() {
    }

    public static void getButtonBindings() {

        if (DriverStation.isJoystickConnected(1)) {
            // if both xbox controllers are connected
            pilotAndOperatorBindings();
        } else if (DriverStation.isJoystickConnected(0)) {
            // if only one xbox controller is connected
            pilotBindings();
        } else if (Robot.isSimulation()) {
            // will show not connected if on
            pilotAndOperatorBindings();
            // simBindings();
        } else {

        }

        setDefaultCommands();
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // Example binding
        // operator.a().whileTrue(new ExampleSubsystemCommand());
        buttonBoard.buttonl1.onTrue(Commands.runOnce(() -> System.out.println("1, 5")));
        buttonBoard.buttonl2.onTrue(Commands.runOnce(() -> System.out.println("1, 6")));
        buttonBoard.buttonl3.onTrue(Commands.runOnce(() -> System.out.println("3, 1")));
        buttonBoard.buttonl4.onTrue(Commands.runOnce(() -> System.out.println("3, 2")));

        buttonBoard.buttonRightSource.onTrue(Commands.runOnce(() -> System.out.println("1, 4")));
        buttonBoard.buttonLeftSource.onTrue(Commands.runOnce(() -> System.out.println("1, 7")));

        buttonBoard.buttonReefZoneA.onTrue(Commands.runOnce(() -> System.out.println("3, 6")));
        buttonBoard.buttonReefZoneB.onTrue(Commands.runOnce(() -> System.out.println("2, 5")));
        buttonBoard.buttonReefZoneC.onTrue(Commands.runOnce(() -> System.out.println("2, 6")));
        buttonBoard.buttonReefZoneD.onTrue(Commands.runOnce(() -> System.out.println("3, 3")));
        buttonBoard.buttonReefZoneE.onTrue(Commands.runOnce(() -> System.out.println("3, 4")));
        buttonBoard.buttonReefZoneF.onTrue(Commands.runOnce(() -> System.out.println("2, 8")));
        buttonBoard.buttonReefZoneG.onTrue(Commands.runOnce(() -> System.out.println("2, 7")));
        buttonBoard.buttonReefZoneH.onTrue(Commands.runOnce(() -> System.out.println("2, 4")));
        buttonBoard.buttonReefZoneI.onTrue(Commands.runOnce(() -> System.out.println("2, 3")));
        buttonBoard.buttonReefZoneJ.onTrue(Commands.runOnce(() -> System.out.println("2, 2")));
        buttonBoard.buttonReefZoneK.onTrue(Commands.runOnce(() -> System.out.println("2, 1")));
        buttonBoard.buttonReefZoneL.onTrue(Commands.runOnce(() -> System.out.println("3, 5")));

        buttonBoard.buttonAlgaeKnockoff.onTrue(Commands.runOnce(() -> System.out.println("1, 3")));
        buttonBoard.buttonUtilityA.onTrue(Commands.runOnce(() -> System.out.println("1, 1")));
        buttonBoard.buttonUtilityB.onTrue(Commands.runOnce(() -> System.out.println("1, 2")));
        buttonBoard.buttonPlayer1Start.onTrue(Commands.runOnce(() -> System.out.println("1, 8")));

  

        // operator.a().whileTrue(Robot.climbArm.getSysIdTuner().runTests());
        // operator.b().whileTrue(Robot.coralArm.getSysIdTuner().runTests());
        // operator.x().whileTrue(Robot.elevator.getSysIdTuner().runTests());

        // operator.a().whileTrue(Robot.swerve.getRotationalSysIdTuner().runTests());
        // operator.b().whileTrue(Robot.swerve.getDriveSysIdTuner().runTests());

        // operator.a().whileTrue(Robot.algaeRoller.getSysIdTuner().runTests());
        // operator.b().whileTrue(Robot.coralRoller.getSysIdTuner().runTests());
        // operator.x().whileTrue(Robot.scoringRoller.getSysIdTuner().runTests());

        // operator.a().onTrue(l1);
        // operator.b().onTrue(l2);
        // operator.x().onTrue(l3);
        // operator.y().onTrue(l4);

        // operator.a().onTrue(Commands.run(() -> Robot.elevator.setVoltage(0)));
        // operator.a().onTrue(Commands.run(() -> Robot.coralArm.setVoltage(0)));
        // operator.a().onTrue(Commands.run(() -> Robot.climbArm.setVoltage(0)));
        // operator.a().onTrue(Commands.run(() -> Robot.algaeRoller.setVoltage(0)));
        // operator.a().onTrue(Commands.run(() -> Robot.coralRoller.setVoltage(0)));
        // operator.a().onTrue(Commands.run(() -> Robot.scoringRoller.setVoltage(0)));
        operator.a().onTrue(climbStow);
        operator.b().onTrue(climb);
        operator.x().onTrue(coralHandOff);
        operator.y().onTrue(coralPickUp);
    }

    public static void pilotBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // Example binding
    }

    public static void simBindings() {
        pilotBindings();
    }

    /**
     * Sets the default commands
     */
    public static void setDefaultCommands() {
        if (Robot.isSimulation()) {
            setSimDefaultCommands();
        } else {
            setRealDefaultCommands();
        }
    }

    private static void setRealDefaultCommands() {
        Robot.swerve.setDefaultCommand(
                new SwerveDefaultCommand(
                        () -> pilot.getLeftX(),
                        () -> pilot.getLeftY(),
                        () -> pilot.getRightX()));
    }

    private static void setSimDefaultCommands() {
        Robot.swerve
                .setDefaultCommand(
                        new SwerveDefaultCommand(
                                () -> pilot.getLeftX(),
                                () -> pilot.getLeftY(),
                                () -> {
                                    if (pilot.y().getAsBoolean()) {
                                        return 1.0;
                                    }
                                    return 0.0;
                                }));
    }

}
