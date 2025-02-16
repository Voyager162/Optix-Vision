package frc.robot.buttons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;

import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.arm.SetCoralArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.roller.MaintainCommand;
import frc.robot.commands.roller.OuttakeRoller;
import frc.robot.commands.roller.RunRoller;
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
    private static final Command coralHandOff = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.HAND_OFF,
            CoralArmConstants.handOffSetPoint_rad);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.CORAL_PICKUP,
            CoralArmConstants.coralPickUpSetPoint_rad);

    private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);

    private static final RunRoller algaeRun = new RunRoller(Robot.algaeRoller);
    private static final RunRoller coralRunIntake = new RunRoller(Robot.coralRoller);
    private static final OuttakeRoller coralRunOuttake = new OuttakeRoller(Robot.coralRoller);
    private static final RunRoller scoringRun = new RunRoller(Robot.scoringRoller);

    private static final MaintainCommand algaeMaintain = new MaintainCommand(Robot.algaeRoller);
    private static final MaintainCommand coralMaintain = new MaintainCommand(Robot.coralRoller);
    private static final MaintainCommand scoringMaintain = new MaintainCommand(Robot.scoringRoller);

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
        // pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        
        // // Checking voltage for all subsystems
        // operator.a().onTrue(Commands.run(() -> Robot.elevator.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.b().onTrue(Commands.run(() -> Robot.coralArm.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.x().onTrue(Commands.run(() -> Robot.climbArm.setVoltage(Robot.subsystemVoltageSetter.get())));
        operator.y().whileTrue(Commands.run(() -> Robot.algaeRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.leftBumper().whileTrue(Commands.run(() -P> Robot.coralRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.leftBumper().whileFalse(Commands.runOnce(() -> Robot.coralRoller.stop()));
        // operator.rightBumper().onTrue(Commands.run(() -> Robot.scoringRoller.setVoltage(Robot.subsystemVoltageSetter.get())));

        // Climb, Coral, Elevator SysId
        // operator.a().whileTrue(Robot.climbArm.getSysIdTuner().runTests());
        // operator.b().whileTrue(Robot.coralArm.getSysIdTuner().runTests());
        // operator.x().whileTrue(Robot.elevator.getSysIdTuner().runTests());

        // Swerve SysId
        // operator.a().whileTrue(Robot.swerve.getRotationalSysIdTuner().runTests());
        // operator.b().whileTrue(Robot.swerve.getDriveSysIdTuner().runTests());

        // Roller SysId
        // operator.a().onTrue(Robot.algaeRoller.getSysIdTuner().runTests());
        // operator.b().onTrue(Robot.coralRoller.getSysIdTuner().runTests());
        // operator.x().onTrue(Robot.scoringRoller.getSysIdTuner().runTests());

        // All elevator stages
        // operator.a().onTrue(l1);
        // operator.b().onTrue(l2);
        // operator.x().onTrue(l3);
        // operator.y().onTrue(l4);

        // Climb + Coral Arms
        // operator.a().onTrue(climbStow);
        // operator.b().onTrue(climb);
 

        // Run
        // operator.a().whileTrue(algaeRun);
        // operator.b().onTrue(coralRunIntake);
        // operator.a().onTrue(coralRunOuttake);
        // operator.x().onTrue(coralHandOff);
        // operator.y().onTrue(coralPickUp);

        // operator.x().whileTrue(scoringRun);

        // Maintain
        // operator.a().whileTrue(algaeMaintain);
        // operator.b().whileTrue(coralMaintain);
        // operator.x().whileTrue(scoringMaintain);
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
