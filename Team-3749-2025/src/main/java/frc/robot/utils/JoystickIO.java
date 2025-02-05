package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.example.ExampleSubsystemCommand;
import frc.robot.commands.swerve.DriveStraight;
import frc.robot.commands.swerve.RotationialSysId;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.coral.CoralConstants;
import frc.robot.subsystems.arm.climb.ClimbConstants;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);
    private static final Command MaintainCommand = new MaintainCommand();
    private static final Command RunCommand = new RunCommand();
    private static final Command StopCommand = new StopCommand();

    private static final Command coralStow = new SetCoralArmState(Robot.coralArm, CoralConstants.ArmStates.STOWED,
            CoralConstants.stowSetPoint_rad);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm, CoralConstants.ArmStates.CORAL_PICKUP,
            CoralConstants.coralPickUpSetPoint_rad);

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbConstants.ArmStates.STOWED,
            ClimbConstants.stowSetPoint_rad);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbConstants.ArmStates.CLIMB,
            ClimbConstants.climbSetPoint_rad);
    private static final RotationialSysId rotate1 = new RotationialSysId(Robot.swerve.getDriveSysIdTuner().sysIdQuasistatic(Direction.kForward), Robot.swerve);
    private static final RotationialSysId rotate2 = new RotationialSysId(Robot.swerve.getDriveSysIdTuner().sysIdQuasistatic(Direction.kReverse), Robot.swerve);
    private static final RotationialSysId rotate3 = new RotationialSysId(Robot.swerve.getDriveSysIdTuner().sysIdDynamic(Direction.kForward), Robot.swerve);
    private static final RotationialSysId rotate4 = new RotationialSysId(Robot.swerve.getDriveSysIdTuner().sysIdDynamic(Direction.kReverse), Robot.swerve);



    public JoystickIO() {
    }

    /**
     * Calls binding methods according to the joysticks connected
     */
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
        pilot.a().whileTrue(RunCommand);
        pilot.b().whileTrue(MaintainCommand);
        pilot.x().whileTrue(StopCommand);

        // Example binding
        // operator.a().whileTrue(new ExampleSubsystemCommand());

        operator.a().whileTrue(Robot.elevator.getSysIdTuner().runTests());
        operator.b().whileTrue(Robot.coralArm.getSysIdTuner().runTests());
        operator.x().whileTrue(Robot.climbArm.getSysIdTuner().runTests());
        operator.y().onTrue(rotate4);
        /*operator.a().onTrue(Robot.swerve.getTurningSysIdTuner().sysIdQuasistatic(Direction.kForward));
        operator.b().onTrue(Robot.swerve.getTurningSysIdTuner().sysIdQuasistatic(Direction.kReverse));
        operator.x().onTrue(Robot.swerve.getTurningSysIdTuner().sysIdDynamic(Direction.kForward));
        operator.y().onTrue(Robot.swerve.getTurningSysIdTuner().sysIdDynamic(Direction.kReverse));*/
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
