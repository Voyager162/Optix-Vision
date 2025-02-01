package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;
import frc.robot.commands.elevator.SetElevatorState;

import frc.robot.commands.roller.MaintainCommand;
import frc.robot.commands.roller.RunCommand;
import frc.robot.commands.roller.StopCommand;

import frc.robot.commands.swerve.SwerveDefaultCommand;
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
    private static final Command MaintainCommand = new MaintainCommand();
    private static final Command RunCommand = new RunCommand();
    private static final Command StopCommand = new StopCommand();


    private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);

    private final static GenericHID buttonBoard = new GenericHID(1);

    // coral elevator positions
    private final static JoystickButton buttonl1 = new JoystickButton(buttonBoard, 1);
    private final static JoystickButton buttonl2 = new JoystickButton(buttonBoard, 2);
    private final static JoystickButton buttonl3 = new JoystickButton(buttonBoard, 3);
    private final static JoystickButton buttonl4 = new JoystickButton(buttonBoard, 4);

    // source buttons
    private final static JoystickButton buttonRightSource = new JoystickButton(buttonBoard, 5);
    private final static JoystickButton buttonLeftSource = new JoystickButton(buttonBoard, 6);

    // reef positions
    private final static JoystickButton buttonReefZoneA = new JoystickButton(buttonBoard, 7);
    private final static JoystickButton buttonReefZoneB = new JoystickButton(buttonBoard, 8);
    private final static JoystickButton buttonReefZoneC = new JoystickButton(buttonBoard, 9);
    private final static JoystickButton buttonReefZoneD = new JoystickButton(buttonBoard, 10);
    private final static JoystickButton buttonReefZoneE = new JoystickButton(buttonBoard, 11);
    private final static JoystickButton buttonReefZoneF = new JoystickButton(buttonBoard, 12);
    private final static JoystickButton buttonReefZoneG = new JoystickButton(buttonBoard, 13);
    private final static JoystickButton buttonReefZoneH = new JoystickButton(buttonBoard, 14);
    private final static JoystickButton buttonReefZoneI = new JoystickButton(buttonBoard, 15);
    private final static JoystickButton buttonReefZoneJ = new JoystickButton(buttonBoard, 16);
    private final static JoystickButton buttonReefZoneK = new JoystickButton(buttonBoard, 17);
    private final static JoystickButton buttonReefZoneL = new JoystickButton(buttonBoard, 18);

    // miscellaneous buttons
    private final static JoystickButton buttonAlgaeKnockoff = new JoystickButton(buttonBoard, 19);
    private final static JoystickButton buttonUtilityA = new JoystickButton(buttonBoard, 20);
    private final static JoystickButton buttonUtilityB = new JoystickButton(buttonBoard, 21);
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

        operator.a().onTrue(l1);
        operator.b().onTrue(l2);
        operator.x().onTrue(l3);
        operator.y().onTrue(l4);

        buttonl1.onTrue(l1);
        buttonl2.onTrue(l2);
        buttonl3.onTrue(l3);
        buttonl4.onTrue(l4);
        
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
