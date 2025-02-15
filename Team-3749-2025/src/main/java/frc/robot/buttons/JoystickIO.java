package frc.robot.buttons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Robot;


import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.OuttakeCoral;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;

import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;

import frc.robot.commands.swerve.SwerveDefaultCommand;

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


    private static final KnockAlgae knockAlgaeLow = new KnockAlgae(ElevatorStates.ALGAE_LOW);
    private static final KnockAlgae knockAlgaeHigh = new KnockAlgae(ElevatorStates.ALGAE_HIGH);
    private static final Handoff handoff = new Handoff();
    private static final IntakeFloor intakeFloor = new IntakeFloor();
    private static final IntakeSource intakeSource = new IntakeSource();
    private static final CoralIntakeSource coralIntakeSource = new CoralIntakeSource();
    private static final OuttakeCoral outtakeCoral = new OuttakeCoral();
    private static final ScoreL1 scoreL1 = new ScoreL1();
    private static final ScoreL234 scoreL4 = new ScoreL234(ElevatorStates.L4);

    // private static final SetArmState climbArm = new Set
    private static final ButtonBoard buttonBoard = new ButtonBoard();

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
 
        // Example binding
        // operator.a().whileTrue(new ExampleSubsystemCommand());
        buttonBoard.buttonl1.onTrue(Commands.runOnce(() -> System.out.println("1, 5")));
        buttonBoard.buttonl2.onTrue(Commands.runOnce(() -> System.out.println("1, 6")));
        buttonBoard.buttonl3.onTrue(Commands.runOnce(() -> System.out.println("3, 1")));
        buttonBoard.buttonl4.onTrue(Commands.runOnce(() -> System.out.println("3, 2")));

        operator.a().onTrue(handoff);
        operator.b().onTrue(intakeFloor);
        operator.x().onTrue(intakeSource);
        operator.y().onTrue(scoreL4);

        pilot.a().onTrue(knockAlgaeHigh);
        pilot.b().onTrue(scoreL1);
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
