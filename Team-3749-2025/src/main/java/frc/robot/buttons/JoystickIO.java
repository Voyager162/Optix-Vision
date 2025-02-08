package frc.robot.buttons;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringLocation;
import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.example.ExampleSubsystemCommand;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.OuttakeCoral;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.commands.swerve.DriveStraight;
import frc.robot.commands.swerve.OnTheFly;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.climb.ClimbConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.swerve.ToPos;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);
    public static final ButtonBoard buttonBoard = new ButtonBoard();
    //private static final Command sample = new ExampleSubsystemCommand(); it was getting on my nerves seeing the warning
    private static final Command driveStraight = new DriveStraight();
    private static final Command onTheFly = new OnTheFly();
    
    // private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    // private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    // private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    // private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);

    private final static GenericHID buttonBoardPlayer1 = new GenericHID(2);
    private final static GenericHID buttonBoardPlayer2 = new GenericHID(3);

    // coral elevator positions
    private final static JoystickButton buttonl1 = new JoystickButton(buttonBoardPlayer1, 1);
    private final static JoystickButton buttonl2 = new JoystickButton(buttonBoardPlayer1, 2);
    private final static JoystickButton buttonl3 = new JoystickButton(buttonBoardPlayer1, 3);
    private final static JoystickButton buttonl4 = new JoystickButton(buttonBoardPlayer1, 4);

    // source buttons
    private final static JoystickButton buttonRightSource = new JoystickButton(buttonBoardPlayer1, 5);
    private final static JoystickButton buttonLeftSource = new JoystickButton(buttonBoardPlayer1, 6);

    // reef positions
    private final static JoystickButton buttonReefZoneA = new JoystickButton(buttonBoardPlayer2, 7);
    private final static JoystickButton buttonReefZoneB = new JoystickButton(buttonBoardPlayer2, 8);
    private final static JoystickButton buttonReefZoneC = new JoystickButton(buttonBoardPlayer2, 9);
    private final static JoystickButton buttonReefZoneD = new JoystickButton(buttonBoardPlayer2, 10);
    private final static JoystickButton buttonReefZoneE = new JoystickButton(buttonBoardPlayer2, 11);
    private final static JoystickButton buttonReefZoneF = new JoystickButton(buttonBoardPlayer2, 12);
    private final static JoystickButton buttonReefZoneG = new JoystickButton(buttonBoardPlayer2, 13);
    private final static JoystickButton buttonReefZoneH = new JoystickButton(buttonBoardPlayer2, 14);
    private final static JoystickButton buttonReefZoneI = new JoystickButton(buttonBoardPlayer2, 15);
    private final static JoystickButton buttonReefZoneJ = new JoystickButton(buttonBoardPlayer2, 16);
    private final static JoystickButton buttonReefZoneK = new JoystickButton(buttonBoardPlayer2, 17);
    private final static JoystickButton buttonReefZoneL = new JoystickButton(buttonBoardPlayer2, 18);

    // miscellaneous buttons
    private final static JoystickButton buttonAlgaeKnockoff = new JoystickButton(buttonBoardPlayer1, 19);
    private final static JoystickButton buttonUtilityA = new JoystickButton(buttonBoardPlayer1, 20);
    private final static JoystickButton buttonUtilityB = new JoystickButton(buttonBoardPlayer1, 21);
    //now this is just TOO much trolling who put this buttonUtilityA here

    private static final SetElevatorState l1 = new SetElevatorState(ElevatorStates.L1);
    private static final SetElevatorState l2 = new SetElevatorState(ElevatorStates.L2);
    private static final SetElevatorState l3 = new SetElevatorState(ElevatorStates.L3);
    private static final SetElevatorState l4 = new SetElevatorState(ElevatorStates.L4);
    private static final SetElevatorState stow = new SetElevatorState(ElevatorStates.STOW);

    private static final KnockAlgae knockAlgaeLow = new KnockAlgae(ElevatorStates.ALGAE_LOW);
    private static final KnockAlgae knockAlgaeHigh = new KnockAlgae(ElevatorStates.ALGAE_HIGH);
    private static final Handoff handoff = new Handoff();
    private static final IntakeFloor intakeFloor = new IntakeFloor();
    private static final IntakeSource intakeSource = new IntakeSource();
    private static final CoralIntakeSource coralIntakeSource = new CoralIntakeSource();
    private static final OuttakeCoral outtakeCoral = new OuttakeCoral();
    private static final ScoreL1 scoreL1 = new ScoreL1();
    private static final ScoreL234 scoreL4 = new ScoreL234(ElevatorStates.L4);

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbConstants.ArmStates.STOWED,
            ClimbConstants.stowSetPoint_rad);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbConstants.ArmStates.CLIMB,
            ClimbConstants.climbSetPoint_rad);

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

    public static void bindButtonBoard()
    {
        buttonLeftSource.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(0)));
        buttonRightSource.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(1)));
        buttonReefZoneA.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(2)));
        buttonReefZoneB.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(4)));
        buttonReefZoneC.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(7)));
        buttonReefZoneD.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(8)));
        buttonReefZoneE.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(10)));
        buttonReefZoneF.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(12)));
        buttonReefZoneG.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(14)));
        buttonReefZoneH.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(16)));
        buttonReefZoneI.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(18)));
        buttonReefZoneJ.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(20)));
        buttonReefZoneK.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(22)));
        buttonReefZoneL.onTrue(Commands.runOnce(() -> Robot.swerve.startOnTheFly(24)));
        buttonl1.onTrue(l1);
        buttonl2.onTrue(l2);
        buttonl3.onTrue(l3);
        buttonl4.onTrue(l4);
        buttonAlgaeKnockoff.onTrue(Commands.runOnce(() -> buttonBoard.setScoringLocation(ScoringLocation.ALGAE)));

    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        pilot.a().whileTrue(driveStraight);

        pilot.x().onTrue(Commands.runOnce(()->Robot.swerve.setIsOTF(true)));

        pilot.leftBumper().onTrue(Commands.runOnce(()-> {ToPos.setSetpointByClosestReefBranch(true);Robot.swerve.setIsOTF(true);}));
        pilot.rightBumper().onTrue(Commands.runOnce(()-> {ToPos.setSetpointByClosestReefBranch(false);Robot.swerve.setIsOTF(true);}));

        pilot.b().onTrue(Commands.runOnce(() -> {
            Robot.swerve.setIsOTF(false);
            Robot.swerve.cyclePPSetpoint();
            Robot.swerve.showSetpointEndGoal();
        }));



        bindButtonBoard();
        ToPosTriggers.createOTFTriggers();

        // new Trigger(() -> Robot.swerve.isOTF).and(() -> UtilityFunctions.withinMargin(0.5,
        //         Robot.swerve.getPose().getTranslation(), Robot.swerve.getPPSetpoint().setpoint.getTranslation())
        //         ).onTrue(Robot.swerve.getSetpointReachedCommand());


        // new Trigger(() -> Robot.swerve.isOTF).and(() -> UtilityFunctions.withinMargin(0.5,
        //         Robot.swerve.getPose().getTranslation(), Robot.swerve.getPPSetpoint().setpoint.getTranslation())).onTrue(Commands.print("SCORE"));

        // Example binding

        operator.a().onTrue(l1);
        operator.b().onTrue(intakeSource);
        operator.x().onTrue(climb);
        operator.y().onTrue(climbStow);

        pilot.povLeft().onTrue(Commands.runOnce(() -> buttonBoard.setScoringLocation(ScoringLocation.L1)));
        pilot.povUp().onTrue(Commands.runOnce(() -> buttonBoard.setScoringLocation(ScoringLocation.L2)));
        pilot.povRight().onTrue(Commands.runOnce(() -> buttonBoard.setScoringLocation(ScoringLocation.L3)));
        pilot.povDown().onTrue(Commands.runOnce(() -> buttonBoard.setScoringLocation(ScoringLocation.L4)));
    }


    public static void pilotBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        // Example binding
        pilot.a().whileTrue(new ExampleSubsystemCommand());
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
                                () -> pilot.getRightX()
                                ));
    }

}
