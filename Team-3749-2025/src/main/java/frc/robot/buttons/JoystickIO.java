package frc.robot.buttons;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.arm.SetCoralArmState;
import frc.robot.commands.elevator.SetElevatorState;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.IntakeFloor;
import frc.robot.commands.integration.IntakeSource;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.OuttakeCoral;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.commands.roller.MaintainCommand;
import frc.robot.commands.roller.OuttakeRoller;
import frc.robot.commands.roller.RunRoller;
import frc.robot.commands.swerve.OnTheFly;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.arm.climb.ClimbArmConstants;
import frc.robot.subsystems.arm.coral.CoralArmConstants;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.swerve.ToPos;
import frc.robot.utils.MiscConstants.ControllerConstants;

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
    // private static final Command sample = new ExampleSubsystemCommand(); it was
    // getting on my nerves seeing the warning
    private static final Command onTheFly = new OnTheFly();

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

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.STOWED);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.CLIMB);
    private static final Command coralHandOff = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.HAND_OFF);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.CORAL_PICKUP);
    private static final Command coralL1 = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.L1);

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
    private static final RunRoller algaeRun = new RunRoller(Robot.algaeRoller);
    private static final RunRoller coralRunIntake = new RunRoller(Robot.coralRoller);
    private static final OuttakeRoller coralRunOuttake = new OuttakeRoller(Robot.coralRoller);
    private static final RunRoller scoringRun = new RunRoller(Robot.scoringRoller);

    private static final MaintainCommand algaeMaintain = new MaintainCommand(Robot.algaeRoller);
    private static final MaintainCommand coralMaintain = new MaintainCommand(Robot.coralRoller);
    private static final MaintainCommand scoringMaintain = new MaintainCommand(Robot.scoringRoller);

    public static ButtonBoard buttonBoard = new ButtonBoard();

    public JoystickIO() {
    }

    public static void getButtonBindings() {

        if (Robot.isSimulation()) {
            // will show not connected if on
            pilotAndOperatorBindings();
            // simBindings();
        } else {
            testBindings();

        }

        setDefaultCommands();
    }

    public static void bindButtonBoard() {
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
        buttonAlgaeKnockoff.onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.ALGAE)));
    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        /**
         * otf related bindings:
         * 
         * Testing:
         * x to activate the driving
         * press b to move through all the setpoints
         * left bumper to go to the nearest reef related branch based on driver
         * perspective (left side)
         * right bumper ^^^ but right
         * 
         * left dpad to switch l1, l2, l3, l4 (left up right down)
         * press l1234 first, then press b to select, and then x
         * 
         * On the real bot:
         * it'll all js be connected to the buttonboard
         */
        pilot.x().onTrue(Commands.runOnce(() -> Robot.swerve.setIsOTF(true)));

        pilot.leftBumper().onTrue(Commands.runOnce(() -> {
            ToPos.setSetpointByClosestReefBranch(true);
            Robot.swerve.setIsOTF(true);
        }));
        pilot.rightBumper().onTrue(Commands.runOnce(() -> {
            ToPos.setSetpointByClosestReefBranch(false);
            Robot.swerve.setIsOTF(true);
        }));

        pilot.b().onTrue(Commands.runOnce(() -> {
            Robot.swerve.setIsOTF(false);
            Robot.swerve.cyclePPSetpoint();
            Robot.swerve.showSetpointEndGoal();
        }));

        pilot.y().onTrue(Commands.runOnce(() -> {
            buttonBoard.setScoringMode(ScoringMode.ALGAE);
        })); // y is for testing only for now: so this command will always change

        new Trigger(() -> Robot.swerve.getIsOTF()).onTrue(onTheFly);
        new Trigger(() -> {
            if (Math.abs(pilot.getLeftX()) > ControllerConstants.deadband
                    || Math.abs(pilot.getLeftY()) > ControllerConstants.deadband
                    || Math.abs(pilot.getRightX()) > ControllerConstants.deadband) {
                return true;
            }
            return false;
        }).onTrue(Commands.runOnce(() -> Robot.swerve.setIsOTF(false)));

        bindButtonBoard();
        ToPosTriggers.createOTFTriggers();

        // operator.b().onTrue(Commands.runOnce(() -> Robot.elevator.setVoltage(12)));
        // operator.b().onTrue(intakeSource);
        // operator.x().onTrue(climb);
        // operator.y().onTrue(climbStow);

        pilot.povLeft().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L1)));
        pilot.povUp().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L2)));
        pilot.povRight().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L3)));
        pilot.povDown().onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L4)));
        // pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

    }

    public static void testBindings() {
        // // Checking voltage for all subsystems
        operator.a().onTrue(Commands.run(() -> Robot.elevator.setVoltage(-1.5), Robot.elevator))
                .onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));
        operator.x().onTrue(Commands.run(() -> Robot.elevator.setVoltage(-3), Robot.elevator))
                .onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));
        operator.b().onTrue(Commands.run(() -> Robot.elevator.setVoltage(-6), Robot.elevator))
                .onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));
        operator.y().onTrue(Commands.run(() -> Robot.elevator.setVoltage(-9), Robot.elevator))
                .onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));
        operator.leftBumper().onTrue(Commands.run(() -> Robot.elevator.setVoltage(-12), Robot.elevator))
                .onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));

        operator.rightBumper().onTrue(Commands.run(() -> Robot.elevator.setVoltage(3))).onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));
        // operator.b().onTrue(Commands.run(() ->
        // Robot.coralArm.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.x().onTrue(Commands.run(() ->
        // Robot.climbArm.setVoltage(4))).onFalse(Commands.run(()->Robot.climbArm.setVoltage(0)));
        // operator.y().onTrue(Commands.run(() ->
        // Robot.climbArm.setVoltage(8))).onFalse(Commands.run(()->Robot.climbArm.setVoltage(0)));

        // operator.y().whileTrue(Commands.run(() ->
        // Robot.algaeRoller.setVoltage(Robot.subsystemVoltageSetter.get()))).onFalse(Commands.run(()
        // -> Robot.algaeRoller.setVoltage(0)));
        // operator.a().onTrue(Commands.run(() ->
        // Robot.elevator.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.b().whileTrue(Commands.run(() ->
        // Robot.coralRoller.setVoltage(Robot.subsystemVoltageSetter.get()))).onFalse(Commands.run(()
        // -> Robot.coralRoller.setVoltage(0)));
        // operator.x().onTrue(Commands.run(() ->
        // Robot.climbArm.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.y().whileTrue(Commands.run(() ->
        // Robot.algaeRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.leftBumper().whileTrue(Commands.run(() -P>
        // Robot.coralRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
        // operator.leftBumper().whileFalse(Commands.runOnce(() ->
        // Robot.coralRoller.stop()));
        // operator.rightBumper().onTrue(Commands.run(() ->
        // Robot.scoringRoller.setVoltage(Robot.subsystemVoltageSetter.get())));//.onFalse(Commands.run(()
        // -> Robot.scoringRoller.setVoltage(0)));
        // kanhay's keyboard comment (donot remove need for college apps bc i coded a
        // robot)
        // operator.leftBumper().onTrue(Commands.run(() ->
        // Robot.coralRoller.setVoltage(Robot.subsystemVoltageSetter.get())));//.onFalse(Commands.run(()
        // -> Robot.scoringRoller.setVoltage(0)));

        // pilot.a().onTrue(Commands.run(() -> Robot.swerve.setTurnVoltage(3),
        // Robot.swerve));//.onFalse(Commands.run(() ->
        // Robot.scoringRoller.setVoltage(0)));

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
        // operator.b().onTrue(new IntakeFloor());
        // operator.b().onTrue(coralRunIntake);
        // operator.a().onTrue(coralRunOuttake);
        // operator.a().onTrue(coralL1);
        // operator.x().onTrue(coralHandOff);
        // operator.y().onTrue(coralPickUp);

        // operator.x().whileTrue(scoringRun);

        // Maintain
        // operator.a().whileTrue(algaeMaintain);
        // operator.b().whileTrue(coralMaintain);
        // operator.x().whileTrue(scoringMaintain);

        // operator.leftBumper().onTrue(intakeSource);
        // operator.rightBumper().onTrue(scoreL234);
        // operator.b().onTrue(Commands.runOnce(() ->
        // Robot.scoringRoller.setVoltage(Robot.subsystemVoltageSetter.get())));
    }

    public static void pilotBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        pilot.a().onTrue(new IntakeFloor());
        pilot.b().onTrue(new Handoff());
        pilot.x().whileTrue(scoreL4);
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
                                () -> pilot.getRightX()));
    }

}
