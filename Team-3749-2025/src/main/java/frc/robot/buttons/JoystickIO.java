package frc.robot.buttons;

import org.opencv.core.Point;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Robot;
import frc.robot.buttons.ButtonBoard.ScoringMode;
import frc.robot.commands.arm.SetClimbArmState;
import frc.robot.commands.arm.SetCoralArmState;
import frc.robot.commands.elevator.OTFElevatorPreflight;
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
import frc.robot.commands.integration.Climb;
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
    private static final Command onTheFly = new OnTheFly();

    private final static GenericHID buttonBoardPlayer1 = new GenericHID(2);
    private final static GenericHID buttonBoardPlayer2 = new GenericHID(3);
    private final static GenericHID buttonBoardPlayer3 = new GenericHID(4);

    // Source buttons & their IDs
    private final static JoystickButton buttonL1 = new JoystickButton(buttonBoardPlayer1, 6); // P1-6
    private final static JoystickButton buttonL2 = new JoystickButton(buttonBoardPlayer1, 5); // P1-5
    private final static JoystickButton buttonL3 = new JoystickButton(buttonBoardPlayer1, 4); // P1-4
    private final static JoystickButton buttonL4 = new JoystickButton(buttonBoardPlayer1, 3); // P1-3
    private final static JoystickButton buttonLeftSource = new JoystickButton(buttonBoardPlayer1, 1); // P1-1
    private final static JoystickButton buttonRightSource = new JoystickButton(buttonBoardPlayer1, 2); // P1-2

    // Reef positions & their IDs
    private final static JoystickButton reefZoneA = new JoystickButton(buttonBoardPlayer2, 1); // P2-1
    private final static JoystickButton reefZoneB = new JoystickButton(buttonBoardPlayer3, 4); // P3-4
    private final static JoystickButton reefZoneC = new JoystickButton(buttonBoardPlayer3, 3); // P3-3
    private final static JoystickButton reefZoneD = new JoystickButton(buttonBoardPlayer3, 2); // P3-2
    private final static JoystickButton reefZoneE = new JoystickButton(buttonBoardPlayer3, 1); // P3-1
    private final static JoystickButton reefZoneF = new JoystickButton(buttonBoardPlayer2, 8); // P2-8
    private final static JoystickButton reefZoneG = new JoystickButton(buttonBoardPlayer2, 7); // P2-7
    private final static JoystickButton reefZoneH = new JoystickButton(buttonBoardPlayer2, 6); // P2-6
    private final static JoystickButton reefZoneI = new JoystickButton(buttonBoardPlayer2, 5); // P2-5
    private final static JoystickButton reefZoneJ = new JoystickButton(buttonBoardPlayer2, 4); // P2-4
    private final static JoystickButton reefZoneK = new JoystickButton(buttonBoardPlayer2, 3); // P2-3
    private final static JoystickButton reefZoneL = new JoystickButton(buttonBoardPlayer1, 7); // P1-7

    // Util buttons & their IDs
    private final static JoystickButton UtilityButtonC = new JoystickButton(buttonBoardPlayer3, 5); // P3-5
    private final static JoystickButton UtilityButtonB = new JoystickButton(buttonBoardPlayer3, 6); // P3-6
    private final static JoystickButton UtilityButtonA = new JoystickButton(buttonBoardPlayer3, 7); // P3-7
    private final static JoystickButton AlgaeKnockButton = new JoystickButton(buttonBoardPlayer3, 8); // P3-8

    private static final Command climbStow = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.STOWED);
    private static final Command climb = new SetClimbArmState(Robot.climbArm, ClimbArmConstants.ArmStates.CLIMB);
    private static final Command coralHandOff = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.HAND_OFF);
    private static final Command coralPickUp = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.CORAL_PICKUP);
    private static final Command coralL1 = new SetCoralArmState(Robot.coralArm,
            CoralArmConstants.ArmStates.L1);

    private static final SetElevatorState stow = new SetElevatorState(ElevatorStates.STOW);
    private static final SetElevatorState el1 = new SetElevatorState(ElevatorStates.L1);
    private static final SetElevatorState el2 = new SetElevatorState(ElevatorStates.L2);
    private static final SetElevatorState el3 = new SetElevatorState(ElevatorStates.L3);
    private static final SetElevatorState el4 = new SetElevatorState(ElevatorStates.L4);

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

    private static final Command l1 = Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L1));
    private static final Command l2 = Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L2));
    private static final Command l3 = Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L3));
    private static final Command l4 = Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.L4));

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
        buttonLeftSource
                .onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(0))));
        buttonRightSource
                .onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(1))));
        reefZoneA.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(2))));
        reefZoneB.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(4))));
        reefZoneC.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(7))));
        reefZoneD.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(8))));
        reefZoneE.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(10))));
        reefZoneF.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(12))));
        reefZoneG.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(14))));
        reefZoneH.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(16))));
        reefZoneI.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(18))));
        reefZoneJ.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(20))));
        reefZoneK.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(22))));
        reefZoneL.onTrue(new OTFElevatorPreflight().andThen(Commands.runOnce(() -> Robot.swerve.startOnTheFly(24))));
        buttonL1.onTrue(l1);
        buttonL2.onTrue(l2);
        buttonL3.onTrue(l3);
        buttonL4.onTrue(l4);
        AlgaeKnockButton.onTrue(Commands.runOnce(() -> buttonBoard.setScoringMode(ScoringMode.ALGAE)));

    }

    /**
     * If both controllers are plugged in (pi and op)
     */
    public static void pilotAndOperatorBindings() {
        // gyro reset
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));
        pilot.leftBumper().onTrue(Commands.runOnce(() -> {
            ToPos.setSetpointByClosestReefBranch(true);
            Robot.swerve.setIsOTF(true);
        }));
        pilot.rightBumper().onTrue(Commands.runOnce(() -> {
            ToPos.setSetpointByClosestReefBranch(false);
            Robot.swerve.setIsOTF(true);
        }));
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
        // pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

    }

    public static void testBindings() {
        pilot.start().onTrue(Commands.runOnce(() -> Robot.swerve.resetGyro()));

        operator.a().onTrue(new Climb());
        operator.b().whileTrue(Commands.runOnce(() -> Robot.climbArm.setVoltage(6)));
        // // Checking voltage for all subsystems
        // operator.a().onTrue(Commands.run(() -> Robot.elevator.setVoltage(1.5),
        // Robot.elevator))
        // .whileFalse(Commands.run(() -> Robot.elevator.setVoltage(0),
        // Robot.elevator));
        // operator.x().onTrue(Commands.run(() -> Robot.elevator.setVoltage(3),
        // Robot.elevator))
        // .whileFalse(Commands.run(() -> Robot.elevator.setVoltage(0),
        // Robot.elevator));
        // operator.b().onTrue(Commands.run(() -> Robot.elevator.setVoltage(10),
        // Robot.elevator))
        // .whileFalse(Commands.run(() -> Robot.elevator.setVoltage(0),
        // Robot.elevator));
        // operator.y().onTrue(Commands.run(() -> Robot.elevator.setVoltage(9),
        // Robot.elevator))
        // .whileFalse(Commands.run(() -> Robot.elevator.setVoltage(0),
        // Robot.elevator));
        // operator.leftBumper().onTrue(Commands.run(() ->
        // Robot.elevator.setVoltage(12), Robot.elevator))
        // .whileFalse(Commands.run(() -> Robot.elevator.setVoltage(0),
        // Robot.elevator));
        // operator.rightBumper().onTrue(Commands.run(() ->
        // Robot.elevator.setVoltage(-3)))
        // .onFalse(Commands.run(() -> Robot.elevator.setVoltage(0), Robot.elevator));

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
        // operator.povUp()
        //         .onTrue(Commands.run(() -> Robot.coralRoller.setVoltage(3),
        //                 Robot.coralRoller));
        // operator.povUp().whileFalse(Commands.runOnce(() -> Robot.coralRoller.stop(), Robot.coralRoller));
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
        // operator.a().onTrue(el1);
        // operator.y().onTrue(el2);
        // operator.x().onTrue(el3);
        // operator.rightBumper().onTrue(el4);
        // operator.leftBumper().onTrue(stow);
        // // operator.x().onTrue(l3);
        // operator.y().onTrue(l4);

        // Climb + Coral Arms
        // operator.a().onTrue(climbStow);
        // operator.b().onTrue(climb);

        // Run
        // operator.a().whileTrue(algaeRun);
        // operator.b().onTrue(new IntakeFloor());
        // operator.x().onTrue(handoff);
        // operator.y().onTrue(scoreL4);

        // operator.x().onTrue(coralHandOff);
        // operator.b().onTrue(coralRunIntake);
        // operator.a().onTrue(coralRunOuttake);
        // operator.a().onTrue(coralL1);
        // operator.y().onTrue(coralPickUp);

        // operator.leftBumper().whileTrue(scoringRun);

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
