package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.commands.example.ExampleSubsystemCommand;
import frc.robot.commands.swerve.DriveStraight;
import frc.robot.commands.swerve.OnTheFly;
import frc.robot.commands.swerve.SwerveDefaultCommand;
import frc.robot.subsystems.swerve.ToPosConstants;
import frc.robot.subsystems.swerve.ToPosConstants.Setpoints.PPSetpoints;
import frc.robot.utils.ButtonBoard.ScoringLocation;

/**
 * Util class for button bindings
 * 
 * @author Rohin Sood
 * @author Noah Simon
 */
public class JoystickIO {

    private static final CommandXboxController pilot = new CommandXboxController(0);
    private static final CommandXboxController operator = new CommandXboxController(1);
    // private static final Command sample = new ExampleSubsystemCommand(); it was
    // getting on my nerves seeing the warning
    private static final Command driveStraight = new DriveStraight();
    private static final Command onTheFly = new OnTheFly();
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
        pilot.a().whileTrue(driveStraight);

        pilot.x().onTrue(Commands.runOnce(() -> {
            Robot.swerve.isOTF = true;
        }));

        new Trigger(() -> Robot.swerve.isOTF).whileTrue(onTheFly);

        pilot.b().onTrue(Commands.runOnce(() -> {
            Robot.swerve.isOTF = false;
            Robot.swerve.cyclePPSetpoint();
            Robot.swerve.showSetpointEndGoal();
        }));
        createOTFTriggers();

        // Example binding
        operator.a().whileTrue(new ExampleSubsystemCommand());

    }

    public static void createOTFTriggers() {
        Trigger coralStation = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralStation = Robot.swerve.getPPSetpoint() == PPSetpoints.CORALLEFT
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.CORALRIGHT;
            return withinMargin && isCoralStation;
        });
        coralStation.onTrue(Commands.print("coral station intake"));

        
        Trigger coralReefL1 = new Trigger(() -> Robot.swerve.isOTF).and(() -> {
            Boolean withinMargin = OTFWithinMargin();
            Boolean isCoralReef = Robot.swerve.getPPSetpoint() == PPSetpoints.A
                    || Robot.swerve.getPPSetpoint() == PPSetpoints.B; // put all reef locaitons here w boolean supplier
            Boolean isL1 = buttonBoard.getScoringLocation() == ScoringLocation.L1;
            return withinMargin && isCoralReef && isL1;
        });
        coralStation.onTrue(Commands.print("Score L1"));

    }

    public static boolean OTFWithinMargin() {
        return UtilityFunctions.withinMargin(ToPosConstants.Setpoints.approachPointDistance,
                Robot.swerve.getPose().getTranslation(),
                Robot.swerve.getPPSetpoint().setpoint.getTranslation());
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
                                () -> pilot.getRightX()));
    }

}
