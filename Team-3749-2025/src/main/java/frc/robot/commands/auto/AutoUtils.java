package frc.robot.commands.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import choreo.trajectory.SwerveSample;
import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.elevator.ElevatorConstants.ElevatorStates;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.commands.integration.CoralIntakeSource;
import frc.robot.commands.integration.Handoff;
import frc.robot.commands.integration.KnockAlgae;
import frc.robot.commands.integration.ScoreL1;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.commands.integration.ScoringModeConditionalHandoff;

/**
 * All setup and helper methods for auto routines, including the
 * choreo factory and auto selector
 * 
 * @author Noah Simon
 */
public class AutoUtils {
    // make sure to properly log the robot's setpoints

    private static AutoFactory factory;
    private static AutoFactory factoryFlipped;
    private static AutoChooser chooser;
    // private static AutoChooser flipChooser;
    private static SendableChooser<Boolean> flippedChooser = new SendableChooser<Boolean>();
    public static ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    /**
     * run all necessary auto setup methods
     */
    public static void initAuto() {
        // seems like this method is setup before the automonous command is clicked
        // which makes it not possible
        // for us to choose before hitting the automonous button

        setupFactory();
        setupFlipChooser();
        setupChooser();
    }

    /**
     * 
     * @return the auto selector object
     */
    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getAutoFactory() {

        SmartDashboard.putBoolean("autoFactory Flipped", flippedChooser.getSelected());

        if (flippedChooser.getSelected()) {
            return factoryFlipped;
        }
        return factory;

    }

    /**
     * setup the choreo factor object with bindings, controller, etc.
     */
    private static void setupFactory() {
        /**
         * Swerve Pose Supplier
         * Reset Odometry Method
         * Swerve Controller (PID and set chassis speeds)
         * Alliance Supplier for swapping
         * Swerve Subsystem for scheduling
         * Bindings, created above
         */

        // will now take a reset odometry

        factory = new AutoFactory(() -> Robot.swerve.getPose(),
                (Pose2d startingPose) -> Robot.swerve
                        .setOdometry(startingPose),
                (SwerveSample sample) -> Robot.swerve.followSample(sample, false),
                true,
                Robot.swerve);

        factoryFlipped = new AutoFactory(() -> Robot.swerve.getPose(),
                (Pose2d startingPose) -> Robot.swerve
                        .setOdometry(getFlippedPose(startingPose)),
                (SwerveSample sample) -> Robot.swerve.followSample(sample, true),
                true,
                Robot.swerve);
        // Event Binding
        factory.bind("Marker", Commands.print("Marker Passed"));
        factoryFlipped.bind("Marker", Commands.print("Marker Passed"));

    }

    /**
     * setup the choreo auto chooser and assign it to the Shuffleboard/Auto tab of
     * networktables
     */

    private static void setupChooser() {

        // interface for choreo

        // Made sendable, use SmartDashbaord now
        chooser = new AutoChooser();

        chooser.addCmd("3-Piece", () -> Autos.get3Piece());
        chooser.addCmd("3-Piece-flipped", () -> Autos.get3PieceFlipped());
        chooser.addCmd("TeamTaxi", () -> Autos.getTeamtaxi());
        chooser.addCmd("Taxi", () -> Autos.getTaxi());
        chooser.addCmd("One Piece Center", () -> Autos.getOnePieceCenter());
        chooser.addCmd("4-Piece", () -> Autos.get4Piece());
        chooser.addCmd("3 Coral and 2 Algae", () -> Autos.get3CoralAnd2Algae());
        chooser.addCmd("Two Piece Score L1", () -> Autos.getTwoPieceScoreL1());
        chooser.addCmd("Coral 4 Piece", () -> Autos.getCoralArm4piece());
        chooser.addCmd("Coral 3 Piece", () -> Autos.getCoralArm3piece());
        chooser.select("4-Piece");

        SmartDashboard.putData("Auto: Auto Chooser", chooser);

    }

    /**
     * This just sets up AutoChooser in Robot Sim
     */

    private static void setupFlipChooser() {

        // flippedChooser.addOption("Yes", true);
        flippedChooser.addOption("No", false);
        flippedChooser.setDefaultOption("No", false);

        SmartDashboard.putData("Flip Auto Path?", flippedChooser);
    }

    /**
     * A command to run a single choreo trajectory with no events
     * 
     * @param trajectoryName
     * @return
     */
    public static Command getSingleTrajectory(String trajectoryName) {
        AutoRoutine routine = getAutoFactory().newRoutine(trajectoryName);
        AutoTrajectory trajectory = routine.trajectory(trajectoryName);

        Command trajectoryCommand = trajectory.cmd();

        routine.active().onTrue(
                getAutoFactory().resetOdometry(trajectoryName).andThen(
                        trajectoryCommand));

        return Commands.print(trajectoryName).andThen(routine.cmd());

    }

    /**
     * Returns the command for a routine that will start reset odometry, and then
     * run the first trajectory and all thing that follow from triggers
     * 
     * @param routine
     * @param firstTrajectoryName
     * @param firstTrajectory
     * @return
     */
    public static Command startRoutine(AutoRoutine routine, String firstTrajectoryName,
            AutoTrajectory firstTrajectory) {

        routine.active()
                .onTrue(
                        firstTrajectory.cmd());
        return routine.cmd();
    }

    public static void addHandoffOnIntake() {
        new Trigger(() -> Robot.coralRoller.hasPiece()).debounce(RollerConstants.Coral.intakeSourceWaitTime)
                .onTrue(new ScoringModeConditionalHandoff());
    }

    /**
     * Returns a command to score at L4 when the robot is approaching the end of the
     * given trajectory
     * 
     * @param trajectory
     * @return
     */
    public static Command addScoreL4(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }
        // Command intakeSource = new IntakeSource();
        Command scoreL4 = new ScoreL234(ElevatorStates.L4);

        trajectory.atPose(endingPose2d, 2.25, 2 * Math.PI).onTrue(scoreL4);

        trajectory.done()
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));
        return scoreL4;
    }

    /**
     * Returns a command to score at L3 when the robot is approaching the end of the
     * given trajectory
     * 
     * @param trajectory
     * @return
     */
    public static Command addScoreL3(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }
        Command scoreL3 = new ScoreL234(ElevatorStates.L3);

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(scoreL3);
        trajectory.done().and(() -> scoreL3.isScheduled())
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));
        return scoreL3;

    }

    /**
     * Returns a command to score at L3 when the robot is approaching the end of the
     * given trajectory
     * 
     * @param trajectory
     * @return
     */
    public static Command addScoreL2(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }
        Command scoreL2 = new ScoreL234(ElevatorStates.L2);

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(scoreL2);
        trajectory.done().and(() -> scoreL2.isScheduled())
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));
        return scoreL2;

    }

    /**
     * Returns a command to score at L1 when the robot is approaching the end of the
     * given trajectory
     * 
     * @param trajectory
     * @return
     */
    public static Command addScoreL1(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }
        Command scoreL1 = new ScoreL1();

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(scoreL1);
        trajectory.done().and(() -> scoreL1.isScheduled())
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));
        return scoreL1;
    }

    /**
     * A command to intake from station when the robot is approaching the end of the
     * given trajectory
     * 
     * @param trajectory
     * @return
     */
    public static Command addIntake(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }
        Command intake = new CoralIntakeSource();
        Command handoff = new ScoringModeConditionalHandoff();

        trajectory.atPose(endingPose2d, 1.5, 2 * Math.PI).onTrue(intake);
        trajectory.done()
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));

        new Trigger(() -> intake.isFinished()).onTrue(new Handoff());
        return handoff;

    }

    public static Command addKnockAlgae(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }

        Command knockAlgae = new KnockAlgae(ElevatorStates.L4);

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(knockAlgae);
        trajectory.done().and(() -> knockAlgae.isScheduled())
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));
        return knockAlgae;

    }

    /**
     * A command to intake from station using coral arm when the robot is
     * approaching the end of the
     * given trajectory
     * 
     * @param trajectory
     * @return
     */
    public static Command addGroundIntake(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing
        // the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red) {
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }
        Command coralintake = new CoralIntakeSource();
        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(coralintake);

        trajectory.done()
                .onTrue(
                        Commands.run(() -> {
                            Robot.swerve.followSample(trajectory.getFinalPose().get(), new Pose2d());
                        }, Robot.swerve));
        return coralintake;

    }

    /**
     * This will begin "nextTrajectory" following the completion of "curTrajectory"
     * and scoring. This should be used to link trajectories together, but only
     * moving on to the next step in the path if the scoring aciton has been
     * properly completed
     * 
     * @param curTrajectory
     * @param nextTrajectory
     * @param ScoreL234
     */

    /**
     * This will begin "nextTrajectory" following the completion of "curTrajectory"
     * and a command. This should be used to link trajectories together, but only
     * moving on to the next step in the path if the aciton has been
     * properly completed
     * 
     * @param curTrajectory
     * @param nextTrajectory
     * @param Command
     */
    public static void goNextAfterCommand(AutoTrajectory curTrajectory, AutoTrajectory nextTrajectory,
            Command command) {
        // continue to move with PID to the final position until the command is done
        // new Trigger(() -> command.isFinished() &&
        // Robot.swerve.atSwerveSetpoint(curTrajectory.getFinalPose().get())).onTrue(nextTrajectory.cmd());

            curTrajectory.doneFor(6).and(() -> command.isFinished()).onTrue(nextTrajectory.cmd());

        
    }

    /**
     * This will flip a given pose across the x-axis of the field
     * 
     * @param pos the position to be flipped
     * @return the flipped position
     */
    private static Pose2d getFlippedPose(Pose2d pos) {
        Translation2d translation = new Translation2d(pos.getX(), flipper.flipY(pos.getY()));

        Rotation2d rotation = new Rotation2d(Math.PI - pos.getRotation().getRadians())
                .rotateBy(new Rotation2d(Math.PI));

        return new Pose2d(translation, rotation);
    }

    /**
     * returns the final position of the trajectory given, vertically flipped
     * according to the flipped chooser
     * 
     * @param trajectory
     * @return
     */
    public static Pose2d getFinalPose2d(AutoTrajectory trajectory) {
        // if (flippedChooser.getSelected()) {
        // System.out.println("Flipped Pose:" +
        // getFlippedPose(trajectory.getFinalPose().get()));

        // return getFlippedPose(trajectory.getFinalPose().get());
        // } else {
        return trajectory.getFinalPose().isPresent() ? trajectory.getFinalPose().get() : new Pose2d();
        // }
    }

}