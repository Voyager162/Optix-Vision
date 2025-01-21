package frc.robot.commands.auto;



import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAllianceFlipUtil;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;


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
    private static SendableChooser<Boolean> flippedChooser;
    public static ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    /**
     * run all necessary auto setup methods
     */
    public static void initAuto() {
        // seems like this method is setup before the automonous command is clicked
        // which makes it not possible
        // for us to choose before hitting the automonous button

        setupFactory();
        setupChooser();
        setupFlipChooser();
    }

    /**
     * 
     * @return the auto selector object
     */
    public static AutoChooser getChooser() {
        return chooser;
    }

    public static AutoFactory getAutoFactory() {
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
        chooser.addCmd("3 Piece", () -> Autos.get3Piece());
        // Default
        chooser.select("Straight");

        SmartDashboard.putData("Auto: Auto Chooser", chooser);

    }

    private static void setupFlipChooser() {
        flippedChooser = new SendableChooser<Boolean>();

        flippedChooser.addOption("Yes", true);
        flippedChooser.addOption("No", false);
        flippedChooser.setDefaultOption("No", false);

        SmartDashboard.putData("Flip Auto Path?", flippedChooser);
    }

    public static Command getSingleTrajectory(String trajectoryName) {
        AutoRoutine routine = getAutoFactory().newRoutine(trajectoryName);
        AutoTrajectory trajectory = routine.trajectory(trajectoryName);

        Command trajectoryCommand = trajectory.cmd();

        routine.active().onTrue(
                getAutoFactory().resetOdometry(trajectoryName).andThen(
                        trajectoryCommand));

        System.out.println(trajectory.getInitialPose().get());
        return Commands.print(trajectoryName).andThen(routine.cmd());

    }

    public static Command startRoutine(AutoRoutine routine, String firstTrajectoryName,
            AutoTrajectory firstTrajectory) {

        routine.active().onTrue(
                AutoUtils.getAutoFactory().resetOdometry(firstTrajectoryName).andThen(
                        firstTrajectory.cmd()));
        return routine.cmd();
    }
    
    //This is scoring on Level 4
    public static void addScoreL4(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red){
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(Commands.print("SCORE L4"));


    }

    //This is scoring on Level 3
    public static void addScoreL3(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red){
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(Commands.print("SCORE L3"));


    }

    public static void addIntake(AutoTrajectory trajectory) {
        Pose2d endingPose2d = getFinalPose2d(trajectory);
        // unflip the alliance so that atPose can flip it; it's a quirk of referencing the trajectory
        if (DriverStation.getAlliance().get() == Alliance.Red){
            endingPose2d = ChoreoAllianceFlipUtil.flip(endingPose2d);
        }

        trajectory.atPose(endingPose2d, 1, 1.57).onTrue(Commands.print("Intaking"));


    }

    public static void goNextAfterScored(AutoTrajectory curTrajectory, AutoTrajectory nextTrajectory) {
        curTrajectory.done().and(() -> true)
                .onTrue(Commands.waitSeconds(0.6).andThen(nextTrajectory.cmd()));
        System.out.println("Next Trajectory started");
    }

    public static void goNextAfterIntake(AutoTrajectory curTrajectory, AutoTrajectory nextTrajectory) {
        curTrajectory.done().and(() -> true)
                .onTrue(Commands.waitSeconds(0.6).andThen(nextTrajectory.cmd()));
        System.out.println("Start Trajectory started");
    }

    /**
     * Each pos will be flipped across the X-axis using Choreo's Flip Util
     * Returns new pos
     * Used in auto factory for flipped paths
     */
    private static Pose2d getFlippedPose(Pose2d pos) {
        Translation2d translation = new Translation2d(pos.getX(), flipper.flipY(pos.getY()));

        Rotation2d rotation = new Rotation2d(Math.PI - pos.getRotation().getRadians())
                .rotateBy(new Rotation2d(Math.PI));

        return new Pose2d(translation, rotation);
    }

    public static Pose2d getFinalPose2d(AutoTrajectory trajectory) {
        if (flippedChooser.getSelected()) {
            System.out.println("Flipped Pose:" + getFlippedPose(trajectory.getFinalPose().get()));

            return getFlippedPose(trajectory.getFinalPose().get());
        } else {
            return trajectory.getFinalPose().get();
        }
    }




}