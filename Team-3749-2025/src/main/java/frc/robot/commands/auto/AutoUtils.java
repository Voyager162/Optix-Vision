package frc.robot.commands.auto;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoTrajectory;
import choreo.auto.AutoRoutine;
import choreo.util.ChoreoAllianceFlipUtil;
import choreo.trajectory.SwerveSample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.utils.UtilityFunctions;

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
    private static ChoreoAllianceFlipUtil.Flipper flipper = ChoreoAllianceFlipUtil.getFlipper();

    /**
     * run all necessary auto setup methods
     */
    public static void initAuto() {
        // seems like this method is setup before the automonous command is clicked which makes it not possible
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
        if (flippedChooser.getSelected()){
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
                        .setOdometry(getXFlippedPose(startingPose)),
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
        chooser.addCmd("My Routine", () -> Autos.getMyRoutine());
        chooser.addCmd("Print", () -> Autos.getPrint());
        chooser.addCmd("Split", () -> Autos.getSplitRoutine());
        chooser.addCmd("Straight", () -> Autos.getStraight());
        chooser.addCmd("Chair", () -> Autos.getChairGame());
        chooser.addCmd("Score/pick note", () -> Autos.getScore_Pick());
        chooser.addCmd("Team Taxi", () -> Autos.getTeamTaxi());
        chooser.addCmd("Push Right and Taxi", () -> Autos.getPushRightAndTaxi());
        chooser.addCmd("Push Left and Taxi", () -> Autos.getPushLeftAndTaxi());
        chooser.addCmd("3 Piece", () -> Autos.get3Piece());
        chooser.addCmd("1 Piece", () -> Autos.get1Piece());
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



    public static void addScoreL4(AutoTrajectory trajectory) {
        
        UtilityFunctions.CheckPose(getFinalPose2d(trajectory), 0.26, Math.toRadians(2))
                .onTrue(Commands.runOnce(() -> {
                    System.out.println("score L4");
                }));

    }
    
    public static void addScoreL3(AutoTrajectory trajectory) {
        UtilityFunctions.CheckPose(getFinalPose2d(trajectory), 0.5, Math.toRadians(10))
                .onTrue(Commands.runOnce(() -> {
                    System.out.println("score L3");
                }));

    }

    public static void addIntake(AutoTrajectory trajectory) {
        trajectory.atPose(getFinalPose2d(trajectory), 6, Math.toRadians(180))
                .onTrue(Commands.print("intake"));

    }

    public static void goNextAfterScored(AutoTrajectory curTrajectory, AutoTrajectory nextTrajectory) {
        // curTrajectory.atPose(getFinalPose2d(curTrajectory), 5, Math.toRadians(180))
        //         .onTrue(nextTrajectory.cmd());
        //         System.out.println("Start Trajectory started");

        curTrajectory.done().and(() -> true)
                .onTrue(nextTrajectory.cmd());
                System.out.println("Start Trajectory started");
    }

    public static void goNextAfterIntake(AutoTrajectory curTrajectory, AutoTrajectory nextTrajectory) {
        // curTrajectory.atPose(getFinalPose2d(curTrajectory), 6, Math.toRadians(180))
        //         .onTrue(nextTrajectory.cmd());
        //         System.out.println("Next Trajectory started");

        curTrajectory.done().and(() -> true)
                .onTrue(nextTrajectory.cmd());
                System.out.println("Start Trajectory started");
    }


    /**
     * Each pos will be flipped across the X-axis using Choreo's Flip Util
     * Returns new pos
     * Used in auto factory for flipped paths
     */
    private static Pose2d getXFlippedPose(Pose2d pos) {
        double newX = pos.getX();
        double newY = flipper.flipY(pos.getY());
        double newHeading = pos.getRotation().getRadians() * -1;

        return new Pose2d(newX, newY,
                new Rotation2d(newHeading));
    }

    public static Pose2d getFinalPose2d(AutoTrajectory trajectory) {
        if (flippedChooser.getSelected()){
            System.out.println("Flipped Pose:"  + getXFlippedPose(trajectory.getFinalPose().get()));
            return getXFlippedPose(trajectory.getFinalPose().get());
        }
        else{
            return trajectory.getFinalPose().get();
        }
    }

}