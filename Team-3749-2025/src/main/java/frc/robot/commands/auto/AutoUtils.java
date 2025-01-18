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
        chooser.addCmd("Start to L5", () -> Autos.getStartL5());
        chooser.addCmd("Start to L4", () -> Autos.getStartL4());
        chooser.addCmd("Station to L4", () -> Autos.getStationL4());
        chooser.addCmd("Station to L3", () -> Autos.getStationL3());
        chooser.addCmd("Station to L2", () -> Autos.getStationL2());
        chooser.addCmd("Station to L1", () -> Autos.getStationL1());
        chooser.addCmd("L2 to Station", () -> Autos.getL2Station());
        chooser.addCmd("L3 to Station", () -> Autos.getL3Station());
        chooser.addCmd("L4 to Station", () -> Autos.getL4Station());
        chooser.addCmd("L5 to Station", () -> Autos.getL5Station());
        chooser.addCmd("L1 to Station", () -> Autos.getL1Station());
        chooser.addCmd("Start to Team Taxi", () -> Autos.getStartTeamTaxi());
        chooser.addCmd("Team Taxi to L5", () -> Autos.getTeamTaxiL5());
        chooser.addCmd("MidStart to L6", () -> Autos.getCenterL6());

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

}