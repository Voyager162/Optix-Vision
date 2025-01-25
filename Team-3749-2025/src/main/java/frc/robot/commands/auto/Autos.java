package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * Class containing our auto routines. Referenced by the auto selector and
 * potentially robot container
 * 
 * @author Noah Simon
 */
public class Autos {

    /***
     * 
     * @return The command current selected by the auto chooser
     */
    public static Command getSelectedCommand() {
        return AutoUtils.getChooser().selectedCommand();
    }

    /***
     * A do nothing auto
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return Print Command
     */
    public static Command getPrint() {
        return Commands.print("Print Auto!");
    }

    /***
     * An example, single route auto path
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return "My Routine" Command
     */
    public static Command getMyRoutine() {
        // instaniate our auto loop and trajectories
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("my routine");
        AutoTrajectory trajectory = routine.trajectory("trajectoryName");

        // create our trajectory commands, setting odometry and resetting logging when
        // finished
        Command trajectoryCommand = trajectory.cmd();

        // set the command to begin when the loop enables
        routine.active().onTrue(trajectoryCommand);

        // our final, total command
        Command cmd;

        // adding things together
        cmd = Commands.waitSeconds(1).andThen(Commands.print("Print then Trajectory!"));
        cmd = cmd.andThen(routine.cmd());
        return cmd;
    }

    /***
     * An example muli-route auto path.
     * Based on the results of a boolean supplier, a different 2nd half will occur
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return "Split Routine" Command
     */
    public static Command getSplitRoutine() {

        // becomes AutoRoutine
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("split routine");
        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("split1");
        AutoTrajectory trajectory2a = routine.trajectory("split2a");
        AutoTrajectory trajectory2b = routine.trajectory("split2b");

        Command trajectoy1Command = trajectory1.cmd();
        // active
        routine.active().onTrue(trajectoy1Command);

        trajectory1.done().and(() -> true).onTrue(trajectory2a.cmd());
        trajectory1.done().and(() -> false).onTrue(trajectory2b.cmd());
        return Commands.print("split trajectory auto!").andThen(routine.cmd());

    }

    public static Command get3Piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3-Piece");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");

        Command score1 = AutoUtils.addScoreL4(trajectory1);
        Command intake1 = AutoUtils.addIntake(trajectory2);
        Command score2 = AutoUtils.addScoreL4(trajectory3);
        Command intake2 = AutoUtils.addIntake(trajectory4);
        AutoUtils.addScoreL4(trajectory5); // third score is the end of the routine, so no need for reference

        // reverse order here (connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterIntake(trajectory4, trajectory5, intake2);
        AutoUtils.goNextAfterScored(trajectory3, trajectory4, score2);
        AutoUtils.goNextAfterIntake(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterScored(trajectory1, trajectory2, score1);

        return Commands.print("3 piece auto!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }

    public static Command getStartToL5() {
        return AutoUtils.getSingleTrajectory("Start-L5");
    }

    public static Command getStartToL4() {
        return AutoUtils.getSingleTrajectory("Start-L4");

    }

    public static Command getL5ToStation() {
        return AutoUtils.getSingleTrajectory("L5-Station");
    }

    public static Command getL4ToStation() {
        return AutoUtils.getSingleTrajectory("L4-Station");
    }

    public static Command getL3ToStation() {
        return AutoUtils.getSingleTrajectory("L3-Station");
    }

    public static Command getL2ToStation() {
        return AutoUtils.getSingleTrajectory("L2-Station");
    }

    public static Command getL1ToStation() {
        return AutoUtils.getSingleTrajectory("L1-Station");
    }

    public static Command getMidstartToL6() {
        return AutoUtils.getSingleTrajectory("Midstart-L6");
    }

    public static Command getStartToTeamTaxi() {
        return AutoUtils.getSingleTrajectory("Start-TeamTaxi");
    }

    public static Command getStationToL1() {
        return AutoUtils.getSingleTrajectory("Station-L1");
    }

    public static Command getStationToL2() {
        return AutoUtils.getSingleTrajectory("Station-L2");
    }

    public static Command getStationToL3() {
        return AutoUtils.getSingleTrajectory("Station-L3");
    }

    public static Command getStationToL4() {
        return AutoUtils.getSingleTrajectory("Station-L4");
    }

    public static Command getTeamtaxiToL5() {
        return AutoUtils.getSingleTrajectory("TeamTaxi-L5");
    }

}