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

    /***
     * A routine that drives straight
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return "Straight" Command
     */
    public static Command getStraight(){
        return AutoUtils.getSingleTrajectory("Straight");

    }

    public static Command getChairGame() {
       return AutoUtils.getSingleTrajectory("Chair Game");

    }
    public static Command getScore_Pick() {
        return AutoUtils.getSingleTrajectory("Score and Pick Note");
 
     }

    public static Command get1Piece() {
        return AutoUtils.getSingleTrajectory("1-r3");
    }

    public static Command getPushLeftAndTaxi() {
        return AutoUtils.getSingleTrajectory("push-left-and-taxi");
    }

    public static Command getPushRightAndTaxi() {
        return AutoUtils.getSingleTrajectory("push-right-and-taxi");
    }

    public static Command get3Piece() {
        return AutoUtils.getSingleTrajectory("3-piece-new");
    }

    public static Command getTeamTaxi() {
        return AutoUtils.getSingleTrajectory("Team Taxi");
    }
    public static Command getStartL5(){
        return AutoUtils.getSingleTrajectory("Start-L5");
    }
    public static Command getStartL4(){
        return AutoUtils.getSingleTrajectory("Start-L4");
    }
    public static Command getL5Station(){
        return AutoUtils.getSingleTrajectory("L5-Station");
    }
    public static Command getStationL4(){
        return AutoUtils.getSingleTrajectory("Station-L4");
    }
    public static Command getL4Station(){
        return AutoUtils.getSingleTrajectory("L4-Station");
    }
    public static Command getStationL3(){
        return AutoUtils.getSingleTrajectory("Station-L3");
    }
    public static Command getL3Station(){
        return AutoUtils.getSingleTrajectory("L3-Station");
    }
    public static Command getStationL2(){
        return AutoUtils.getSingleTrajectory("Station-L2");
    }
    public static Command getL2Station(){
        return AutoUtils.getSingleTrajectory("L2-Station");
    }
    public static Command getL1Station(){
        return AutoUtils.getSingleTrajectory("L1-Station");
    }   
    public static Command getStationL1(){
        return AutoUtils.getSingleTrajectory("Station-L1");
    }
    public static Command getStartTeamTaxi(){
        return AutoUtils.getSingleTrajectory("Start-TeamTaxi");
    }
    public static Command getTeamTaxiL5(){
        return AutoUtils.getSingleTrajectory("TeamTaxi-L5");
    }
    public static Command getCenterL6(){
        return AutoUtils.getSingleTrajectory("MidStart-L6");
    }
    

}