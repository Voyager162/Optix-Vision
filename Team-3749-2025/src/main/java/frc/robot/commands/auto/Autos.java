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

    public static Command getTaxi() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Taxi");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");

        AutoUtils.addScoreL4(trajectory1);
 
        return Commands.print("Taxi!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }

    public static Command getTeamtaxi() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Team Taxi");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-TeamTaxi");     

        return Commands.print("Team taxi").andThen(
                AutoUtils.startRoutine(routine, "Start-TeamTaxi", trajectory1));
    }

    public static Command getOnePieceCenter() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("One-Piece-Center");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("MidStart-L6");
      
        AutoUtils.addScoreL4(trajectory1);

        return Commands.print("One Piece Center").andThen(
                AutoUtils.startRoutine(routine, "MidStart-L6", trajectory1));
    }

    public static Command getFullAutoRP() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("Full Auto RP");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-TeamTaxi");
        AutoTrajectory trajectory2 = routine.trajectory("TeamTaxi-L5");
  
      
        AutoUtils.addScoreL4(trajectory2);

        // reverse order here (connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterScored(trajectory1, trajectory2);

        return Commands.print("Full Auto RP!").andThen(
                AutoUtils.startRoutine(routine, "Start-TeamTaxi", trajectory1));
    }

    public static Command get4piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("4-Piece");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");
        AutoTrajectory trajectory6 = routine.trajectory("L3-Station");
        AutoTrajectory trajectory7 = routine.trajectory("Station-L2");
      
        AutoUtils.addScoreL4(trajectory1);
        AutoUtils.addIntake(trajectory2);
        AutoUtils.addScoreL4(trajectory3);
        AutoUtils.addIntake(trajectory4);
        AutoUtils.addScoreL4(trajectory5);
        AutoUtils.addIntake(trajectory6);
        AutoUtils.addScoreL4(trajectory7);

        // reverse order here (connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterIntake(trajectory6, trajectory7);
        AutoUtils.goNextAfterIntake(trajectory5, trajectory6);
        AutoUtils.goNextAfterIntake(trajectory4, trajectory5);
        AutoUtils.goNextAfterIntake(trajectory3, trajectory4);
        AutoUtils.goNextAfterIntake(trajectory2, trajectory3);
        AutoUtils.goNextAfterScored(trajectory1, trajectory2);

        return Commands.print("4 piece auto!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }
    
    public static Command get3Piece() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3-Piece");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");
      
        AutoUtils.addScoreL4(trajectory1);
        AutoUtils.addIntake(trajectory2);
        AutoUtils.addScoreL4(trajectory3);
        AutoUtils.addIntake(trajectory4);
        AutoUtils.addScoreL4(trajectory5);

        // reverse order here (connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterIntake(trajectory4, trajectory5);
        AutoUtils.goNextAfterIntake(trajectory3, trajectory4);
        AutoUtils.goNextAfterIntake(trajectory2, trajectory3);
        AutoUtils.goNextAfterScored(trajectory1, trajectory2);

        return Commands.print("3 piece auto!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }

    public static Command get3PieceandAlgae() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("3-Piece-and-Algae");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L5");
        AutoTrajectory trajectory2 = routine.trajectory("L5-Station");
        AutoTrajectory trajectory3 = routine.trajectory("Station-L4");
        AutoTrajectory trajectory4 = routine.trajectory("L4-Station");
        AutoTrajectory trajectory5 = routine.trajectory("Station-L3");
        AutoTrajectory trajectory6 = routine.trajectory("L3-Algae");
      
        AutoUtils.addScoreL4(trajectory1);
        AutoUtils.addIntake(trajectory2);
        AutoUtils.addScoreL4(trajectory3);
        AutoUtils.addIntake(trajectory4);
        AutoUtils.addScoreL4(trajectory5);
        AutoUtils.addAlgaeIntake(trajectory6);

        // reverse order here (connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterIntake(trajectory5, trajectory6);
        AutoUtils.goNextAfterIntake(trajectory4, trajectory5);
        AutoUtils.goNextAfterIntake(trajectory3, trajectory4);
        AutoUtils.goNextAfterIntake(trajectory2, trajectory3);
        AutoUtils.goNextAfterScored(trajectory1, trajectory2);

        return Commands.print("3 Piece and Algae!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }
    
    public static Command get2AlgaeAndKnock() {
        AutoRoutine routine = AutoUtils.getAutoFactory().newRoutine("2-Algae-and-Knock");

        // loop.trajectory, or the new name
        AutoTrajectory trajectory1 = routine.trajectory("Start-L4");
        AutoTrajectory trajectory2 = routine.trajectory("L4-Algae1");
        AutoTrajectory trajectory3 = routine.trajectory("Algae1 - Processor");
        AutoTrajectory trajectory4 = routine.trajectory("Processor-Algae3");
        AutoTrajectory trajectory5 = routine.trajectory("Algae3- L1");
      
        AutoUtils.addScoreL4(trajectory1);
        AutoUtils.addAlgaeIntake(trajectory2);
        AutoUtils.addScoreAlgae(trajectory3);
        AutoUtils.addAlgaeIntake(trajectory4);
        AutoUtils.addScoreL4(trajectory5);

        // reverse order here (connect 3 to 2, THEN 2 to 1)      
        AutoUtils.goNextAfterIntake(trajectory4, trajectory5);
        AutoUtils.goNextAfterIntake(trajectory3, trajectory4);
        AutoUtils.goNextAfterIntake(trajectory2, trajectory3);
        AutoUtils.goNextAfterScored(trajectory1, trajectory2);

        return Commands.print("2-Algae-and-Knock").andThen(
                AutoUtils.startRoutine(routine, "Start-L4", trajectory1));
    }
    


    

}