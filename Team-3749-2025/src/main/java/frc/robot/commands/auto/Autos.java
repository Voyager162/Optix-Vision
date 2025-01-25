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
     * @return Print Command
     */
    public static Command getPrint() {
        return Commands.print("Print Auto!");
    }

   
    /**
     * 
     * @return a command that scores at positions 4, 3, and 2 at L4 
     */
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

        // reverse order here (ex. connect 3 to 2, THEN 2 to 1)
        AutoUtils.goNextAfterIntake(trajectory4, trajectory5, intake2);
        AutoUtils.goNextAfterScored(trajectory3, trajectory4, score2);
        AutoUtils.goNextAfterIntake(trajectory2, trajectory3, intake1);
        AutoUtils.goNextAfterScored(trajectory1, trajectory2, score1);

        return Commands.print("3 piece auto!").andThen(
                AutoUtils.startRoutine(routine, "Start-L5", trajectory1));
    }


}