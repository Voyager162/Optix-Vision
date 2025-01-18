package frc.robot.commands.auto;

import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.elevator.Elevator;

/**
 * Class containing our auto routines. Referenced by the auto selector and
 * potentially robot container
 * 
 * @author Noah Simon
 */
public class SysIdCommand {

    /***
     * A do nothing auto
     * 
     * @param factory the AutoFactory from AutoUtils
     * @return Print Command
     */
    public static Command getPrint() {
        return Commands.print("Print Auto!");
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return Elevator.sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return Elevator.sysIdRoutine.dynamic(direction);
    }

}
