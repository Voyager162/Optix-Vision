package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Robot;
import frc.robot.subsystems.swerve.Swerve;
public class ToPos {

    public void getPos(){
        Robot.swerve.getPose();
    }
}
/** Main Goal
 *     What is a way to get to another setpoint without odometry,or more specifically rather than caring about 
 *     where robot is with odometry adding asd subtracting positions regardless to reach a setpoint 
 * 
 * 
 * Step one done by 1/7/2025
 *      Reaserch. What are we doing and how do we do it. 
 * Step two  
 *      whatever we do, high chances that there will be a initial state and end state. Make a method to geneorate trajecotry to end state from initial using a library. 
 * Step three 
 *       Already did this but get pos from bot and then use that as initial. 
 * Step four.
 *      Add constants as get end positions
 *      
**/