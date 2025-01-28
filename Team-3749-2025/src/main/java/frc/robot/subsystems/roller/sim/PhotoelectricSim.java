package frc.robot.subsystems.roller.sim;

import java.time.Duration;

import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.commands.integration.ScoreL234;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.RollerConstants;
import frc.robot.subsystems.roller.implementations.ScoringRoller;

public class PhotoelectricSim implements PhotoelectricIO { 
    private double scoreTimer = -1;
    private boolean sensing;

    // @Override
    // public void resetTimer() {
    //     scoreTimer = -1;
    //     System.out.println("DEBUG: reset the score timer");
    // }

    @Override
    public void setSensing(String commandName) {
        

        if (scoreTimer < 0) {  // Initialize timer only once per command
            scoreTimer = Timer.getFPGATimestamp();
            System.out.println("DEBUG : init score timer");
        }

        if (scoreTimer == 999999999) { // Initialize timer only once per command
            scoreTimer = -1;
            System.out.println("DEBUG : set score timer to " + scoreTimer);
        }

        switch(commandName) {
            case "Handoff": 
            case "IntakeFloor": 
            case "IntakeSource":
            case "CoralIntakeSource":
                sensing = false;
                // System.out.println("here intake");
                if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                    sensing = true;
                    if (scoreTimer != -1) {
                        scoreTimer = 999999999; // scoreTimer = -1; // Reset timer when sensing turns false
                    } 
                    System.out.println("DEBUG : intake done after 2 second, scoreTimer == " + scoreTimer);
                }
                break;

            case "OuttakeCoral":
                    sensing = true; 
                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = false;
                        // System.out.println("here outtake after timer");
                        if (scoreTimer != -1) {
                            scoreTimer = 999999999; // scoreTimer = -1; // Reset timer when sensing turns false
                        }
                        System.out.println("DEBUG : outtake done after 2 second, scoreTimer == " + scoreTimer);
                    }
                break;

            case "ScoreL1": 
                break;
            case "ScoreL234": 
                System.out.println("sensing before: " + sensing);
                sensing = !sensing;
                System.out.println("sensing after: " + sensing);
                break;

             // A new command to pre-set the status of "sensing"   
            case "SensorSwitch":
                sensing = !sensing;
                System.out.println("sensing: " + sensing);
                break;
                
    }
        // if (scoreTimer < 0) {
        //     scoreTimer = Timer.getFPGATimestamp();
        // }
        // if (Timer.getFPGATimestamp() - scoreTimer > 5){
        //     this.sensing = false;
        // }
        // if (!this.sensing) {
        //     Robot.scoringRoller.setHasPiece(false);
        // }
    
    }

    @Override
    public void updateData(PhotoelectricData data) {
        data.sensing = sensing;
    }

    
}



