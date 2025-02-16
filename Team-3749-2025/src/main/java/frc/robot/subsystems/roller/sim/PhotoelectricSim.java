package frc.robot.subsystems.roller.sim;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.roller.PhotoelectricIO;

/**
 * Simulation for Photoelectric sensor behavior in the roller subsystem
 *
 * @author Lilian Wu
 */
public class PhotoelectricSim implements PhotoelectricIO { 
    private double scoreTimer = -1;
    private boolean sensing;

    /**
     * Sets sensing to an initial state before its updated. Based on hasPiece
     * 
     * @param initialState 
     */
    @Override
    public void setInitialState(boolean initialState) {
        sensing = initialState;
    }

    /**
     * Updates the photoelectric data with the current state of the roller
     * 
     * @param data the data to be updated
     */
    @Override
    public void updateData(PhotoelectricData data) {
        if (Robot.scoringRoller.getCurrentCommand() != null) {
            switch(Robot.scoringRoller.getCurrentCommand().getName()) {
                case "ScoreL234": 
                    if (scoreTimer == 1000000000) {
                        scoreTimer = -1;
                    }

                    if (scoreTimer < 0) {  
                        scoreTimer = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = false;
                        scoreTimer = 1000000000; // resets timer for next command
                    }
                break;
                case "IntakeSource": 
                    if (scoreTimer == 1000000000) {
                        scoreTimer = -1;
                    }

                    if (scoreTimer < 0) {  
                        scoreTimer = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = true;
                        // System.out.println("scoreTimer" + scoreTimer);
                        if (scoreTimer != -1) {
                            scoreTimer = 1000000000; // resets timer for next command
                        } 
                    }
                break;
            }
        }
        
        if (Robot.coralRoller.getCurrentCommand() != null) {
            switch(Robot.coralRoller.getCurrentCommand().getName()) {
                case "CoralIntakeSource": 
                    if (scoreTimer == 1000000000) {
                        scoreTimer = -1;
                    }
                    
                    if (scoreTimer < 0) {  
                        scoreTimer = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = true;
                        if (scoreTimer != -1) {
                            scoreTimer = 1000000000; // resets timer for next command
                        } 
                    }
                break;
                case "ScoreL1": 
                    if (scoreTimer == 1000000000) {
                        scoreTimer = -1;
                    }
                    
                    if (scoreTimer < 0) {  
                        scoreTimer = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = false;
                        if (scoreTimer != -1) {
                            scoreTimer = 1000000000; // resets timer for next command
                        } 
                    }
                break;
                case "IntakeFloor": 
                    if (scoreTimer == 1000000000) {
                        scoreTimer = -1;
                    }
                    
                    if (scoreTimer < 0) {  
                        scoreTimer = Timer.getFPGATimestamp();
                    }

                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = true;
                        if (scoreTimer != -1) {
                            scoreTimer = 1000000000; // resets timer for next command
                        } 
                    }
                break;
            }
        }

    data.sensing = sensing;
  
    }
}
    



