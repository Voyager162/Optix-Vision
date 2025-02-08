package frc.robot.subsystems.roller.sim;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.roller.PhotoelectricIO;

public class PhotoelectricSim implements PhotoelectricIO { 
    private double scoreTimer = -1;
    private boolean sensing;
    // private boolean changedSensing;
    /**
     * Should only be used for simulation implementation
     * @param initialState 
     */
    @Override
    public void setInitialState(boolean initialState) {
        sensing = initialState;
    }

    /**
     * Updates the photoelectric data with the current state of the roller.
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
                        scoreTimer = 1000000000;
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
                        if (scoreTimer != -1) {
                            scoreTimer = 1000000000;
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
                            scoreTimer = 1000000000;
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
                            scoreTimer = 1000000000;
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
                            scoreTimer = 1000000000;
                        } 
                    }
                break;
            }
        }

    data.sensing = sensing;
  
    }
}
    



