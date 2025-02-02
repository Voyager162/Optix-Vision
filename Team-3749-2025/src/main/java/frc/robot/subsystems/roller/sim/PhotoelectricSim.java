package frc.robot.subsystems.roller.sim;

import javax.sound.sampled.SourceDataLine;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.subsystems.roller.PhotoelectricIO;

public class PhotoelectricSim implements PhotoelectricIO { 
    private double scoreTimer = -1;
    private boolean sensing;
    /**
     * Should only be used for simulation implementation
     * @param initialState 
     */
    @Override
    public void setInitialState(boolean initialState) {
        if (initialState) {
            sensing = true;
            System.out.println("Running InitalState");
        } else {
            sensing = false;
            System.out.println("NOT Running InitalState");
        }
    }

    @Override
    public void updateData(PhotoelectricData data) {
        if (Robot.scoringRoller == null || Robot.scoringRoller.getCurrentCommand() == null) {
            System.out.println("getCurrentCommand is null");
            return;
        }

        String currentCommandName = Robot.scoringRoller.getCurrentCommand().getName();
        System.out.println("getCurrentCommand is not null: " + currentCommandName);

        if (scoreTimer == 999999999) { // Initialize timer only once per command
            System.out.println("get the scoreTimer AGAIN");
            scoreTimer = -1;
        }
        
        if (scoreTimer < 0) {  // Initialize timer only once per command
            System.out.println("get the scoreTimer");
            scoreTimer = Timer.getFPGATimestamp();
        }
        
        switch (currentCommandName) {
            case "Handoff": 
            case "IntakeFloor": 
            case "IntakeSource":
            case "CoralIntakeSource":
                System.out.println("sensing before score timer" + sensing);
                // sensing = false;
                if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                    sensing = true;
                    System.out.println("scoreTimer" + scoreTimer);
                    if (scoreTimer != -1) {
                        scoreTimer = 999999999;
                        System.out.println("sensing set after score timer: " + sensing);
                    } 
                }
                break;

            case "ScoreL1": 
            case "OuttakeCoral":
                    // sensing = true; 
                    if (Timer.getFPGATimestamp() - scoreTimer > 2) {
                        sensing = false;
                        if (scoreTimer != -1) {
                            scoreTimer = 999999999; 
                        }
                    }
                break;
       case "ScoreL234": 
           System.out.println("THE CASE IS CALLED");
           if (scoreTimer < 0) {  // Initialize timer only once per command
               System.out.println("get the scoreTimer");
               scoreTimer = Timer.getFPGATimestamp();
               
               sensing = true; // Set sensing to true immediately
               Robot.scoringRoller.getCurrentCommand().andThen(new WaitCommand(2)).andThen(new InstantCommand(() -> sensing = false)).schedule(); // Wait for 2 seconds and then set sensing to false
           }
           break;
        
        }
        data.sensing = sensing;
    }
}


