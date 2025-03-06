package frc.robot.buttons;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoard {

    // IPAC players
    public GenericHID buttonBoardPlayer1 = new GenericHID(2);
    public GenericHID buttonBoardPlayer2 = new GenericHID(3);
    public GenericHID buttonBoardPlayer3 = new GenericHID(4);

    // coral elevator positions
    public JoystickButton buttonl1 = new JoystickButton(buttonBoardPlayer1, 6);
    public JoystickButton buttonl2 = new JoystickButton(buttonBoardPlayer1, 5);
    public JoystickButton buttonl3 = new JoystickButton(buttonBoardPlayer1,4);
    public JoystickButton buttonl4 = new JoystickButton(buttonBoardPlayer1, 3);

    // source buttons
    public JoystickButton buttonRightSource = new JoystickButton(buttonBoardPlayer1, 2);
    public JoystickButton buttonLeftSource = new JoystickButton(buttonBoardPlayer1, 1);

    // reef positions
    public JoystickButton buttonReefZoneLeft1 = new JoystickButton(buttonBoardPlayer2, 1);
    public JoystickButton buttonReefZoneLeft2 = new JoystickButton(buttonBoardPlayer1, 7);
    public JoystickButton buttonReefZoneLeft3 = new JoystickButton(buttonBoardPlayer2, 3);
    public JoystickButton buttonReefZoneLeft4 = new JoystickButton(buttonBoardPlayer2, 4);
    public JoystickButton buttonReefZoneLeft5 = new JoystickButton(buttonBoardPlayer2, 5);
    public JoystickButton buttonReefZoneLeft6 = new JoystickButton(buttonBoardPlayer2, 6);
    public JoystickButton buttonReefZoneRight1 = new JoystickButton(buttonBoardPlayer3, 4);
    public JoystickButton buttonReefZoneRight2 = new JoystickButton(buttonBoardPlayer3,3);
    public JoystickButton buttonReefZoneRight3 = new JoystickButton(buttonBoardPlayer3, 2);
    public JoystickButton buttonReefZoneRight4 = new JoystickButton(buttonBoardPlayer3, 1);
    public JoystickButton buttonReefZoneRight5 = new JoystickButton(buttonBoardPlayer2, 8);
    public JoystickButton buttonReefZoneRight6 = new JoystickButton(buttonBoardPlayer2, 7);

    // miscellaneous buttons
    public JoystickButton buttonAlgaeKnockoff = new JoystickButton(buttonBoardPlayer3, 8);
    public JoystickButton buttonUtilityA = new JoystickButton(buttonBoardPlayer3, 7);
    public JoystickButton buttonUtilityB = new JoystickButton(buttonBoardPlayer3, 6);
    public JoystickButton buttonReset = new JoystickButton(buttonBoardPlayer3, 5);

    public ButtonBoard() {
    }

    private ScoringMode scoringMode = ScoringMode.L4;

    public enum ScoringMode {
        L1,
        L2,
        L3,
        L4,
        ALGAE;
    }

    public ScoringMode getScoringMode() {
        return scoringMode;
    }

    public void setScoringMode(ScoringMode scoringMode) {
        this.scoringMode = scoringMode;
        Logger.recordOutput("/buttons/buttonboard/scoringMode", getScoringMode());
    }

}
