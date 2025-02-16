package frc.robot.buttons;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class ButtonBoard {

    public GenericHID buttonBoardPlayer1 = new GenericHID(1);
    public GenericHID buttonBoardPlayer2 = new GenericHID(2);
    public GenericHID buttonBoardPlayer3 = new GenericHID(3);

    // coral elevator positions
    public JoystickButton buttonl1 = new JoystickButton(buttonBoardPlayer3, 2);
    public JoystickButton buttonl2 = new JoystickButton(buttonBoardPlayer3, 1);
    public JoystickButton buttonl3 = new JoystickButton(buttonBoardPlayer1, 6);
    public JoystickButton buttonl4 = new JoystickButton(buttonBoardPlayer1, 5);

    // source buttons
    public JoystickButton buttonRightSource = new JoystickButton(buttonBoardPlayer1, 4);
    public JoystickButton buttonLeftSource = new JoystickButton(buttonBoardPlayer1, 7);

    // reef positions
    public JoystickButton buttonReefZoneA = new JoystickButton(buttonBoardPlayer3, 6);
    public JoystickButton buttonReefZoneB = new JoystickButton(buttonBoardPlayer2, 5);
    public JoystickButton buttonReefZoneC = new JoystickButton(buttonBoardPlayer2, 6);
    public JoystickButton buttonReefZoneD = new JoystickButton(buttonBoardPlayer3, 3);
    public JoystickButton buttonReefZoneE = new JoystickButton(buttonBoardPlayer3, 4);
    public JoystickButton buttonReefZoneF = new JoystickButton(buttonBoardPlayer2, 8);
    public JoystickButton buttonReefZoneG = new JoystickButton(buttonBoardPlayer2, 7);
    public JoystickButton buttonReefZoneH = new JoystickButton(buttonBoardPlayer2, 4);
    public JoystickButton buttonReefZoneI = new JoystickButton(buttonBoardPlayer2, 3);
    public JoystickButton buttonReefZoneJ = new JoystickButton(buttonBoardPlayer2, 2);
    public JoystickButton buttonReefZoneK = new JoystickButton(buttonBoardPlayer2, 1);
    public JoystickButton buttonReefZoneL = new JoystickButton(buttonBoardPlayer3, 5);

    // miscellaneous buttons
    public JoystickButton buttonAlgaeKnockoff = new JoystickButton(buttonBoardPlayer1, 3);
    public JoystickButton buttonUtilityA = new JoystickButton(buttonBoardPlayer1, 1);
    public JoystickButton buttonUtilityB = new JoystickButton(buttonBoardPlayer1, 2);
    public JoystickButton buttonPlayer1Start = new JoystickButton(buttonBoardPlayer1, 8);

    public ButtonBoard() {
    }

    private ScoringMode scoringMode;

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

    public Object setScoringMode(ScoringMode algae) {
        throw new UnsupportedOperationException("Unimplemented method 'setScoringMode'");
    }

}
