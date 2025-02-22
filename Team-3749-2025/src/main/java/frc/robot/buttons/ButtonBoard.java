package frc.robot.buttons;
public class ButtonBoard {

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

    public void setScoringMode(ScoringMode scoringMode) {
        this.scoringMode = scoringMode;
    }

}
