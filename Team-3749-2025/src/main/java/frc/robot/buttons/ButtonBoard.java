package frc.robot.buttons;

public class ButtonBoard {

    private ScoringLocation scoringLocation;

    public ScoringLocation getScoringLocation(){
        return scoringLocation;
    }

    public void setScoringLocation(ScoringLocation location)
    {
      this.scoringLocation = location;
    }

    public enum ScoringLocation {
      L1,
      L2,
      L3,
      L4,
      ALGAE;  
    }
    
}
