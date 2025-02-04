package frc.robot.utils;

public class ButtonBoard {

    private ScoringLocation scoringLocation;

    public ScoringLocation getScoringLocation(){
        return scoringLocation;
    }

    public enum ScoringLocation {
      L1,
      L2,
      L3,
      L4,
      ALGAE;  
    }
    
}
