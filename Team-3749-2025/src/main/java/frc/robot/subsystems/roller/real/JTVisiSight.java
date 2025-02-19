package frc.robot.subsystems.roller.real;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.RollerConstants;

public class JTVisiSight implements PhotoelectricIO{

    private DigitalInput photoelectricSensor = new DigitalInput(RollerConstants.Scoring.sensorPort);

    public JTVisiSight(){

    }

    @Override
    public void updateData(PhotoelectricData data){
        // sensor is inverted
        data.sensing = !photoelectricSensor.get();

    }
}