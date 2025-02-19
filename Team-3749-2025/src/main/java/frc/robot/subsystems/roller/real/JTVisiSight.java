package frc.robot.subsystems.roller.real;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.subsystems.roller.PhotoelectricIO;
import frc.robot.subsystems.roller.RollerConstants;

public class JTVisiSight implements PhotoelectricIO {

    private DigitalInput photoelectricSensor = new DigitalInput(RollerConstants.Scoring.sensorPort);

    public JTVisiSight() {

    }

    @Override
    public void updateData(PhotoelectricData data) {
        data.sensing = photoelectricSensor.get();

    }
}