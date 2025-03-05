package frc.robot.subsystems.leds.sim;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import frc.robot.subsystems.leds.LEDIO;

public class LedSim implements LEDIO{
    public AddressableLEDSim leds;

    public LedSim(int port, AddressableLEDBuffer buf) {

    }
}
