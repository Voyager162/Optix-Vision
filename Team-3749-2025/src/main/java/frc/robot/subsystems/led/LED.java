package frc.robot.subsystems.led;

import java.sql.Driver;
import java.util.Optional;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.led.LEDConstants.LEDPattern;

public class LED extends SubsystemBase {

    private AddressableLED LEDs = new AddressableLED(9); //port
    private AddressableLEDBuffer LEDBuffer = new AddressableLEDBuffer(frc.robot.subsystems.led.LEDConstants.length);
    private LEDPattern currentPattern = LEDPattern.WHITE;
    private int hue = 0;
    private double brightness = 1;

    public LED()
    {
       LEDs.setLength(LEDBuffer.getLength());
       LEDs.setData(LEDBuffer);
       LEDs.start();
       setLEDPattern(LEDPattern.WHITE);
    }

    public LED(double brightness)
    {
       LEDs.setLength(LEDBuffer.getLength());
       LEDs.setData(LEDBuffer);
       LEDs.start();
       setLEDPattern(LEDPattern.WHITE);
       setBrightness(brightness);
    }

    private LEDPattern teamColorLED()
    {
       Optional<Alliance> team = DriverStation.getAlliance(); //i hate doing it this way but it throws an error without it
       return team.get() == Alliance.Blue ? LEDPattern.BLUE : LEDPattern.RED;
    }

    private void setLEDOneColorRGB(int R, int G, int B)
    {   
        double curBrightness = brightness;
        if (DriverStation.isEnabled()){
            curBrightness = 1;
        }
        int setR = (int)(R * curBrightness);
        int setG = (int)(G * curBrightness);
        int setB = (int)(B * curBrightness);


        for(int i=0;i<LEDBuffer.getLength();i++)
        {
            LEDBuffer.setRGB(i, setR, setG, setB);
        }
    }

    private void setLEDOneColorHSV(int H, int S, int V)
    {
        for(int i=0;i<LEDBuffer.getLength();i++)
        {
            LEDBuffer.setRGB(i, H, S, V);
        }
    }

    private void setLEDRainbow() //requires a loop
    {
        hue++;
        setLEDOneColorHSV(hue, 255, 255);
        if(hue >= 350)
        {
            hue = 0;
        }
    }

    public void setLEDPattern(LEDPattern pattern)
    {
        if (pattern == LEDPattern.WHITE && RobotController.getBatteryVoltage() <8){
            pattern = LEDPattern.RED;
        }
        this.currentPattern = pattern;
    }

    public LEDPattern getCurrentPattern(){
        return currentPattern;
    }

    // runs every 0.02 sec
    @Override
    public void periodic()
    {
        switch(this.currentPattern)
        {
            case RED:
                setLEDOneColorRGB(255,0,0);
            break;

            case BLUE:
                setLEDOneColorRGB(0,0,255);
            break;

            case GREEN:
                setLEDOneColorRGB(0, 255, 0);
            break;

            case YELLOW:
                setLEDOneColorRGB(255, 255, 0);
            break;

            case RAINBOW:
                setLEDRainbow();
            break;

            case WHITE:
                setLEDOneColorRGB(255, 255, 255);
            break;

            case NOTHING:


                setLEDOneColorRGB(0, 0, 0);
            break;

            default:
                setLEDOneColorRGB(210,105,30);
                System.out.println("LEDpattern missing case");
            break;
        }
        LEDs.setData(LEDBuffer);
        
    }
    
    /***
     * 
     * @param setBrightness value from 1 - 0 for brightness
     */
    public void setBrightness(double setBrightness) {
        brightness = setBrightness;
    }
}