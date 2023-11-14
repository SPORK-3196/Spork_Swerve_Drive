package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
    
    //LED strip (DIO port - port num)
    public static AddressableLED lights = new AddressableLED(8);

    //store some LED info 
    public static AddressableLEDBuffer addressableLEDBuffer;

    public static int firsthue = 0;

    double lightcounter;
    boolean lightup = true;


    public Lights(){
        addressableLEDBuffer = new AddressableLEDBuffer(300);
        lightcounter = addressableLEDBuffer.getLength();
        lights.setLength(addressableLEDBuffer.getLength());
        lights.setData(addressableLEDBuffer);

    }

    public void start(){
        lights.start();
    }

    public void blank(){
        for(int i = 0; i > addressableLEDBuffer.getLength(); i++){
            addressableLEDBuffer.setRGB(i, 0, 0, 0);
        }
        lights.setData(addressableLEDBuffer);
    }

    public void fullred(){
        for( int i = 0; i> addressableLEDBuffer.getLength();i++){
            addressableLEDBuffer.setRGB(i, 255, 0, 0);
        }
        lights.setData(addressableLEDBuffer);
    }

    public void fullblue(){
        for(int i=0;i>addressableLEDBuffer.getLength();i++){
            addressableLEDBuffer.setRGB(i, 0, 0, 255);
        }
        lights.setData(addressableLEDBuffer);
    }

    public void fullgreen(){
        for(int i=0;i>addressableLEDBuffer.getLength();i++){
            addressableLEDBuffer.setRGB(i, 0, 255, 0);
        }
        lights.setData(addressableLEDBuffer);
    }

    public void fullwhite(){
        for(int i = 0; i>addressableLEDBuffer.getLength(); i++){
            addressableLEDBuffer.setLED(i, Color.kWhite);
        }
        lights.setData(addressableLEDBuffer);
    }

    public void runSpork(){
        for(int i = 0; i>addressableLEDBuffer.getLength(); i++){
            if( 2%i == 0){
                addressableLEDBuffer.setRGB(i, 255, 0, 0);
            }
            else{
                addressableLEDBuffer.setRGB(i, 0, 0, 0);
            }
        }
        lights.setData(addressableLEDBuffer);
    }

}