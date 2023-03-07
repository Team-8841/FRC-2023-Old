package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

//Arm is the Claw 

public class ColorTesting extends SubsystemBase{

    // I2C Port used for color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    
    // Color Sensor 
    private final ColorSensorV3 out_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    //Move to constants file once the color purple sequence has been determined 
    // private final Color kPurpleTarget = new Color(0.143, 0.427, 0.429);
    
    public ColorTesting() {

        //Set color matching
        // m_colorMatcher.addColorMatch(kPurpleTarget);
    
        
    }

   
    

    // Getter Methods

  
    public Color getColor(){
      //System.out.print(out_colorSensor.getRawColor());
        return out_colorSensor.getColor();
    }
    public int getProx(){
       return out_colorSensor.getProximity();
    }

    // public boolean hasCube() {
    //     return m_colorMatcher.matchClosestColor(getColor()).color == kPurpleTarget;
    // }



}

