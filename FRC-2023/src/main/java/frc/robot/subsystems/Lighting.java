package frc.robot.subsystems;

import edu.wpi.first.hal.util.UncleanStatusException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lighting extends SubsystemBase{
    private SerialPort arduino;

    public Lighting(){
        try {
            arduino = new SerialPort(9600, SerialPort.Port.kUSB);
            arduino.setTimeout(3);
        }
        catch (UncleanStatusException e){
            DriverStation.reportError("Unable to connect to arduino.", false);
        }
    }

    public void sendString(String data) {
        arduino.writeString(data);
    }
}
