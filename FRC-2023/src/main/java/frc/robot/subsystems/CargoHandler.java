package frc.robot.subsystems;


// This is the Intake subsystem 
// This is not meant to handle arm/claw 

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CargoHandlerConstants;

public class CargoHandler extends SubsystemBase {

    // Motors
    private final CANSparkMax out_horizontalIntake = new CANSparkMax(CargoHandlerConstants.horizontalMotorID, MotorType.kBrushed);
    private final CANSparkMax out_verticalIntake = new CANSparkMax(CargoHandlerConstants.verticalMotorID, MotorType.kBrushed);
    private final CANSparkMax out_vacuumMotor = new CANSparkMax(CargoHandlerConstants.gripperMotorID, MotorType.kBrushed);
    private final CANSparkMax out_turnTable = new CANSparkMax(CargoHandlerConstants.turnTableMotorID, MotorType.kBrushed);

    // Vertical intake absolute encoder
    private final DutyCycleEncoder in_verticalIntakeEncoder = new DutyCycleEncoder(CargoHandlerConstants.verticalDIOEncoderID);

    // Agitator Solenoid
    private final Solenoid out_agitatorSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, CargoHandlerConstants.agitatorSolenoidID);
    
    // Front horizontal intake wheel sensor
    private final DigitalInput in_horizontalIntakeSensor = new DigitalInput(CargoHandlerConstants.horizontalWheelSensor);
    
    public double intakeTargetAngle = CargoHandlerConstants.verticalIntakePositionTarget;
    public PIDController verticalIntakePositionPID = new PIDController(CargoHandlerConstants.verticalIntakePositionPID_P, CargoHandlerConstants.verticalIntakePositionPID_I, CargoHandlerConstants.verticalIntakePositionPID_D);
    PowerDistribution p = new PowerDistribution();
    
    public CargoHandler() {

        // Configure motor controllers for break mode
        configureSparkBrake(out_horizontalIntake);
        configureSparkBrake(out_verticalIntake);

        // Configure motor controllers for coast mode
        configureSparkCoast(out_vacuumMotor);
        SmartDashboard.putNumber("vertical intake PID P", CargoHandlerConstants.verticalIntakePositionPID_P);
     SmartDashboard.putNumber("vertical intake PID I", CargoHandlerConstants.verticalIntakePositionPID_I);
   SmartDashboard.putNumber("vertical intake PID D", CargoHandlerConstants.verticalIntakePositionPID_D);
   SmartDashboard.putNumber("Intake targ", CargoHandlerConstants.verticalIntakePositionTarget);
    }

    @Override
    public void periodic(){
        UpdatePID();
        updateStatus();
    }
    

    private void configureSparkCoast(CANSparkMax spark){
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(CargoHandlerConstants.currentLimit);
        spark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    private void configureSparkBrake(CANSparkMax spark){
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(CargoHandlerConstants.currentLimit);
        spark.setIdleMode(CANSparkMax.IdleMode.kBrake); 
    }

    // Setter functions

    public void setVerticalIntake (double speed){
        out_verticalIntake.set(speed);
    }

    public void setTurntableSpeed (double speed){
        out_turnTable.set(speed);
    }
    //updates the vertical and horizontal motor intake speeds
    public void updateIntakeState(boolean intakeOut, boolean intakeIn) {
        if (intakeOut) {
            setVerticalIntake(-CargoHandlerConstants.verticalIntakeOutSpeed); 
            setHorizontalIntake(CargoHandlerConstants.horizontalIntakeOutSpeed); // horizontal intake is in reverse 
        } else if(intakeIn && !intakeOut) {
            setVerticalIntake(CargoHandlerConstants.verticalIntakeInSpeed); //might need to reverse direction
            setHorizontalIntake(-CargoHandlerConstants.horizontalIntakeInSpeed);
        } else {
            double angle = ((in_verticalIntakeEncoder.get()+Math.abs((int)in_verticalIntakeEncoder.get()) + 10) % 1) * 360;
            double dif = (180 + intakeTargetAngle - angle + 720) % (360) - 180;
            double val = -verticalIntakePositionPID.calculate(dif/180);
            //System.out.println("G " + ((in_verticalIntakeEncoder.getPosition()*Math.PI/180) % (2 * Math.PI)));
            //System.out.println(val);
            setVerticalIntake(val); 
            setHorizontalIntake(0);
        }  
     }
    
    //returns whether the wheels are in the frame
    public boolean verticalIntakeInFrame() {
        double angle = ((in_verticalIntakeEncoder.get()+Math.abs((int)in_verticalIntakeEncoder.get()) + 10) % 1) * 360;
        double dif = (180 + intakeTargetAngle - angle + 720) % (360) - 180;
        return Math.abs(dif) < CargoHandlerConstants.verticalIntakePositionAtTargetMoE;
    }
    
    
  public void UpdatePID() {
    double verticalP = SmartDashboard.getNumber("vertical intake PID P", CargoHandlerConstants.verticalIntakePositionPID_P);
    double verticalI = SmartDashboard.getNumber("vertical intake PID I", CargoHandlerConstants.verticalIntakePositionPID_I);
    double verticalD = SmartDashboard.getNumber("vertical intake PID D", CargoHandlerConstants.verticalIntakePositionPID_D);
      if(verticalP != verticalIntakePositionPID.getP() || verticalI != verticalIntakePositionPID.getI() || verticalD != verticalIntakePositionPID.getD()) {
        verticalIntakePositionPID.setP(verticalP);
        verticalIntakePositionPID.setI(verticalI);
        verticalIntakePositionPID.setD(verticalD);
      }
  }

    public void setHorizontalIntake (double speed){
        out_horizontalIntake.set(speed);
    }

    public void setVaccumMotor (double speed){
        out_vacuumMotor.set(speed);
    }

    public void setAgitatorSolenoid(boolean state){
        out_agitatorSolenoid.set(state);  
    }

    // Getter functions

    public boolean getAgitatorSolenoid(){
       return out_agitatorSolenoid.get();
    }

    public boolean getHorizontalWheelSensor(){
        return in_horizontalIntakeSensor.get();
    }


    // Update Cargo handler data to dashboard    
    public void updateStatus(){
        /*SmartDashboard.putBoolean("agitatorSolenoid", getAgitatorSolenoid());
        SmartDashboard.putBoolean("HorizontalWheelSensor", getHorizontalWheelSensor());*/
    }


}

