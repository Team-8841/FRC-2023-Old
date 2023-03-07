package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

//Arm is the Claw 

public class Arm extends SubsystemBase {

    // I2C Port used for color sensor
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    // Arm Motors
    private final CANSparkMax out_leadArmMotor = new CANSparkMax(ArmConstants.rightArmMotorID, MotorType.kBrushed);
    private final SparkMaxAbsoluteEncoder in_leadArmMotorEncoder = out_leadArmMotor.getAbsoluteEncoder(Type.kDutyCycle);
    private final CANSparkMax out_followArmMotor = new CANSparkMax(ArmConstants.leftArmMotorID, MotorType.kBrushed);
    private final CANSparkMax out_gripperMotor = new CANSparkMax(ArmConstants.gripperMotorID, MotorType.kBrushed);
    // Arm Solenoids
    private final Solenoid out_gripperSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            ArmConstants.gripperSolenoidID);
    private final Solenoid out_armSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, ArmConstants.armSolenoidID);

    // Color Sensor (duh)
    private final ColorSensorV3 out_colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch m_colorMatcher = new ColorMatch();
    // Move to constants file once the color purple sequence has been determined
    private final Color kPurpleTarget = new Color(0.143, 0.427, 0.429);

    //Arm Positions //TO DO: fix this for what we need, this is just measurement conversions for convenience
    private final double topPurple = 0.127; //inches to meters
    private final double middlePurple = 0.5969;
    private final double home = 1.2065;
    private final double topYellow = 1.1684;
    private final double middleYellow = 0.8636;

    PIDController anglePID = new PIDController(ArmConstants.anglePID_P, ArmConstants.anglePID_I,
            ArmConstants.anglePID_D);

    public Arm() {
        anglePID.setSetpoint(0);

        // Set sparks to brake mode
        configureSparkBrake(out_leadArmMotor);
        configureSparkBrake(out_followArmMotor);

        // Make left arm motor follow right arm
        out_followArmMotor.follow(out_leadArmMotor, true);

        out_leadArmMotor.setInverted(false);
        // out_followArmMotor.setInverted(true);

        // Set color matching
        m_colorMatcher.addColorMatch(kPurpleTarget);

    }

    @Override
    public void periodic() {
        updateStatus();
        UpdatePID();
    }

    private void configureSparkBrake(CANSparkMax spark) {
        spark.restoreFactoryDefaults();
        spark.setSmartCurrentLimit(ArmConstants.currentLimit);
        spark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    // Setter Methods

    public void setArmSolenoid(boolean state){
        out_armSolenoid.set(state);  
    }

    public void setArmExtension(boolean state) {
        out_gripperSolenoid.set(state);
    }

    public void setArmMotors(double speed) {
        out_leadArmMotor.set(speed);
        // out_followArmMotor.set(speed);
    }

    public void setGripperSpeed(double speed) {
        out_gripperMotor.set(speed);
    }

    public void setArmAngle(double angle) {
        setArmMotors(anglePID.calculate(angle - getArmAngle()));
    }

    // Getter Methods

    public boolean getArmSolenoid() {
        return out_armSolenoid.get();
    }

    public double getArmAngle() {
        return in_leadArmMotorEncoder.getPosition();
    }

    public boolean getArmExtension() {
        return out_gripperSolenoid.get();
    }

    public Color getColor() {
        return out_colorSensor.getColor();
    }

    public int getProx() {
        return out_colorSensor.getProximity();
    }

    public boolean hasCube() {
        return m_colorMatcher.matchClosestColor(getColor()).color == kPurpleTarget;
    }

    // Update arm data to dahsboard
    public void updateStatus() {
        /*SmartDashboard.putNumber("prox", getProx());
        SmartDashboard.putBoolean("Arm pivot", getArmSolenoid());
        SmartDashboard.putBoolean("Arm extension", getArmExtension());
        SmartDashboard.putNumber("Red, Green, Blue", getColor().red);
        SmartDashboard.putNumber("Red, Green, Blue", getColor().green);
        SmartDashboard.putNumber("Red, Green, Blue", getColor().blue);*/
    }

    public void UpdatePID() {
        /*
         * double angleP = SmartDashboard.getNumber("Arm angle PID P",
         * ArmConstants.anglePID_P);
         * double angleI = SmartDashboard.getNumber("Arm angle PID I",
         * ArmConstants.anglePID_I);
         * double angleD = SmartDashboard.getNumber("Arm angle PID D",
         * ArmConstants.anglePID_D);
         * if(angleP != anglePID.getP() || angleI != anglePID.getI() || angleD !=
         * anglePID.getD()) {
         * anglePID.setP(angleP);
         * anglePID.setI(angleI);
         * anglePID.setD(angleD);
         * }
         */
    }

}
