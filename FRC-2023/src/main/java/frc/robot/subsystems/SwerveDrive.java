package frc.robot.subsystems;

import java.text.DecimalFormat;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.Grid;
import frc.robot.Constants.SwerveDriveMathConstants;
import frc.robot.helpers.Orientation;
import frc.robot.helpers.SwerveTiltOutput;
import frc.robot.helpers.Vector2;

public class SwerveDrive extends SubsystemBase {
    double lastStepTime = 0;
    public Timer stepTimer = new Timer();
    double robotRotationOffset;
    TalonFX  out_frontLeftDriveMotor = new TalonFX(Constants.DriveConstants.swerveMotorDrivePorts[0]);
    TalonFX  out_frontRightDriveMotor = new TalonFX(Constants.DriveConstants.swerveMotorDrivePorts[1]);
    TalonFX  out_backLeftDriveMotor = new TalonFX(Constants.DriveConstants.swerveMotorDrivePorts[2]);
    TalonFX  out_backRightDriveMotor = new TalonFX(Constants.DriveConstants.swerveMotorDrivePorts[3]);

    TalonFX  out_frontLeftTurnMotor = new TalonFX(Constants.DriveConstants.swerveMotorTurnPorts[0]);
    TalonFX  out_frontRightTurnMotor = new TalonFX(Constants.DriveConstants.swerveMotorTurnPorts[1]);
    TalonFX  out_backLeftTurnMotor = new TalonFX(Constants.DriveConstants.swerveMotorTurnPorts[2]);
    TalonFX  out_backRightTurnMotor = new TalonFX(Constants.DriveConstants.swerveMotorTurnPorts[3]);
    
    CANCoder in_frontLeftEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[0]);
    CANCoder in_frontRightEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[1]);
    CANCoder in_backLeftEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[2]);
    CANCoder in_backRightEncoder = new CANCoder(Constants.DriveConstants.swerveEncoderPorts[3]);

    public boolean fieldOriented = true;
    
    public final AHRS in_navx = new AHRS(SPI.Port.kMXP, (byte) 200);
    PIDController pid = new PIDController(Constants.DriveConstants.turnPID_P, Constants.DriveConstants.turnPID_I, Constants.DriveConstants.turnPID_D);
    //Isn't used, just used to store PID values for updatePID :)
    PIDController speedPID = new PIDController(Constants.DriveConstants.speedPID_P, Constants.DriveConstants.speedPID_I, Constants.DriveConstants.speedPID_D);
    
    public Vector2 position = new Vector2();
    //Shuffle Board//
    private GenericEntry maxSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, SwerveDriveMathConstants.maxMotorUse_default)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1)).getEntry();
    }
    private GenericEntry motorSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, "<0, 0>")
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    private GenericEntry encoderSwerveShuffle(String name) {
        return Shuffleboard.getTab("Drive").add(name, "N/A")
        .withWidget(BuiltInWidgets.kTextView).getEntry();
    }
    private GenericEntry maxSwerveMotorUse = maxSwerveShuffle("Max Swerve Motor Use");
    private GenericEntry maxSwerveRotationSpeed = maxSwerveShuffle("Max Swerve Rotation Speed");
    private GenericEntry maxSwerveSpeed = maxSwerveShuffle("Max Swerve Speed");
    private GenericEntry[] swerveWheels = {motorSwerveShuffle("Front Left Wheel"), motorSwerveShuffle("Front Right Wheel"), motorSwerveShuffle("Back Left Wheel"), motorSwerveShuffle("Back Right Wheel")};
    private GenericEntry[] encoders = {encoderSwerveShuffle("Front Left Encoder"), encoderSwerveShuffle("Front Right Encoder"), encoderSwerveShuffle("Back Left Encoder"), encoderSwerveShuffle("Back Right Encoder")};
    private GenericEntry NAVXDisplacement = Shuffleboard.getTab("Drive").add("NAVX Displacement", "<0, 0, 0>")
    .withWidget(BuiltInWidgets.kTextView).getEntry();
    /////////////////


    public SwerveDrive() {
        stepTimer.start();
        lastStepTime = stepTimer.get();
        speedPID = new PIDController(Constants.DriveConstants.speedPID_P, Constants.DriveConstants.speedPID_I, Constants.DriveConstants.speedPID_D);
        out_frontLeftDriveMotor.config_kP(0, DriveConstants.speedPID_P);
        out_frontLeftDriveMotor.config_kI(0, DriveConstants.speedPID_I);
        out_frontLeftDriveMotor.config_kD(0, DriveConstants.speedPID_D);

        out_frontRightDriveMotor.config_kP(0, DriveConstants.speedPID_P);
        out_frontRightDriveMotor.config_kI(0, DriveConstants.speedPID_I);
        out_frontRightDriveMotor.config_kD(0, DriveConstants.speedPID_D);
        
        out_backLeftDriveMotor.config_kP(0, DriveConstants.speedPID_P);
        out_backLeftDriveMotor.config_kI(0, DriveConstants.speedPID_I);
        out_backLeftDriveMotor.config_kD(0, DriveConstants.speedPID_D);
        
        out_backRightDriveMotor.config_kP(0, DriveConstants.speedPID_P);
        out_backRightDriveMotor.config_kI(0, DriveConstants.speedPID_I);
        out_backRightDriveMotor.config_kD(0, DriveConstants.speedPID_D);

        UpdatePID();
    }

    //does some useful startup functions
    public void Start() {
        lastStepTime = stepTimer.get();
        pid.enableContinuousInput(-Math.PI, Math.PI);
        pid.setSetpoint(0);

        UpdatePID();

        configureFalconBrake(out_frontLeftDriveMotor);
        configureFalconBrake(out_frontRightDriveMotor);
        configureFalconBrake(out_backLeftDriveMotor);
        configureFalconBrake(out_backRightDriveMotor);

        //
        configureFalconCoast(out_frontLeftTurnMotor);
        configureFalconCoast(out_frontRightTurnMotor);
        configureFalconCoast(out_backLeftTurnMotor);
        configureFalconCoast(out_backRightTurnMotor);

        in_navx.resetDisplacement();
    }
    

  private void configureFalconCoast(TalonFX talon) {
    talon.configFactoryDefault();
    talon.configOpenloopRamp(Constants.DriveConstants.driveMotorRampTime);
    talon.setNeutralMode(NeutralMode.Coast); 
  }

  private void configureFalconBrake(TalonFX talon) {
    talon.configFactoryDefault();
    talon.configOpenloopRamp(Constants.DriveConstants.driveMotorRampTime);
    talon.setNeutralMode(NeutralMode.Brake); 
  }

  
  @Override
  public void periodic(){
    updateStatus();
      UpdatePID();
  }

    //makes angle in radians positive
    double StandardizeRAD(double radians) {
        return (radians + 8 * Math.PI) % (2 * Math.PI);
    }

    //gets the true wheel angle
    double GetEncoderAngle(CANCoder encoder, int ind) {
        SmartDashboard.putNumber("Encoder " + ind, encoder.getAbsolutePosition());
        return StandardizeRAD((encoder.getAbsolutePosition() - Constants.DriveConstants.turnAngleOffsets[ind]) * Math.PI / 180 * (Constants.DriveConstants.turnEncoderInversed[ind] ? 1 : -1));
    }

    //uses the PID to turn the wheels
    Vector2 TurnPID(TalonFX motor, double currentAngle, double targetAngle) {
        double angleDif = Math.PI / 2 - (currentAngle - targetAngle - Math.PI/2 + Math.PI*5) % Math.PI;
        motor.set(ControlMode.PercentOutput, pid.calculate(angleDif));
        return new Vector2(angleDif, Math.signum((currentAngle - targetAngle - Math.PI/2 + Math.PI*6) % (2*Math.PI) - Math.PI));
    }
    
  public void UpdatePID() {
    double rotationP = SmartDashboard.getNumber("wheel rotation PID P", DriveConstants.turnPID_P);
    double rotationI = SmartDashboard.getNumber("wheel rotation PID I", DriveConstants.turnPID_I);
    double rotationD = SmartDashboard.getNumber("wheel rotation PID D", DriveConstants.turnPID_D);
      if(rotationP != pid.getP() || rotationI != pid.getI() || rotationD != pid.getD()) {
        pid.setP(rotationP);
        pid.setI(rotationI);
        pid.setD(rotationD);
      }
      double speedP = SmartDashboard.getNumber("wheel speed PID P", DriveConstants.speedPID_P);
      double speedI = SmartDashboard.getNumber("wheel speed PID I", DriveConstants.speedPID_I);
      double speedD = SmartDashboard.getNumber("wheel speed PID D", DriveConstants.speedPID_D);
        if(speedP != speedPID.getP() || speedI != speedPID.getI() || speedD != speedPID.getD()) {
            speedPID.setP(speedP);
            speedPID.setI(speedI);
            speedPID.setD(speedD);
            
          
            out_frontLeftDriveMotor.config_kP(0, DriveConstants.speedPID_P);
            out_frontLeftDriveMotor.config_kI(0, DriveConstants.speedPID_I);
            out_frontLeftDriveMotor.config_kD(0, DriveConstants.speedPID_D);

            out_frontRightDriveMotor.config_kP(0, DriveConstants.speedPID_P);
            out_frontRightDriveMotor.config_kI(0, DriveConstants.speedPID_I);
            out_frontRightDriveMotor.config_kD(0, DriveConstants.speedPID_D);
            
            out_backLeftDriveMotor.config_kP(0, DriveConstants.speedPID_P);
            out_backLeftDriveMotor.config_kI(0, DriveConstants.speedPID_I);
            out_backLeftDriveMotor.config_kD(0, DriveConstants.speedPID_D);
            
            out_backRightDriveMotor.config_kP(0, DriveConstants.speedPID_P);
            out_backRightDriveMotor.config_kI(0, DriveConstants.speedPID_I);
            out_backRightDriveMotor.config_kD(0, DriveConstants.speedPID_D);
        }
  }

    //sets the wheel speeds based on the inputs and constraints set in the constants file
    public Vector2[] SetWheelSpeeds(double joystickMag, double joystickAngle, double joystickRotation, boolean inMeters) {
        
        double maxMotorUse = maxSwerveMotorUse.getDouble(SwerveDriveMathConstants.maxMotorUse_default);
        double maxRotationSpeed = maxSwerveRotationSpeed.getDouble(SwerveDriveMathConstants.maxRotationSlider_default);
        double maxMoveSpeed = maxSwerveSpeed.getDouble(SwerveDriveMathConstants.maxMoveSpeed_default);
        if(maxMotorUse != SwerveDriveMathConstants.maxMotorUse || maxRotationSpeed != SwerveDriveMathConstants.maxRotationSlider || maxMoveSpeed != SwerveDriveMathConstants.maxMoveSpeed) {
        SwerveDriveMathConstants.maxMotorUse = maxMotorUse;
        SwerveDriveMathConstants.maxRotationSlider = maxRotationSpeed;
        SwerveDriveMathConstants.maxMoveSpeed = maxMoveSpeed;
        SwerveDriveMathConstants.updateMax();
        }

        if(!inMeters) {
            joystickMag = Math.min(joystickMag, 1);
        }
        //
        double robotRotation = getYAW();
        Vector2[] wheelSpeeds = SwerveDriveMath.WheelSpeeds(joystickMag, joystickAngle, joystickRotation, robotRotation, inMeters);

        double frontLeftAngle = 2 * Math.PI - GetEncoderAngle(in_frontLeftEncoder, 0);
        double frontRightAngle = 2 * Math.PI - GetEncoderAngle(in_frontRightEncoder, 1);
        double backLeftAngle = 2 * Math.PI - GetEncoderAngle(in_backLeftEncoder, 2);
        double backRightAngle = 2 * Math.PI - GetEncoderAngle(in_backRightEncoder, 3);

        Vector2[] angleDifs = {new Vector2(), new Vector2(), new Vector2(), new Vector2()};
        
        if(joystickMag == 0 && joystickRotation == 0) {
            out_frontLeftTurnMotor.set(ControlMode.PercentOutput, 0);
            out_frontRightTurnMotor.set(ControlMode.PercentOutput, 0);
            out_backLeftTurnMotor.set(ControlMode.PercentOutput, 0);
            out_backRightTurnMotor.set(ControlMode.PercentOutput, 0);
        } else {
            angleDifs[0] = TurnPID(out_frontLeftTurnMotor, frontLeftAngle, wheelSpeeds[0].getAngle());
            angleDifs[1] = TurnPID(out_frontRightTurnMotor, frontRightAngle, wheelSpeeds[1].getAngle());
            angleDifs[2] = TurnPID(out_backLeftTurnMotor, backLeftAngle, wheelSpeeds[2].getAngle());
            angleDifs[3] = TurnPID(out_backRightTurnMotor, backRightAngle, wheelSpeeds[3].getAngle());
        }

        if(!inMeters) {
            out_frontLeftDriveMotor.set(ControlMode.PercentOutput, wheelSpeeds[0].getMagnitude() * angleDifs[0].y);
            out_frontRightDriveMotor.set(ControlMode.PercentOutput, wheelSpeeds[1].getMagnitude() * angleDifs[1].y);
            out_backLeftDriveMotor.set(ControlMode.PercentOutput, wheelSpeeds[2].getMagnitude() * angleDifs[2].y);
            out_backRightDriveMotor.set(ControlMode.PercentOutput, wheelSpeeds[3].getMagnitude() * angleDifs[3].y);
        } else {
            //velocity is per 100 ms, divided by 10 to convert to seconds
            out_frontLeftDriveMotor.set(ControlMode.Velocity, wheelSpeeds[0].getMagnitude()/DriveConstants.talon_DriveMotor_SensorToMeters / 10);
            out_frontRightDriveMotor.set(ControlMode.Velocity, wheelSpeeds[1].getMagnitude()/DriveConstants.talon_DriveMotor_SensorToMeters / 10);
            out_backLeftDriveMotor.set(ControlMode.Velocity, wheelSpeeds[2].getMagnitude()/DriveConstants.talon_DriveMotor_SensorToMeters / 10);
            out_backRightDriveMotor.set(ControlMode.Velocity, wheelSpeeds[3].getMagnitude()/DriveConstants.talon_DriveMotor_SensorToMeters / 10);
        }
        //System.out.println(out_frontLeftDriveMotor.getSelectedSensorPosition());

        //Shuffle Board//
        for(int i = 0; i < wheelSpeeds.length; i++) {
            swerveWheels[i].setString(wheelSpeeds[i].toStringPolar(3));
        }
        DecimalFormat format = new DecimalFormat("#.###");
        encoders[0].setString(format.format(frontLeftAngle * 180 / Math.PI) + " | " + format.format(angleDifs[0].x * 180 / Math.PI));
        encoders[1].setString(format.format(frontRightAngle * 180 / Math.PI) + " | " + format.format(angleDifs[1].x * 180 / Math.PI));
        encoders[2].setString(format.format(backLeftAngle * 180 / Math.PI)+ " | " + format.format(angleDifs[2].x * 180 / Math.PI));
        encoders[3].setString(format.format(backRightAngle * 180 / Math.PI) + " | " + format.format(angleDifs[3].x * 180 / Math.PI));
        /////////////////
        NAVXDisplacement.setString("<" + in_navx.getDisplacementX() + ", " + in_navx.getDisplacementY() + ", " + in_navx.getDisplacementZ() + ">");
        return wheelSpeeds;
    }

    //gets the robot's yaw
    public double getYAW() {
        double robotRotation = 0;
        if(fieldOriented) {
            if (in_navx.isMagnetometerCalibrated()) {
                robotRotation = -(in_navx.getFusedHeading()) * Math.PI / 180 - robotRotationOffset;
            } else {
                robotRotation = in_navx.getYaw() * Math.PI / 180 - robotRotationOffset;
            }
        }
        return StandardizeRAD(robotRotation);
    }

    public Vector2 getVelocity() {
        double yaw = getYAW();
        double anglefrontLeft = GetEncoderAngle(in_frontLeftEncoder, 0) + yaw;
        double anglefrontRight = GetEncoderAngle(in_frontRightEncoder, 1) + yaw;
        double anglebackLeft = GetEncoderAngle(in_backLeftEncoder, 2) + yaw;
        double anglebackRight = GetEncoderAngle(in_backRightEncoder, 3) + yaw;
        
        //velocity is per 100 ms, divided by 10 to convert to seconds
        Vector2 speedFrontLeft = Vector2.fromPolar(anglefrontLeft, out_frontLeftDriveMotor.getSelectedSensorVelocity()/10);
        Vector2 speedFrontRight = Vector2.fromPolar(anglefrontRight, out_frontRightDriveMotor.getSelectedSensorVelocity()/10);
        Vector2 speedBackLeft = Vector2.fromPolar(anglebackLeft, out_backLeftDriveMotor.getSelectedSensorVelocity()/10);
        Vector2 speedBackRight = Vector2.fromPolar(anglebackRight, out_backRightDriveMotor.getSelectedSensorVelocity()/10);
        return new Vector2((speedFrontLeft.x + speedFrontRight.x + speedBackLeft.x + speedBackRight.x)/4*DriveConstants.talon_DriveMotor_SensorToMeters, (speedFrontLeft.y + speedFrontRight.y + speedBackLeft.y + speedBackRight.y)/4*DriveConstants.talon_DriveMotor_SensorToMeters);
    }

    //resets the robot's yaw to a specified angle
    public void setYAW(double yaw) {
        SmartDashboard.putNumber("test", stepTimer.get());
        if (in_navx.isMagnetometerCalibrated()) {
            robotRotationOffset = yaw - (in_navx.getFusedHeading()) * Math.PI / 180;
        } else {
            robotRotationOffset = yaw + in_navx.getYaw() * Math.PI / 180;
        }
        SmartDashboard.putNumber("yaw2", getYAW());
    }

    //overide for SetWheelSpeeds
    public Vector2[] SetWheelSpeeds(double joystickMag, double joystickAngle, double joystickRotation) {    
        return SetWheelSpeeds(joystickMag, joystickAngle, joystickRotation, false);
    }

    //updates the robot's position and rotation using information from the limelight
    public Orientation UpdateOrientation(Vision limelight) {
        StepPosition();
        Orientation visionOrientation = limelight.getOrientation();
        if(visionOrientation != null) {
            position = visionOrientation.position;
            setYAW(visionOrientation.yaw);
        }
        return visionOrientation;
    }
    public void StepPosition() {
        double time = stepTimer.get();
        double delta = time - lastStepTime;
        Vector2 velocity = getVelocity();
        position.x += delta * velocity.x;
        position.y += delta * velocity.y;
        lastStepTime = time;
    }

    public SwerveTiltOutput GetTilt() {
        double yaw = getYAW();
        double roll = in_navx.getRoll() * Math.PI / 180;
        double pitch = in_navx.getPitch() * Math.PI / 180;
        double angle = Math.acos(Math.cos(pitch) * Math.cos(roll));
        
        double dirDiv = Math.cos(roll) * Math.cos(pitch);
        dirDiv = Math.sqrt(1 - dirDiv*dirDiv);
        double dirRight = Math.cos(pitch)*Math.sin(roll)/dirDiv;
        double dirForward = Math.sin(pitch)/dirDiv;
        //double dirAngle = Math.atan2(dirForward, dirRight);

        return new SwerveTiltOutput(angle, new Vector2(dirForward, dirRight));
    }

    public Grid getClosestGrid() {
        int row = Math.max(Math.min((int)Math.round((position.y + FieldConstants.closestGridOffset) / FieldConstants.gridOffset), 1), -1);
        if(row == -1) {
            return Grid.left;
        } else if(row == 0) {
            return Grid.middle;
        } else {
            return Grid.right;
        }
    }
    

    // Update Cargo handler data to dashboard    
    public void updateStatus(){
        SmartDashboard.putString("Robot position", position.toString());
        SmartDashboard.putString("Robot velocity", getVelocity().toString());
        SmartDashboard.putString("Robot tilt direction", GetTilt().tiltDirection.toString(3));
        SmartDashboard.putNumber("Robot tilt", GetTilt().tilt);
    }
}








  

//We can remove the SwerveDriveMath.java code and use the piece below to consolidate file structure

// class SwerveDriveMath {
//     //helper functions
//     private static double rotX(double x, double y, double rot) {
//       double angle = Math.atan(y/x);
//       if(x < 0) {
//         angle += Math.PI;
//       }
//       return Math.cos(angle - rot) * Math.sqrt(x*x + y*y);  
//     }
    
//     private static double rotY(double x, double y, double rot) {
//       double angle = Math.atan(y/x);
//       if(x < 0) {
//         angle += Math.PI;
//       }
//       return Math.sin(angle - rot) * Math.sqrt(x*x + y*y);  
//     }
//     //
  
//     //math
//     public static double maxNonLimitSpeed(double pRADS)//gets the maximum speed that's available at every movement direction (decreases as the rotation speed increases)
//      {
//       return Math.min((2*SwerveDriveMathConstants.maxMotorUse - Math.abs(SwerveDriveMathConstants.hypotenuse * pRADS)) / (2 * SwerveDriveMathConstants.maxMoveSpeed), 1);
//     }
//     private static double velocityMagnitude(double jMAG, double jRAD, double pRADS) {
//       int mod = ((int)Math.ceil(2 * jRAD / Math.PI) % 2);
//       double frac = SwerveDriveMathConstants.robotWidth/SwerveDriveMathConstants.robotLength;
//       if(mod == 0) {
//         frac = 1/frac;
//       }
//       double a = jRAD - Math.atan(frac);
//       double cos = Math.cos(a) * SwerveDriveMathConstants.hypotenuse;
//       double sin = Math.sin(a) * SwerveDriveMathConstants.hypotenuse;
//       if(mod == 0) {
//         double temp = cos;
//         cos = sin;
//         sin = temp;
//       }
//       return Math.min((-Math.abs(cos * pRADS) + Math.sqrt(-sin*sin*pRADS*pRADS + 4 * SwerveDriveMathConstants.maxMotorUse*SwerveDriveMathConstants.maxMotorUse)) / 2, jMAG * SwerveDriveMathConstants.maxMoveSpeed);
//     }
//     //  
//   public static Vector2[] WheelSpeeds(
//     double joystickMag,//(0-1) //magnitude of move joystick
//     double joystickAngle,//(0-2pi) //angle of move joystick
//     double joystickRotation,//(-1) to 1 //value of turn joystick
//     double robotRotation//0-2pi //robot's rotation
//     ) {
//     //math
//     ////
//     double joystickTrue = joystickAngle + robotRotation;
//     double pRADS = joystickRotation * SwerveDriveMathConstants.maxRotationSpeed;
//     //
//     double robotSpeed = velocityMagnitude(joystickMag,  joystickTrue, pRADS);
//     double robotSpeedX = Math.cos(joystickTrue) * robotSpeed;
//     double robotSpeedY = Math.sin(joystickTrue) * robotSpeed;
//     ////
//     double w = pRADS*SwerveDriveMathConstants.robotWidth / 2;
//     double h = pRADS*SwerveDriveMathConstants.robotLength / 2;
//     //
//     Vector2 frontLeft = new Vector2(robotSpeedX + h, robotSpeedY + w);
//     Vector2 frontRight = new Vector2(robotSpeedX + h, robotSpeedY - w);
//     Vector2 backLeft = new Vector2(robotSpeedX - h, robotSpeedY + w);
//     Vector2 backRight = new Vector2(robotSpeedX - h, robotSpeedY - w);
//     //
//     return new Vector2[] {frontLeft, frontRight, backLeft, backRight};
//   }
  
// }
  