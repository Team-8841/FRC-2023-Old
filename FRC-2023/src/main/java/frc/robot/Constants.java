// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathConstraints;

import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.helpers.Vector2;

public final class Constants {

  public static class AutoConstants {
    // D: amount dropped(doesn't include preload)
    // P: amount picked up(doesn't include preload)
    // C: parking on charging station
    // T: start near loading zone
    // B: start away from loading zone

    public enum AutoPaths {
      D0_P1_C_B(0),
      D1_P1_C_B(1),
      D1_P1_T(2),
      D1_P2_C_B(3),
      D2_P2_B(4),
      D2_P2_C_B(5),
      D2_P2_T(6),
      D2_P3_C_T(7),
      Make_Brian_Sad(8);

      private AutoPaths(int value) {
        this.value = value;
      }

      public final int value;
      public static final AutoPaths defaultPath = Make_Brian_Sad;
    }

    public static String[] pathNames = new String[] {
        "D0 P1 C B",
        "D1 P1 C B",
        "D1 P1 T",
        "D1 P2 C B",
        "D2 P2 B",
        "D2 P2 C B",
        "D2 P2 T",
        "D2 P3 C T",
        "Make Brian sad"
    };

    public static PathConstraints[] autoPathContraints = new PathConstraints[] {
        // TODO: update maxVelocity and maxAcceleration for each path
        // (x: maxVelocity, y: maxAcceleration)
        new PathConstraints(4, 3), // D0_P1_C_B
        new PathConstraints(4, 3), // D1_P1_C_B
        new PathConstraints(4, 3), // D1_P1_T
        new PathConstraints(4, 3), // D1_P2_C_B
        new PathConstraints(4, 3), // D2_P2_B
        new PathConstraints(4, 3), // D2_P2_C_B
        new PathConstraints(4, 3), // D2_P2_T
        new PathConstraints(4, 3), // D2_P3_C_T
        new PathConstraints(4, 3) // Make Brian Sad
    };

    public static double autoCorrectionPID_P = 0.5;
    public static double autoCorrectionPID_I = 0;
    public static double autoCorrectionPID_D = 0;
    public static double autoCorrection_MaxMeters = 1;

    public static double autoRotationCorrectionPID_P = 0.5;
    public static double autoRotationCorrectionPID_I = 0;
    public static double autoRotationCorrectionPID_D = 0;
  }

  public static class DriveConstants {

    public static final int[] swerveMotorTurnPorts = { 1, 4, 7, 10 };
    public static final int[] swerveMotorDrivePorts = { 2, 5, 8, 11 };
    public static final int[] swerveEncoderPorts = { 3, 6, 9, 12 };

    public static final double driveMotorRampTime = 0.1; // Drivetrain open loop ramp time

    public static final double[] turnAngleOffsets = { 150+90, 42+90, 99+90, 195+90-180}; // degrees
    public static final boolean[] turnEncoderInversed = { false, false, false, false };

    public static final double turnPID_P = 0.5;
    public static final double turnPID_I = 0;
    public static final double turnPID_D = 0;
    public static final double speedPID_P = 0.5;
    public static final double speedPID_I = 0;
    public static final double speedPID_D = 0;

    public static final double driveDeadband = 0.03;

    public static final double driveToPositionMoE = 0.254;
    public static final double driveToPositionMaxEndSpeed = 0.1;

    // meters
    private static final double wheelRadius = 0.0460375;
    private static final double talon_DriveMotor_SpeedCoef = 1D / 13871;
    public static final double talon_DriveMotor_SensorToMeters = 2 * Math.PI * wheelRadius * talon_DriveMotor_SpeedCoef;

    public static final double manualSlowMult = 0.2;
    public static final double manualSlowTurnMult = 0.5;
  }

  public static class SwerveDriveMathConstants {
    // not all of these are constant but mentors wanted it in here

    public static final double robotWidth = 0.631825; // distance between front motors
    public static final double robotLength = 0.631825; // distance between front and back motors
    ////
    public static final double maxMotorUse_default = 1;// 0-1 //maximum usage of all motors

    // Lester: maxMoveSpeed_default: 1, maxRotationSlider_default: 0.77
    public static final double maxRotationSlider_default = 0.77;// 0-1 //max rotation motor usage
    public static final double maxMoveSpeed_default = 1;// 0-1 //max move motor usage
    //
    public static double maxMotorUse = maxMotorUse_default;
    public static double maxRotationSlider = maxRotationSlider_default;
    public static double maxMoveSpeed = maxMoveSpeed_default;
    ////

    // don't worry about it :)
    public static final double hypotenuse = Math.sqrt(robotLength * robotLength + robotWidth * robotWidth);

    public static double maxRotationSpeed = maxRotationSlider * (2 * maxMotorUse) / hypotenuse;
    public static double maxNonLimitRotation = 2 * (maxMotorUse - maxMoveSpeed) / (maxRotationSpeed * hypotenuse);

    public static void updateMax() {
      maxRotationSpeed = maxRotationSlider * (2 * maxMotorUse) / hypotenuse;
      maxNonLimitRotation = 2 * (maxMotorUse - maxMoveSpeed) / (maxRotationSpeed * hypotenuse);
    }
    //
  }

  public static class CargoHandlerConstants {

    public static final int horizontalMotorID = 13;
    public static final int verticalMotorID = 14;
    public static final int verticalDIOEncoderID = 0;
    public static final int gripperMotorID = 15;
    public static final int turnTableMotorID = 19;

    public static final int agitatorSolenoidID = 2;

    public static final int colorSensorID = 3;
    public static final int horizontalWheelSensor = 4;

    public static final int currentLimit = 80;

    public static final double verticalIntakeInSpeed = 0.3; // 0-1
    public static final double verticalIntakeOutSpeed = 0.3; // 0-1

    public static final double horizontalIntakeInSpeed = 1; // 0-1
    public static final double horizontalIntakeOutSpeed = 1; // 0-1

      public static final double verticalIntakePositionPID_P = 0.5;
      public static final double verticalIntakePositionPID_I = 0;
      public static final double verticalIntakePositionPID_D = 0;
      public static final double verticalIntakePositionTarget = 310;//TODO: set this to required angle
      public static final double verticalIntakePositionAtTargetMoE = 5;//TODO: set this to required angle

    public static final int verticalIntakeEncoderID = 16; // TODO: update this

    public static final double turnTableSpeed = 1;
  }

    public static class ArmConstants {

    public static final int rightArmMotorID = 16;
    public static final int leftArmMotorID = 17;
    public static final int gripperMotorID = 18;

    public static final int armSolenoidID = 0;
    public static final int gripperSolenoidID = 1;

    public static final int currentLimit = 60;

    public static final double gripperMotorSpeed = 0.5;
    public static final double scoreGripperMotorSpeed = -0.3;

    public static final double manualArmMotorSpeed = 0.3;

    public static final double cubeProximityLimit = 152.4; // arm.getProx in set gripper command. Used to stop the motor
                                                           // 6in before it hits the end. Purple cube.

    public static final double gripperTimeout = 1;

    public enum ArmPosition {// TODO: set angles
      home(false, false, 0),
      floor(false, true, 270), // setting extension to false for safety (for now)
      middle(false, true, 270),
      top(false, true, 270);

      public boolean extension;
      public boolean solenoid;
      public double angle;

      private ArmPosition(boolean extension, boolean solenoid, double angle) {
        this.extension = extension;
        this.solenoid = solenoid;
        this.angle = angle;
      }
    }

    public static final double extensionAngle = 180;

    public static final double anglePID_P = 0.5;
    public static final double anglePID_I = 0;
    public static final double anglePID_D = 0;
  }

  public static class VisionConstants {

    // All height measurements are in inches
    public static final double heightOfCamera = 43; // Height of camera from floor
    public static final double heightOfTarget = 29; // Height of our target
    public static final double angleOfCamera = -20; // Angle the camera is mounted at

  }

  public static class DSConstants {

    public enum IntakeState {
      in,
      out,
      off
    };

    // driver station knob positions for each auto mode
    public static final double[] autoKnobAngles = new double[] { // TODO: configure
        0, // D0 P1 C B
        0, // D1 P1 C B
        0, // D1 P1 T
        0, // D1 P2 C B
        0, // D2 P2 B
        0, // D2 P2 C B
        0, // D2 P2 T
        0, // D2 P3 C T
        0// Make Brian sad
    };

    // driver station knob position margin of error, allows some wiggle room for the
    // knob positions set for each auto
    public static final double autoKnobMoE = 1;// TODO: configure

    // Driver station input ports
    public static final class DSPorts {
      public static final int copilotDSPort = 1; // DS port

      // Driver Station Knobs (Analog Inputs)
      public static final int auto = 0; // Autonomous Mode Select Knob
      public static final int arm = 1; // Manual Arm Control Joystick Y-axis

      // Driver Station Buttons (Digital Inputs)
      public static final int compressor = 2; // Compressor Override
      public static final int manualArmMode = 1; // Manual Arm Mode Enable
      public static final int intakeIN = 3; // Intake IN/OFF Switch
      public static final int intakeOUT = 4; // Intake OUT Momentary Button
      // public static final int hip = 6; //Hip FWD/BACK Switch
      public static final int goalLeft = 10; // Left Column Button
      public static final int goalCenter = 11; // Center Column Button
      public static final int goalRight = 12; // Right Column Button
      public static final int goalTop = 14; // Top Row Switch Position
      public static final int goalBottom = 13; // Bottom Row Switch Position
      public static final int turnTableClockwise = 5; // Turn Table on Switch Position - its the backwards one
      public static final int turnTableCounterClockwise = 6; // Turn Table off Switch Position - its the forward one
      public static final int gripperOn = 9; // Gripper on Switch Position
      public static final int armExtension = 7; // Arm extend button
      public static final int armPivot = 8; // This may be hip
    }

    // Gamepad input ports
    public static final class GPPorts {
      public static final int controllerPort = 0; // Gamepad

      // Gamepad Buttons (Digital Inputs)
      public static final int buttonA = 1; // NOT USED
      public static final int buttonB = 2; // NOT USED
      public static final int buttonX = 3; // NOT USED
      public static final int buttonY = 4; // Limelight Toggle Button
      public static final int buttonLB = 5; // Gripper Toggle Button
      public static final int buttonRB = 6; // Score Button
      public static final int buttonSelect = 7; // NOT USED
      public static final int buttonStart = 8; // Reset Sensors

      // Gamepad (Analog Inputs)
      public static final int axisLX = 0;
      public static final int axisLY = 1;
      public static final int triggerL = 2; // NOT USED
      public static final int triggerR = 3; // NOT USED
      public static final int axisRX = 4;
      public static final int axisRY = 5;
    }
  }

  public static class FieldConstants {

    public enum GoalColumn {
      left(goalColumnOffset),
      center(0),
      right(-goalColumnOffset),
      none(0);

      private GoalColumn(double offset) {
        this.offset = offset;
      }

      public double offset;
    };

    public enum GoalRow {
      bottom(0.5, 0.38, ArmPosition.floor),
      middle(0.45, 0.381, ArmPosition.middle),
      top(0.381, 0.381, ArmPosition.top);

      private GoalRow(double initialXDistance, double finalXDistance, ArmPosition armPosition) {
        this.initialXDistance = initialXDistance;
        this.finalXDistance = finalXDistance;
        this.armPosition = armPosition;
      }

      // TODO: tune distances
      public double initialXDistance;
      public double finalXDistance;
      public ArmPosition armPosition;
    };

    public enum Grid {
      left(-gridOffset),
      middle(0),
      right(gridOffset);

      private Grid(double offset) {
        this.offset = offset;
      }

      public double offset;
    }

    // distance between the center of each grid
    public static final double gridOffset = 1.6764;
    // distance between the center of each grid column
    public static final double goalColumnOffset = 0.5588;
    // position of the center grid
    public static final Vector2 gridPosition = new Vector2(6.892544, -1.285875);

    public static final double closestGridOffset = 1.285875;
  }

  public static final class BalanceConstants {
    public static final double rampWidth = 1.2192;
    public static final double robotDisplacementDropOffStart = 0.6096;
    public static final double rampXPos = 4.33197;
    public static final double robotWidth = 0.762;// don't take into account bumpers

    public static final double velocityPID_P = 0.5*1.4;
    public static final double velocityPID_I = 0;
    public static final double velocityPID_D = 0;
  }
}
