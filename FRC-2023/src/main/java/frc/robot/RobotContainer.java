// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.AutoConstants.AutoPaths;
import frc.robot.Constants.CargoHandlerConstants;
import frc.robot.Constants.DSConstants;
import frc.robot.Constants.DSConstants.DSPorts;
import frc.robot.Constants.DSConstants.GPPorts;
import frc.robot.Constants.FieldConstants.GoalColumn;
import frc.robot.Constants.FieldConstants.GoalRow;
import frc.robot.commands.ArmGotoPosition;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SimpleAuto2;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.CargoHandler;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

//L class
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive swerveDrive = new SwerveDrive();
  private final CargoHandler cargoHandler = new CargoHandler();
  private final Arm arm = new Arm();
  // TODO: get lighting working
  // private final Lighting lighting = new Lighting();
  private final Vision vision = new Vision();

  private static final Joystick copilotDS = new Joystick(DSPorts.copilotDSPort);
  private static final CommandXboxController gamepad = new CommandXboxController(GPPorts.controllerPort);

  private final Compressor out_compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  public boolean autoBalancing = false;
  public boolean autoBalanceSet = false;
  
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Enables the compressor in digital mode using the digital pressure switch.
    if (copilotDS.getRawButton(DSPorts.compressor)) {
      out_compressor.enableDigital();
    } else {
      out_compressor.disable();
    }
    // Swerve Drive Default Command
    if(swerveDrive.getCurrentCommand() != null) {
      swerveDrive.getCurrentCommand().cancel();
    }
    autoBalancing=false;
    swerveDrive.setDefaultCommand(new DriveCommand(swerveDrive, vision, gamepad));
    
    // swerveDrive.setDefaultCommand(new BalanceCommand(swerveDrive, vision));
    // Cargo Handler Default Command
    cargoHandler.setDefaultCommand(new RunCommand(() -> {
      cargoHandler.UpdatePID();
      cargoHandler.updateIntakeState(copilotDS.getRawButton(DSPorts.intakeOUT),
          copilotDS.getRawButton(DSPorts.intakeIN));
    }, cargoHandler));

    // Arm Default Command
    arm.setDefaultCommand(new ArmGotoPosition(arm, ArmPosition.home));

    // Lighting Default Command //TODO: make lighting command
    /*
     * lighting.setDefaultCommand(new RunCommand(() -> {
     * 
     * }, lighting));
     */

    // Vision Default Command
    vision.setDefaultCommand(new RunCommand(() -> {
      vision.updateStatus();
    }, vision));
    Autos.Start();
  }
    public void periodic() {
      //I'm sorry Brian
      if(gamepad.start().getAsBoolean()) {
      swerveDrive.setYAW(0); 
      }
      SmartDashboard.putBoolean("AFIPOANF", gamepad.leftBumper().getAsBoolean());
      if(gamepad.leftBumper().getAsBoolean()) {
        
       // Toggle auto balance
       if(!autoBalanceSet) {
      if(autoBalancing) {
        swerveDrive.getCurrentCommand().cancel();
        swerveDrive.setDefaultCommand(new DriveCommand(swerveDrive, vision, gamepad));
        SmartDashboard.putNumber("Testing2", 0);
      } else {
        SmartDashboard.putNumber("Testing2", 1);
        swerveDrive.getCurrentCommand().cancel();
        swerveDrive.setDefaultCommand(new SimpleAuto2(swerveDrive, vision));
      }
      autoBalancing=!autoBalancing;
      autoBalanceSet=true;
      SmartDashboard.putBoolean("Auto Balancing", autoBalancing);
    }
      } else {
      autoBalanceSet=false;
    }
    }
  private void configureBindings(){

    // Configure any controller button functions here
    /*
     * Follow this format to configure a button with the
     * gamepad/commandxboxcontroller
     * m_driverController.buttonname.onTrue(new RunCommand(() -> { function to run
     * }));
     * .onFalse
     * .whileTrue
     * .whileFalse
     * .toggleOnTrue
     * .toggleOnFalse
     * 
     * 
     * Follow this format for the copilot buttons/switches
     * 
     * new JoystickButton(m_copilotDS, constantvar).onTrue(new RunCommand(() -> {
     * function to run }));
     * .onFalse
     * .whileTrue
     * .whileFalse
     * .toggleOnTrue
     * .toggleOnFalse
     * 
     * 
     * What each method does
     * We will mainly only use the first 3
     * 
     * onTrue - Runs the command once when the button is pressed
     * onFalse - Runs the command once when the button is released
     * whileTrue - Runs the command continuously until the button is released
     * whileFalse - Runs the command continuously until the button is pressed (we
     * shouldnt ever use this)
     * toggleOnTrue - Toggles the command when the button is pressed (not sure what
     * the use case is for this)
     * toggleOnFalse - Toggles the command when the button is released (not sure
     * what the use case is for this)
     */

    // Reset robot rotation when you press start
    /*
     * gamepad.start().onTrue(new RunCommand(() -> {
     * swerveDrive.setYAW(0);
     * }));
     */

    // Toggle the camera drive mode when you press Y
    gamepad.y().onTrue(new RunCommand(() -> {
      vision.toggleDriveMode();
    }));

    // Turn off The compressor while the switch is active;
    new JoystickButton(copilotDS, DSPorts.compressor).whileTrue(new RunCommand(() -> {
      out_compressor.disable();
    }));

    // Turn on the compressor while the switch is inactive
    new JoystickButton(copilotDS, DSPorts.compressor).whileFalse(new RunCommand(() -> {
      out_compressor.enableDigital();
    }));

    // Turn the turntable clockwise while the switch is active
    new JoystickButton(copilotDS, DSPorts.turnTableClockwise).whileTrue(new RunCommand(() -> {
      cargoHandler.setTurntableSpeed(CargoHandlerConstants.turnTableSpeed);
    }));

    // Turn the turntable counter-clockwise while the switch is active
    new JoystickButton(copilotDS, DSPorts.turnTableCounterClockwise).whileTrue(new RunCommand(() -> {
      cargoHandler.setTurntableSpeed(-CargoHandlerConstants.turnTableSpeed);
    }));

    // Run functions for manual arm mode
    new JoystickButton(copilotDS, DSPorts.manualArmMode).whileTrue(new RunCommand(() -> {
      if (copilotDS.getRawButton(DSPorts.gripperOn)) {
        arm.setGripperSpeed(ArmConstants.gripperMotorSpeed);
      } else {
        arm.setGripperSpeed(0);
      }
      arm.setArmExtension(copilotDS.getRawButton(DSPorts.armExtension));
      arm.setArmSolenoid(copilotDS.getRawButton(DSPorts.armPivot));
      arm.setArmMotors(getArmSpeed());
    }));

    // starts score command sequence
    gamepad.rightBumper().whileTrue(new RunCommand(() -> {
      if (copilotDS.getRawButton(DSPorts.manualArmMode)) {
        arm.setGripperSpeed(ArmConstants.scoreGripperMotorSpeed);
      } else {
        arm.setGripperSpeed(0);
        // new RunCommand( () -> {
        // ScoreCommandSequence(swerveDrive, vision, arm, getGoalColumn(), getGoalRow(),
        // swerveDrive.getClosestGrid());
        // });
      }
    }));

    // Move the intake in/out/off
    cargoHandler.updateIntakeState(copilotDS.getRawButton(DSPorts.intakeIN), copilotDS.getRawButton(DSPorts.intakeOUT));

  }

  public Command getAutonomousCommand() {
    return new AutoCommand(swerveDrive, vision, getAutonomousPath());
  }

  // Gets the selected scoring column
  public static GoalColumn getGoalColumn() {
    if (copilotDS.getRawButton(DSPorts.goalLeft)) {
      return GoalColumn.left;
    } else if (copilotDS.getRawButton(DSPorts.goalCenter)) {
      return GoalColumn.center;
    } else if (copilotDS.getRawButton(DSPorts.goalRight)) {
      return GoalColumn.right;
    } else {
      return GoalColumn.none;
    }
  }

  // Gets the selected scoring row
  public static GoalRow getGoalRow() {
    if (copilotDS.getRawButton(DSPorts.goalTop)) {
      return GoalRow.top;
    } else if (copilotDS.getRawButton(DSPorts.goalBottom)) {
      return GoalRow.bottom;
    } else {
      return GoalRow.middle;
    }
  }

  private double getArmSpeed() {
    SmartDashboard.putNumber("Arm Joystick", copilotDS.getRawAxis(DSPorts.arm));
    if (copilotDS.getRawAxis(DSPorts.arm) < 0.03) {
      return 1;
    } else if (copilotDS.getRawAxis(DSPorts.arm) > .1) {
      return -0.3;
    } else {
      return 0.0;
    }
  }

  // Gets the selected autonomous path
  public static AutoPaths getAutonomousPath() {
    double angle = copilotDS.getRawAxis(DSPorts.auto);
    AutoPaths path = AutoPaths.defaultPath;
    double currentDif = DSConstants.autoKnobMoE;
    for (var i : AutoPaths.values()) {
      double angleDif = Math.abs(angle - DSConstants.autoKnobAngles[i.value]);
      if (angleDif < currentDif) {
        currentDif = angleDif;
        path = i;
      }
    }
    return path;
  }
}
