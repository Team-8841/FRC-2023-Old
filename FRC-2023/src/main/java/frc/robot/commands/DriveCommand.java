package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DSConstants.GPPorts;
import frc.robot.helpers.AutoOutput;
import frc.robot.helpers.Vector2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

//reads joystick inputs to determine swerve drive wheel speeds
public class DriveCommand extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;
    public Vision limelight;

    private CommandXboxController controller;

    public DriveCommand(SwerveDrive swerveDrive, Vision limelight, CommandXboxController gamepad) {
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;

        this.controller = gamepad;
        addRequirements(swerveDrive);
        swerveDrive.Start();
        Autos.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.UpdateOrientation(limelight);
        Vector2 joystickVector = new Vector2(controller.getRawAxis(GPPorts.axisLX), -controller.getRawAxis(GPPorts.axisLY));
        double rotationSpeed = controller.getRawAxis(GPPorts.axisRX);
        if(Math.abs(rotationSpeed) < 0.05) {
          rotationSpeed = 0;
        }
        //
        if(!controller.povCenter().getAsBoolean()) {
          rotationSpeed=0;
          double targAngle = Math.PI/180;
          
          if(controller.povUp().getAsBoolean()) {
            targAngle*=0;
          } else if(controller.povDown().getAsBoolean()) {
            targAngle*=180;
          } else if(controller.povRight().getAsBoolean()) {
            targAngle*=90;
          } else if(controller.povLeft().getAsBoolean()) {
            targAngle*=-90;
          } 
          else if(controller.povUpRight().getAsBoolean()) {
            targAngle*=45;
          } else if(controller.povUpLeft().getAsBoolean()) {
            targAngle*=-45;
          } else if(controller.povDownRight().getAsBoolean()) {
            targAngle*=180-45;
          } else if(controller.povDownLeft().getAsBoolean()) {
            targAngle*=180+45-360;
          }
          Autos.UpdatePID();
          SmartDashboard.putNumber("TArg Test ", targAngle * 180 / Math.PI);
          AutoOutput output = Autos.GetSpeed(new Vector2(), swerveDrive.getYAW(), new Vector2(), new Vector2(), targAngle, 0);
          SmartDashboard.putNumber("TArg Rot", output.rotationSpeed);
          rotationSpeed = output.rotationSpeed*0.2;
        }
        //
        double joyMag = joystickVector.getMagnitude();
        joyMag = Math.max((joyMag-Constants.DriveConstants.driveDeadband)/(1-Constants.DriveConstants.driveDeadband), 0);

        double curvedMag = ((7*joyMag*joyMag*joyMag) + Math.sqrt(joyMag))/8; //todo: make better curves :(
        double absRotation = Math.abs(rotationSpeed);
        double newRotationSpeed = ((3*absRotation*absRotation*absRotation+1)/4*Math.sqrt(absRotation)) * Math.signum(rotationSpeed); 
        if(controller.rightBumper().getAsBoolean()) {
          curvedMag*=Constants.DriveConstants.manualSlowMult;
          newRotationSpeed*=Constants.DriveConstants.manualSlowTurnMult;
        }
        Vector2[] wheelSpeeds = swerveDrive.SetWheelSpeeds(curvedMag, joystickVector.getAngle(), newRotationSpeed);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
}
  
