package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.DSConstants.GPPorts;
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
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.UpdateOrientation(limelight);
        Vector2 joystickVector = new Vector2(controller.getRawAxis(GPPorts.axisLX), -controller.getRawAxis(GPPorts.axisLY));
        double rotationSpeed = controller.getRawAxis(GPPorts.axisRX);
  
        if(Math.abs(rotationSpeed) < 0.1) {
          rotationSpeed = 0;
        }
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
  