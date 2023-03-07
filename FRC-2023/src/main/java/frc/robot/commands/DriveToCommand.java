package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.helpers.AutoOutput;
import frc.robot.helpers.Vector2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

//uses autonomous position correction to move robot to specified location
public class DriveToCommand extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;
    public Vision limelight;
    public Vector2 position;
    public double yaw;

    public DriveToCommand(SwerveDrive swerveDrive, Vision limelight, Vector2 position, double yaw) {
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        this.position = position;
        this.yaw = yaw;
        addRequirements(swerveDrive);
        addRequirements(limelight);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.UpdateOrientation(limelight);
        AutoOutput autoOutput = Autos.GetSpeed(swerveDrive.position, swerveDrive.getYAW(), position, new Vector2(), yaw, 0);
        Vector2 targSpeed = autoOutput.velocity;
        double targeRotSpeed = autoOutput.rotationSpeed/50;
        double targetSpeed = targSpeed.getMagnitude()/50;

        swerveDrive.SetWheelSpeeds(targetSpeed, targSpeed.getAngle(), targeRotSpeed, true);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    

  
    // Returns true when the command should end.
    //ends command when close enough to target location
    @Override
    public boolean isFinished() {
      return new Vector2(position.x - swerveDrive.position.x, position.y - swerveDrive.position.y).getMagnitude() < DriveConstants.driveToPositionMoE && swerveDrive.getVelocity().getMagnitude() < DriveConstants.driveToPositionMaxEndSpeed;
    }
}
  