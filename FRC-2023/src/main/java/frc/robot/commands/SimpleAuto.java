package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class SimpleAuto extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;

    public SimpleAuto(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putNumber("Testing", Math.random());
        swerveDrive.SetWheelSpeeds(0.25, -swerveDrive.getYAW() - Math.PI/2, 0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return swerveDrive.GetTilt().tilt*180/Math.PI > 4;
    }
}
//probably how you have to call this
//swerveDrive.setDefaultCommand(new SequentialCommandGroup(new SimpleAuto(swerveDrive), new BalanceCommand(swerveDrive, vision)));