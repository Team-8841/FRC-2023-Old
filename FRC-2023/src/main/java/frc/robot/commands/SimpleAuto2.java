package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class SimpleAuto2 extends SequentialCommandGroup {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    public SimpleAuto2(SwerveDrive swerveDrive, Vision vision) {
        super(new SimpleAuto2P1(swerveDrive), 
        new SimpleAuto2P2(swerveDrive), 
        new SimpleAuto2P3(swerveDrive), 
        new SimpleAuto2P35(swerveDrive), 
        new SimpleAuto2P4(swerveDrive), 
        new BalanceCommand(swerveDrive, vision));
    }
}
class SimpleAuto2P1 extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;

    public SimpleAuto2P1(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.SetWheelSpeeds(Constants.AutoConstants.simpleAutoTranslationSpeed, -swerveDrive.getYAW() + Math.PI/2, 0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        var tilt = swerveDrive.GetTilt();
      return tilt.tilt*180/Math.PI > 4;
    }
}
class SimpleAuto2P2 extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;

    public SimpleAuto2P2(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.SetWheelSpeeds(Constants.AutoConstants.simpleAutoTranslationSpeedOnBalance, -swerveDrive.getYAW() + Math.PI/2, 0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        var tilt = swerveDrive.GetTilt();
      return tilt.tilt*180/Math.PI > 10 && tilt.tiltDirection.x<=-0.9;
    }
}
class SimpleAuto2P3 extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;

    public SimpleAuto2P3(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.SetWheelSpeeds(Constants.AutoConstants.simpleAutoTranslationSpeedOnBalance, -swerveDrive.getYAW() + Math.PI/2, 0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        var tilt = swerveDrive.GetTilt();
      return tilt.tilt*180/Math.PI < 3;
    }
}

class SimpleAuto2P35 extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;
    Timer time = new Timer();

    public SimpleAuto2P35(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        time.start();
        addRequirements(swerveDrive);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(time.get() < 1.5) {
        swerveDrive.SetWheelSpeeds(Constants.AutoConstants.simpleAutoTranslationSpeed, -swerveDrive.getYAW() + Math.PI/2, 0);
        } else {
            swerveDrive.SetWheelSpeeds(0, -swerveDrive.getYAW() + Math.PI/2, 0);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return time.get() > 1.5+3;
    }
}

class SimpleAuto2P4 extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;

    public SimpleAuto2P4(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        
        addRequirements(swerveDrive);
        swerveDrive.Start();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        swerveDrive.SetWheelSpeeds(Constants.AutoConstants.simpleAutoTranslationSpeed, -swerveDrive.getYAW() - Math.PI/2, 0);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        var tilt = swerveDrive.GetTilt();
      return tilt.tilt*180/Math.PI > 4;
    }
}
//probably how you have to call this
//swerveDrive.setDefaultCommand(new SequentialCommandGroup(new SimpleAuto(swerveDrive), new BalanceCommand(swerveDrive, vision)));