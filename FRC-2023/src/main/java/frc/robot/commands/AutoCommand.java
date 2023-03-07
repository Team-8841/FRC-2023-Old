package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants.AutoPaths;
import frc.robot.helpers.AutoOutput;
import frc.robot.helpers.Vector2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

//moves robot along curve specified by pathplanner
public class AutoCommand extends CommandBase{
    SwerveDrive swerveDrive;
    Vision limelight;
    public Timer autoTimer;
    public AutoCommand(SwerveDrive subsystem, Vision limelight, AutoPaths path) {
      swerveDrive = subsystem;
      this.limelight = limelight;
      addRequirements(swerveDrive);
      addRequirements(limelight);
      autoTimer = new Timer();
      //sets the path that Autos will use
      Autos.SetPath(path);
    }

    @Override
    public void initialize() {
      autoTimer.reset();
      autoTimer.start();
    }


    //sets wheel speeds to values calculated by Autos.java
    @Override
    public void execute() {
      Autos.UpdatePID();
      swerveDrive.UpdateOrientation(limelight);
      AutoOutput autoOutput = Autos.GetSpeed(autoTimer.get(), swerveDrive.position, swerveDrive.getYAW());
      Vector2 targSpeed = autoOutput.velocity;
      double targetRotSpeed = autoOutput.rotationSpeed/50;
      double targetSpeed = targSpeed.getMagnitude()/50;
      
      swerveDrive.SetWheelSpeeds(targetSpeed, targSpeed.getAngle(), targetRotSpeed, true);
    }


    @Override
    public void end(boolean interrupted) {
    }


    @Override
    public boolean isFinished() {
      return autoTimer.get() > Autos.GetPath().getTotalTimeSeconds();
    }
  }