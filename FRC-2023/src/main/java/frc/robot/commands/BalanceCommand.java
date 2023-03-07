package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.BalanceConstants;
import frc.robot.helpers.AutoOutput;
import frc.robot.helpers.SwerveTiltOutput;
import frc.robot.helpers.Vector2;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

public class BalanceCommand extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;
    public PIDController velocityPID;
    public double robotRotation;
    public Vision limelight;

    public BalanceCommand(SwerveDrive swerveDrive, Vision limelight) {
        this.swerveDrive = swerveDrive;
        this.limelight = limelight;
        addRequirements(swerveDrive);
        addRequirements(limelight);
        swerveDrive.Start();
        robotRotation = swerveDrive.getYAW();
        velocityPID = new PIDController(BalanceConstants.velocityPID_P, BalanceConstants.velocityPID_I, BalanceConstants.velocityPID_D);
    }
    
    public void UpdatePID() {
        double velocityP = SmartDashboard.getNumber("balance velocity PID P", BalanceConstants.velocityPID_P);
        double velocityI = SmartDashboard.getNumber("balance velocity PID I", BalanceConstants.velocityPID_I);
        double velocityD = SmartDashboard.getNumber("balance velocity PID D", BalanceConstants.velocityPID_D);
        if(velocityP != velocityPID.getP() || velocityI != velocityPID.getI() || velocityD != velocityPID.getD()) {
            velocityPID.setP(velocityP);
            velocityPID.setI(velocityI);
            velocityPID.setD(velocityD);
        }
        }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        UpdatePID();
        swerveDrive.UpdateOrientation(limelight);
        SwerveTiltOutput tiltOutput = swerveDrive.GetTilt();
        double displacementX = swerveDrive.position.x - BalanceConstants.rampXPos * (DriverStation.getAlliance() == Alliance.Red ? -1 : 1);
        double maxDisplacement = getMaxDisplacement(tiltOutput.tilt);
        double dif = maxDisplacement - BalanceConstants.robotDisplacementDropOffStart;
        double targSpeed = Math.abs(velocityPID.calculate(tiltOutput.tilt));
        /*if(tiltOutput.tiltDirection.x * targSpeed > 0) {
            double speedMult = Math.min(Math.max(maxDisplacement/dif - Math.abs(displacementX/dif), 0), 1);
            targSpeed *= speedMult;
        }*/
        Vector2 velocity = new Vector2(targSpeed * tiltOutput.tiltDirection.x, targSpeed * tiltOutput.tiltDirection.y);
        
        AutoOutput autoOutput = Autos.GetSpeed(swerveDrive.position, swerveDrive.getYAW(), swerveDrive.position, velocity, robotRotation, 0);
        Vector2 targSpeed2 = autoOutput.velocity;
        targSpeed2.y*=-1;
        double targetRotSpeed2 = autoOutput.rotationSpeed/50;
        double targetSpeed2 = targSpeed2.getMagnitude()*1;
        SmartDashboard.putNumber("Testing 3", targetSpeed2);
        swerveDrive.SetWheelSpeeds(targetSpeed2, targSpeed2.getAngle()-swerveDrive.getYAW()+Math.PI/2, targetRotSpeed2, false);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    

  
    // Returns true when the command should end.
    //ends command when close enough to target location
    @Override
    public boolean isFinished() {
        return false;
    }  
    public double getMaxDisplacement(double tilt) {
        return Math.cos(tilt) * (BalanceConstants.rampWidth - BalanceConstants.robotWidth) / 2;
    }
}
