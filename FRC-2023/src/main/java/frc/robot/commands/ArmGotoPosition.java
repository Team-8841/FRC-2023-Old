package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.subsystems.Arm;

//moves arm to specified location
public class ArmGotoPosition extends CommandBase {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public Arm arm;
    public ArmPosition position;
    public ArmGotoPosition(Arm arm, ArmPosition position) {
        this.arm = arm;
        this.position = position;
        addRequirements(arm);
        arm.setArmSolenoid(position.solenoid);
        if(!position.extension) {
          arm.setArmExtension(false);
        }
      // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //add pid code to set arm position
        arm.setArmAngle(position.angle);
        if(position.extension && arm.getArmAngle() > ArmConstants.extensionAngle) {
            arm.setArmExtension(true);
        }
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
