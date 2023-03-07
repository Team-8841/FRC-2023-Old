package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.Arm;

//sets gripper to closed or opened
public class SetGripperCommand extends CommandBase {
      
        @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

        public Arm arm ;
        public boolean closed;
        public boolean in;
        public Timer timeout;
        public double timeoutTime = ArmConstants.gripperTimeout;
        public SetGripperCommand (Arm arm, boolean closed, boolean in) {
          this(arm, closed, in, ArmConstants.gripperTimeout);
        }
        public SetGripperCommand (Arm arm, boolean closed, boolean in, double timeoutTime) {
          this.closed = closed;
          this.in = in;
           this.arm = arm; 
            addRequirements(arm);
          arm.setArmExtension(closed);
          timeout.start();
          this.timeoutTime = timeoutTime;
        }


    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(in) {
          if(arm.hasCube()) {
            if(arm.getProx() <= ArmConstants.cubeProximityLimit) {
              arm.setGripperSpeed(0);
            } else {
              arm.setGripperSpeed(ArmConstants.gripperMotorSpeed);
            }
          } else {
            if(arm.getProx() <= 0) {
              arm.setGripperSpeed(0);
            } else {
              arm.setGripperSpeed(ArmConstants.gripperMotorSpeed);
            }
          }
        } else {
          arm.setGripperSpeed(-ArmConstants.gripperMotorSpeed);
        }
    }


    @Override
    public void end(boolean interrupted) {
    super.end(interrupted);
    arm.setGripperSpeed(0);
    }
        // Returns true when the command should end.
        @Override
        public boolean isFinished() {
          return timeout.get() > timeoutTime;
        }
}
