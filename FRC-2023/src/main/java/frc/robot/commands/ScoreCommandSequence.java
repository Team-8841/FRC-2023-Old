package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ArmConstants.ArmPosition;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.FieldConstants.GoalColumn;
import frc.robot.Constants.FieldConstants.GoalRow;
import frc.robot.Constants.FieldConstants.Grid;
import frc.robot.helpers.Vector2;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

//the general command sequence for automatically scoring
public class ScoreCommandSequence extends SequentialCommandGroup  {
      
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    public SwerveDrive swerveDrive;
    public Arm arm;
    public GoalColumn targetColumn;
    public GoalRow targetRow;
    public Grid targetGoal;
    public DriveToCommand driveToCommand;

    private CommandXboxController controller;

    public ScoreCommandSequence(SwerveDrive swerveDrive, Vision limelight, Arm arm, GoalColumn targetColumn, GoalRow targetRow, Grid targetGoal) {
      super(
        new ArmGotoPosition(arm, ArmPosition.home),
        new SetGripperCommand(arm, true, true),
        new DriveToCommand(swerveDrive, limelight, new Vector2(FieldConstants.gridPosition.x - targetRow.initialXDistance, FieldConstants.gridPosition.y + targetGoal.offset + targetColumn.offset), (DriverStation.getAlliance() == Alliance.Red ? 0 : Math.PI)),
        new ArmGotoPosition(arm, targetRow.armPosition),
        new DriveToCommand(swerveDrive, limelight, new Vector2(FieldConstants.gridPosition.x - targetRow.finalXDistance, FieldConstants.gridPosition.y + targetGoal.offset + targetColumn.offset), (DriverStation.getAlliance() == Alliance.Red ? 0 : Math.PI)),
        new SetGripperCommand(arm, false, false),
        new ArmGotoPosition(arm, ArmPosition.home)
      );
      
    }
}