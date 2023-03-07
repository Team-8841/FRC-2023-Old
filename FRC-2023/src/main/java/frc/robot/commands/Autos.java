// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.AutoPaths;
import frc.robot.helpers.AutoOutput;
import frc.robot.helpers.Vector2;

public final class Autos {
  static PathPlannerTrajectory path;
  static PIDController displacementPid = new PIDController(AutoConstants.autoCorrectionPID_P, AutoConstants.autoCorrectionPID_I, AutoConstants.autoCorrectionPID_D);
  static PIDController rotationPid = new PIDController(AutoConstants.autoRotationCorrectionPID_P, AutoConstants.autoRotationCorrectionPID_I, AutoConstants.autoRotationCorrectionPID_D);
  public static void Start() {
    displacementPid.enableContinuousInput(-AutoConstants.autoCorrection_MaxMeters, AutoConstants.autoCorrection_MaxMeters);
    displacementPid.setSetpoint(0);
  }
  //ex: SetPath(AutoConstants.AutoPaths.D0_P1_C_B);
  public static void SetPath(AutoPaths chosenPath) {
    path = PathPlanner.loadPath(AutoConstants.pathNames[chosenPath.value], AutoConstants.autoPathContraints[chosenPath.value]);
  }
  public static PathPlannerState GetPathState(double time) {
    return (PathPlannerState) path.sample(time);
  }
  public static PathPlannerTrajectory GetPath() {
    return path;
  }
  public static void UpdatePID() {
    double displacementP = SmartDashboard.getNumber("auto displacement PID P", AutoConstants.autoCorrectionPID_P);
    double displacementI = SmartDashboard.getNumber("auto displacement PID I", AutoConstants.autoCorrectionPID_I);
    double displacementD = SmartDashboard.getNumber("auto displacement PID D", AutoConstants.autoCorrectionPID_D);
      if(displacementP != displacementPid.getP() || displacementI != displacementPid.getI() || displacementD != displacementPid.getD()) {
        displacementPid.setP(displacementP);
        displacementPid.setI(displacementI);
        displacementPid.setD(displacementD);
      }

      double rotationP = SmartDashboard.getNumber("auto rotation PID P", AutoConstants.autoRotationCorrectionPID_P);
      double rotationI = SmartDashboard.getNumber("auto rotation PID I", AutoConstants.autoRotationCorrectionPID_I);
      double rotationD = SmartDashboard.getNumber("auto rotation PID D", AutoConstants.autoRotationCorrectionPID_D);
        if(rotationP != rotationPid.getP() || rotationI != rotationPid.getI() || rotationD != rotationPid.getD()) {
          rotationPid.setP(rotationP);
          rotationPid.setI(rotationI);
          rotationPid.setD(rotationD);
        }
  }
  //get calculate speed neccesary to follow path and position corrections when neccesary
  public static AutoOutput GetSpeed(double time, Vector2 position, double rotation) {
    var state = Autos.GetPathState(time);

    Vector2 targetPosition = new Vector2(state.poseMeters);
    double targSpeed = state.velocityMetersPerSecond; 
    double velocityAngle = state.poseMeters.getRotation().getRadians();
    Vector2 targVelocity = Vector2.fromPolar(velocityAngle, targSpeed);

    return GetSpeed(position, rotation, targetPosition, targVelocity, state.holonomicRotation.getRadians(), state.holonomicAngularVelocityRadPerSec);
  }
  static AutoOutput GetSpeed(Vector2 position, double rotation, Vector2 targetPosition, Vector2 targVelocity, double targRot, double targRotSpeed) {

    Vector2 dir = new Vector2(targetPosition.x - position.x, targetPosition.y - position.y);

    double rotDif = (Math.PI + targRot - rotation + 8 * Math.PI) % (2 * Math.PI) - Math.PI;
    double rotationCorrectionSpeed = rotationPid.calculate(rotDif)*0;

    double distance = dir.getMagnitude();
    double correctionSpeed = displacementPid.calculate(distance / Math.max(AutoConstants.autoCorrection_MaxMeters, Math.abs(distance)));
    if(dir.getMagnitude() != 0) {
      dir.x/=dir.getMagnitude();
      dir.y/=dir.getMagnitude();
    }
    return new AutoOutput(new Vector2(targVelocity.x + dir.x * correctionSpeed, targVelocity.y+ dir.y * correctionSpeed), targRotSpeed + rotationCorrectionSpeed);

  }
  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
