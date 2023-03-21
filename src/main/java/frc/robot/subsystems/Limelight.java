// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public boolean targetInView;
  public Pose2d targetPose2d;
  public double closestTagID;
  private RobotStates robotStates;

  public Limelight(RobotStates robotStates) {
    this.robotStates = robotStates;
    targetInView = false;
  }

  @Override
  public void periodic() {
    targetInView = LimelightHelpers.getTV("");
    if(targetInView){
      targetPose2d = LimelightHelpers.toPose2D(LimelightHelpers.getBotPose_TargetSpace(""));
      closestTagID = LimelightHelpers.getFiducialID("");
    }
    if((closestTagID != 5.0 || closestTagID != 4.0) && targetInView && (Math.abs(targetPose2d.getX()) <= 2.0)){
      robotStates.inFrontOfCubeStation = true;
    } else robotStates.inFrontOfCubeStation = false;
  }
}
