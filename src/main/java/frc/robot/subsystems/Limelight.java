// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  public boolean targetInView;
  public Pose2d targetPose2d;
  public double closestTagID;

  public Limelight() {
  }

  @Override
  public void periodic() {
    targetInView = LimelightHelpers.getTV("");
    if(targetInView){
      targetPose2d = LimelightHelpers.toPose2D(LimelightHelpers.getTargetPose_RobotSpace(""));
      closestTagID = LimelightHelpers.getFiducialID("");
    }
  }
}
