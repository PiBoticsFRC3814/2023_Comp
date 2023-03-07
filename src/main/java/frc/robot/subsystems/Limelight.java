// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  final double LINEAR_P = 1.0;
  final double LINEAR_I = 0.0;
  final double LINEAR_D = 0.0;
  PIDController distanceController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

  final double ANGULAR_P = 0.015;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  final double STRAFE_P = 0.04;
  final double STRAFE_I = 0.0;
  final double STRAFE_D = 0.0;
  PIDController strafeController = new PIDController(STRAFE_P, STRAFE_I, STRAFE_D);

  DoubleSubscriber xAngleSub;
  DoubleArraySubscriber robotPose;
  DoubleSubscriber gotTarget;

  GyroSwerveDrive m_drivetrain;

  double pastDistance;

  MedianFilter xFilter = new MedianFilter(3);
  MedianFilter zFilter = new MedianFilter(3);
  MedianFilter aFilter = new MedianFilter(3);

  public Limelight() {
    double[] result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    turnController.disableContinuousInput();
    turnController.setTolerance(0.2);

    /*
    // before getEntry it was getDoubleTopic
    // before getDoubleArray it was getDoubleArrayTopic
    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    xAngleSub = limelight.getEntry("tx").getDouble(0).subscribe(0.0);
    robotPose = limelight.getDoubleArray("campose").subscribe(result);
    gotTarget = limelight.getEntry("tv").getDouble(0).subscribe(0.0);
    pastDistance = robotPose.get()[2];
    //*/
  }

  /*
  public double limelightAngleX() {
    return xAngleSub.get();
  }

  public double limelightPosZ() {
    return robotPose.get()[2];
  }

  public double limelightAngleAlpha() {
    return robotPose.get()[4];
  }
//*/
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
