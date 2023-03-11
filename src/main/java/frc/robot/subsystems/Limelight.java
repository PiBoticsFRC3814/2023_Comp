// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends SubsystemBase {
  /** Creates a new Limelight. */
  private DoubleSubscriber xAngle;
  private DoubleArraySubscriber robotPose;
  private DoubleSubscriber gotTarget;
  private NetworkTable limelight;
  private final double[] defaultPose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  private double xSetpoint, ySetpoint;

  public boolean validTarget;
  public double[] targetPose;
  public double targetAngle;

  public double[] outputs;

  private PIDController targetLock = new PIDController(Constants.SWERVE_ROTATION_PID_CONSTANTS[0], Constants.SWERVE_ROTATION_PID_CONSTANTS[1], Constants.SWERVE_ROTATION_PID_CONSTANTS[2]);
  private PIDController movePositionX = new PIDController(Constants.SWERVE_STR_PID[0], Constants.SWERVE_STR_PID[1], Constants.SWERVE_STR_PID[2]);
  private PIDController movePositionY = new PIDController(Constants.SWERVE_FWD_PID[0], Constants.SWERVE_FWD_PID[1], Constants.SWERVE_FWD_PID[2]);

  public boolean inPosition;

  public Limelight() {
    limelight = NetworkTableInstance.getDefault().getTable("limelight");
    robotPose = limelight.getDoubleArrayTopic("targetpose_robotspace").subscribe(defaultPose);
    gotTarget = limelight.getDoubleTopic("tv").subscribe(0.0);
    xAngle = limelight.getDoubleTopic("tx").subscribe(0.0);

    targetLock.enableContinuousInput(-180, 180);
    targetLock.setTolerance(0.5);
    movePositionX.setTolerance(0.02);
    movePositionY.setTolerance(0.02);
  }

  public void setPositionSetPoint(double x, double y){
    xSetpoint = x;
    ySetpoint = y;
    
    targetLock.reset();
    movePositionX.reset();
    movePositionY.reset();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    targetPose = robotPose.get();
    targetAngle = xAngle.get();
    validTarget = gotTarget.equals(1.0) && targetPose != defaultPose;

    if(validTarget) {
      outputs[0] = MathUtil.clamp(targetLock.calculate(targetAngle, 0.0), -0.3, 0.3);
      outputs[1] = MathUtil.clamp(-movePositionY.calculate(targetPose[2], ySetpoint), -0.3, 0.3);
      outputs[2] = MathUtil.clamp(movePositionX.calculate(targetPose[1], xSetpoint), -0.3, 0.3);
    } else{
      outputs[0] = 0.0;
      outputs[1] = 0.0;
      outputs[2] = 0.0;
    }

    inPosition = validTarget && movePositionX.atSetpoint() && movePositionY.atSetpoint();
  }
}
