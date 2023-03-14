// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSwerveDrive;

public class PositionSubstation extends CommandBase {

  final double LINEAR_P = 0.4;
  final double LINEAR_I = 0.001;
  final double LINEAR_D = 0.005;
  PIDController distanceController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.00;
  PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  final double STRAFE_P = 0.4;
  final double STRAFE_I = 0.001;
  final double STRAFE_D = 0.005;
  PIDController strafeController = new PIDController(STRAFE_P, STRAFE_I, STRAFE_D);

  DoubleSubscriber xAngleSub;
  DoubleArraySubscriber robotPose;
  DoubleSubscriber gotTarget;

  GyroSwerveDrive m_drivetrain;

  ADIS16470_IMU m_gyro;

  double pastDistance;

  MedianFilter xFilter = new MedianFilter(3);
  MedianFilter zFilter = new MedianFilter(3);
  MedianFilter aFilter = new MedianFilter(3);

  /** Creates a new AutoPosition. */
  public PositionSubstation(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_gyro = gyro;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(0.01);
    distanceController.setTolerance(0.01);
    strafeController.setTolerance(0.01);

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    robotPose = limelight.getDoubleArrayTopic("targetpose_robotspace").subscribe(result);
    gotTarget = limelight.getDoubleTopic("tv").subscribe(0.0);
    DriverStation.reportError("Init", false);
    pastDistance = robotPose.get()[2];
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // a whole bunch of PID stuff that maybe shouldn't be here
    double forwardSpeed = 0.0;
    double rotateSpeed = 0.0;
    double strafeSpeed = 0.0;
    double distance = 0.0;
    double distanceX = 0.0;
    double poseResult[] = robotPose.get();
    double aPos;

    if (gotTarget.get() == 1.0) {
      if (poseResult.length > 0) {
        double xPos = xFilter.calculate(poseResult[0]);
        double zPos = zFilter.calculate(poseResult[2]);

        distance = zPos;// / Math.cos(Math.toRadians(aPos));
        distanceX = xPos;
        if(distance != 0.0){
          if(Math.abs(distance - 1.05) >= 0.03) forwardSpeed = -distanceController.calculate(distance, 1.05);
          rotateSpeed = turnController.calculate(m_gyro.getAngle(), 0.0);
          if(Math.abs(distanceX + 0.5) >= 0.03) strafeSpeed = strafeController.calculate(distanceX, -0.45);
        }

        SmartDashboard.putNumber("Distance", distanceX);
      }
      DriverStation.reportError("Got Position", false);
    } else{
      rotateSpeed = turnController.calculate(m_gyro.getAngle() % 360.0, 0.0);
    }
    forwardSpeed = MathUtil.clamp(forwardSpeed, -0.2, 0.2);
    rotateSpeed = MathUtil.clamp(rotateSpeed, -0.2, 0.2);
    strafeSpeed = MathUtil.clamp(strafeSpeed, -0.2, 0.2);
    m_drivetrain.drive(strafeSpeed, forwardSpeed, rotateSpeed);
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
