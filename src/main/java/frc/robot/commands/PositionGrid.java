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
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Grabber;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;

public class PositionGrid extends CommandBase {

  final double LINEAR_P = 0.4;
  final double LINEAR_I = 0.001;
  final double LINEAR_D = 0.005;
  PIDController distanceController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

  final double ANGULAR_P = 0.005;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  final double STRAFE_P = 0.4;
  final double STRAFE_I = 0.001;
  final double STRAFE_D = 0.005;
  PIDController strafeController = new PIDController(STRAFE_P, STRAFE_I, STRAFE_D);

  DoubleSubscriber xAngleSub;
  DoubleArraySubscriber robotPose;
  DoubleSubscriber gotTarget;

  GyroSwerveDrive m_drivetrain;
  RobotStates m_robotStates;

  MedianFilter xFilter = new MedianFilter(3);
  MedianFilter zFilter = new MedianFilter(3);
  MedianFilter aFilter = new MedianFilter(3);

  Timer timer;

  boolean inPositionX, inPositionZ, inPositionA, finished;

  ADIS16470_IMU m_gyro;
  Grabber grabber;

  /** Creates a new AutoPosition. */
  public PositionGrid(GyroSwerveDrive drivetrain, RobotStates robotStates, ADIS16470_IMU gyro, Grabber grabber) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_robotStates = robotStates;
    timer = new Timer();
    m_gyro = gyro;
    this.grabber = grabber;
    addRequirements(m_drivetrain, m_robotStates, grabber);
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

    inPositionA = false;
    inPositionX = false;
    inPositionZ = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double forwardSpeed = 0.0;
    double rotateSpeed = 0.0;
    double strafeSpeed = 0.0;
    if(!(inPositionX && inPositionZ)){
      double distance = 0.0;
      double distanceX = 0.0;
      double poseResult[] = robotPose.get();

      if (gotTarget.get() == 1.0) {
        if (poseResult.length > 0) {
          double xPos = xFilter.calculate(poseResult[0]);
          double zPos = zFilter.calculate(poseResult[2]);

          distance = zPos;
          distanceX = xPos;
          if(distance != 0.0){
            if(Math.abs(distance - 0.78) >= 0.03) forwardSpeed = -distanceController.calculate(distance, 0.78); else inPositionZ = true;
            if(m_robotStates.autonomous)rotateSpeed = turnController.calculate(m_gyro.getAngle() % 360.0, 0.0);
            else  rotateSpeed = turnController.calculate(m_gyro.getAngle() % 360.0, 180.0);
            if(Math.abs(distanceX - 0.2023) >= 0.03) strafeSpeed = strafeController.calculate(distanceX, 0.1523); else inPositionX = true;
          }

          SmartDashboard.putNumber("Distance", distance);
          SmartDashboard.putNumber("Correction", forwardSpeed);
        }
        DriverStation.reportError("Got Position", false);
      }
      forwardSpeed = MathUtil.clamp(forwardSpeed, -0.2, 0.2);
      rotateSpeed =   MathUtil.clamp(rotateSpeed, -0.2, 0.2);
      strafeSpeed =   MathUtil.clamp(strafeSpeed, -0.2, 0.2);
    } else {
    }
    //if(timer.hasElapsed(Constants.SCORE_FWD_TIME)) forwardSpeed = Constants.SCORE_SPEED;

    m_drivetrain.drive(strafeSpeed, forwardSpeed, rotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_robotStates.inFrontOfCubeStation = true;
    m_robotStates.moveFromLastAlign = 0;
    m_drivetrain.drive(0.0, 0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return inPositionZ && m_robotStates.autonomous;
  }
}
