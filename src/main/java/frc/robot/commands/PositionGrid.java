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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.RobotStates;

public class PositionGrid extends CommandBase {

  final double LINEAR_P = 0.4;
  final double LINEAR_I = 0.001;
  final double LINEAR_D = 0.0;
  PIDController distanceController = new PIDController(LINEAR_P, LINEAR_I, LINEAR_D);

  final double ANGULAR_P = 0.01;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  final double STRAFE_P = 0.4;
  final double STRAFE_I = 0.001;
  final double STRAFE_D = 0.0;
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

  /** Creates a new AutoPosition. */
  public PositionGrid(GyroSwerveDrive drivetrain, RobotStates robotStates) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    m_robotStates = robotStates;
    addRequirements(m_drivetrain, m_robotStates);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    turnController.disableContinuousInput();
    turnController.setTolerance(0.1);
    distanceController.setTolerance(0.1);
    strafeController.setTolerance(0.1);

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    robotPose = limelight.getDoubleArrayTopic("targetpose_cameraspace").subscribe(result);
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
    if(!(inPositionA && inPositionX && inPositionZ)){
      double distance = 0.0;
      double distanceX = 0.0;
      double poseResult[] = robotPose.get();

      if (gotTarget.get() == 1.0) {
        if (poseResult.length > 0) {
          double xPos = xFilter.calculate(poseResult[0]);
          double zPos = zFilter.calculate(poseResult[2]);
          double aPos = Math.abs(poseResult[4] - 1.0) >= 1.0 ? aFilter.calculate(poseResult[4]) : 0.0;

          distance = zPos / Math.cos(Math.toRadians(aPos));
          distanceX = xPos;
          if(distance != 0.0){
            if(Math.abs(distance - 0.50) >= 0.03) forwardSpeed = -distanceController.calculate(distance, 0.5); else inPositionZ = true;
            if(Math.abs(aPos) >= 0.03) rotateSpeed = turnController.calculate(aPos, 0.0); else inPositionA = true;
            if(Math.abs(distanceX) >= 0.03) strafeSpeed = strafeController.calculate(distanceX, 0.0); else inPositionX = true;
          }

          SmartDashboard.putNumber("Distance", distanceX);
        }
        DriverStation.reportError("Got Position", false);
      }
      forwardSpeed = MathUtil.clamp(forwardSpeed, -0.5, 0.5);
      rotateSpeed = MathUtil.clamp(rotateSpeed, -0.5, 0.5);
      strafeSpeed = MathUtil.clamp(strafeSpeed, -0.5, 0.5);
    } else {
      m_robotStates.inFrontOfCubeStation = true;
      m_robotStates.moveFromLastAlign = 0;
      timer.reset();
      timer.start();
    }
    if(timer.hasElapsed(Constants.SCORE_FWD_TIME)) forwardSpeed = Constants.SCORE_SPEED;

    m_drivetrain.drive(strafeSpeed, forwardSpeed, rotateSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
