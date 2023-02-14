// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSwerveDrive;

public class AutoPosition extends CommandBase {

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

  /** Creates a new AutoPosition. */
  public AutoPosition(GyroSwerveDrive drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_drivetrain = drivetrain;
    addRequirements(m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    double[] result = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    turnController.disableContinuousInput();
    turnController.setTolerance(0.2);

    NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
    xAngleSub = limelight.getDoubleTopic("tx").subscribe(0.0);
    robotPose = limelight.getDoubleArrayTopic("campose").subscribe(result);
    gotTarget = limelight.getDoubleTopic("tv").subscribe(0.0);
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
    double poseResult[] = robotPose.get();

    if (gotTarget.get() == 1.0) {
      if (poseResult.length > 0) {
        double xPos = xFilter.calculate(xAngleSub.get());
        double zPos = zFilter.calculate(poseResult[2]);
        double aPos = aFilter.calculate(poseResult[4]);

        distance = zPos / Math.cos(Math.toRadians(aPos));
        forwardSpeed = distanceController.calculate(distance, -1.5);
        rotateSpeed = turnController.calculate(aPos, 0.0);
        strafeSpeed = -strafeController.calculate(xPos, 0.0);

        m_drivetrain.drive(strafeSpeed, forwardSpeed, rotateSpeed);
      }
      DriverStation.reportError("Got Position", false);
      DriverStation.reportError("" + distance, false);
      DriverStation.reportError("Output " + forwardSpeed, false);
    }
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
