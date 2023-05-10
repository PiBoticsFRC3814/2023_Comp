// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSwerveDrive;

public class PathFollowingPleaseDontBreakRobot extends CommandBase {
  /** Creates a new PathFollowingPleaseDontBreakRobot. */
  PIDController fwdController = new PIDController(0, 0, 0);
  PIDController strController = new PIDController(0, 0, 0);
  ProfiledPIDController rotController = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(6.28, 3.14));
  HolonomicDriveController followerController = new HolonomicDriveController(strController, fwdController, rotController);
  PathPlannerTrajectory testingPath = PathPlanner.loadPath("Testing1", new PathConstraints(1, 1));
  Timer pathTimer = new Timer();
  ADIS16470_IMU gyro;
  GyroSwerveDrive drivetrain;
  public PathFollowingPleaseDontBreakRobot(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pathTimer.reset();
    pathTimer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Trajectory.State goal = testingPath.sample(pathTimer.get());
    ChassisSpeeds adjustedSpeeds = followerController.calculate(drivetrain.getPose(), goal, Rotation2d.fromDegrees(gyro.getAngle()));
    drivetrain.driveUnits(0, 0, 0, 0);
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
