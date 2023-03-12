// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSwerveDrive;

public class AutoDriveDistance extends CommandBase {
  /** Creates a new AutoDriveDistance. */
  private GyroSwerveDrive drivetrain;
  private ADIS16470_IMU gyro;
  private Timer autoTimer;
  private TrapezoidProfile driveProfile;

  private double heading, direction;

  private boolean finished;

  final double ANGULAR_P = 0.005;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);

  double rotateSpeed;
  
  public AutoDriveDistance(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro, DoubleSupplier distance, DoubleSupplier heading, DoubleSupplier direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    this.heading = heading.getAsDouble();
    this.direction = direction.getAsDouble();
    driveProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(0.5, 0.5),
       new TrapezoidProfile.State(distance.getAsDouble(), 0.0),
        new TrapezoidProfile.State(0.0, 0.0)
    );

    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(0.01);
    
    autoTimer = new Timer();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.reset();
    autoTimer.start();
    finished = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = driveProfile.calculate(autoTimer.get());
    rotateSpeed = autoTimer.get() >= 2.0 ? turnController.calculate(gyro.getAngle() % 360.0, 180.0) : 0.0;
    rotateSpeed =   MathUtil.clamp(rotateSpeed, -0.2, 0.2);
    drivetrain.gyroDrive(setpoint.velocity * Math.sin(Math.toRadians(direction)), setpoint.velocity * Math.cos(Math.toRadians(direction)), rotateSpeed, Math.toRadians(gyro.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoTimer.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
