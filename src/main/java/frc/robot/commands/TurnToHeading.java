// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.GyroSwerveDrive;

public class TurnToHeading extends CommandBase {
  /** Creates a new TurnToHeading. */
  GyroSwerveDrive drivetrain;
  ADIS16470_IMU gyro;

  final double ANGULAR_P = 0.005;
  final double ANGULAR_I = 0.0;
  final double ANGULAR_D = 0.0;
  PIDController turnController = new PIDController(ANGULAR_P, ANGULAR_I, ANGULAR_D);
  
  public TurnToHeading(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro, DoubleSupplier heading) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;

    turnController.setSetpoint(heading.getAsDouble());

    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(0.5, 1.0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0.0, 0.0, MathUtil.clamp(turnController.calculate(gyro.getAngle() % 360.0), -0.2, 0.2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return turnController.atSetpoint();
  }
}
