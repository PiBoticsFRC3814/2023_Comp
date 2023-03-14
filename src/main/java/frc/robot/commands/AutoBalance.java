// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

public class AutoBalance extends CommandBase {
  /** Creates a new AutoBalance. */
  GyroSwerveDrive drivetrain;
  ADIS16470_IMU gyro;
  PIDController balanceController;

  public AutoBalance(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    balanceController = new PIDController(Constants.AUTO_BALANCE_PID[0], Constants.AUTO_BALANCE_PID[1], Constants.AUTO_BALANCE_PID[2]);
    balanceController.setTolerance(1.0, 0.1);
    balanceController.setSetpoint(0.0);
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.drive(0.0, MathUtil.clamp(balanceController.calculate(gyro.getYComplementaryAngle()), -0.3, 0.3), 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.brakeAngle();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return balanceController.atSetpoint();
  }
}
