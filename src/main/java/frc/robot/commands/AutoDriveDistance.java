// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.DoubleSupplier;

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
  
  public AutoDriveDistance(GyroSwerveDrive drivetrain, ADIS16470_IMU gyro, DoubleSupplier distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.gyro = gyro;
    driveProfile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(5, 9.8),
       new TrapezoidProfile.State(distance.getAsDouble(), 0.0),
        new TrapezoidProfile.State(0.0, 0.0)
    );
    
    autoTimer = new Timer();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    autoTimer.reset();
    autoTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    var setpoint = driveProfile.calculate(autoTimer.get());
    drivetrain.driveAtHeading(180.0, setpoint.velocity, 0.0, gyro.getAngle());
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
