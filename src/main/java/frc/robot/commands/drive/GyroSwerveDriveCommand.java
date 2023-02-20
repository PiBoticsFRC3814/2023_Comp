package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import java.util.function.DoubleSupplier;

public class GyroSwerveDriveCommand extends CommandBase {
  DoubleSupplier dX, dY, dZ, dZ2;
  ADIS16470_IMU m_gyro;
  GyroSwerveDrive m_gyroSwerveDrive;

  public GyroSwerveDriveCommand(
      DoubleSupplier stick_x,
      DoubleSupplier stick_y,
      DoubleSupplier stick_z,
      DoubleSupplier stick_z2,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive) {
    dX = stick_x;
    dY = stick_y;
    dZ = stick_z;
    dZ2 = stick_z2;
    m_gyro = imu;
    m_gyroSwerveDrive = gyroSwerveDrive;
    addRequirements(m_gyroSwerveDrive);
  }

  @Override
  public void execute() {
    m_gyroSwerveDrive.alteredGyroDrive(dX.getAsDouble(), dY.getAsDouble(), dZ.getAsDouble(), dZ2.getAsDouble(), Math.toRadians(m_gyro.getAngle()));
    DriverStation.reportError("Driving", false);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
