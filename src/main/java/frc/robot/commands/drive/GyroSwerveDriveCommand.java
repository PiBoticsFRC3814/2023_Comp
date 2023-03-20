package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class GyroSwerveDriveCommand extends CommandBase {
  DoubleSupplier dX, dY, dZ;
  BooleanSupplier driveFast;
  ADIS16470_IMU m_gyro;
  GyroSwerveDrive m_gyroSwerveDrive;

  public GyroSwerveDriveCommand(
      DoubleSupplier dX,
      DoubleSupplier dY,
      DoubleSupplier dZ,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive) {
    this.dX = dX;
    this.dY = dY;
    this.dZ = dZ;
    m_gyro = imu;
    m_gyroSwerveDrive = gyroSwerveDrive;
    addRequirements(m_gyroSwerveDrive);
  }

  @Override
  public void execute() {
    m_gyroSwerveDrive.alteredGyroDrive(
      dX.getAsDouble(),
       dY.getAsDouble(),
        dZ.getAsDouble(),
          Math.toRadians(m_gyro.getAngle())
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
