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
      DoubleSupplier stick_x,
      DoubleSupplier stick_y,
      DoubleSupplier stick_z,
      BooleanSupplier multiplier,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive) {
    dX = stick_x;
    dY = stick_y;
    dZ = stick_z;
    driveFast = multiplier;
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
         driveFast.getAsBoolean() ? Constants.FAST_SPEED : Constants.SLOW_SPEED,
          Math.toRadians(m_gyro.getAngle() % 360.0)
    );
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
