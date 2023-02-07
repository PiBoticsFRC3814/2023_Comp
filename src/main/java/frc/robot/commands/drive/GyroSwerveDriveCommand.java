package frc.robot.commands.drive;

import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandBase;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.GyroSwerveDrive;

public class GyroSwerveDriveCommand extends CommandBase {
    DoubleSupplier dX, dY, dZ;
    double driveX, driveY, driveZ;
    double rawX, rawY, rawZ;
    ADIS16470_IMU m_gyro;
    GyroSwerveDrive m_gyroSwerveDrive;
    private static double DEADZONE_MAP_DIVISOR = 1.0 / (1.0 - Constants.JOYSTICK_DEADZONE);

    public GyroSwerveDriveCommand(DoubleSupplier stick_x, DoubleSupplier stick_y, DoubleSupplier stick_z, ADIS16470_IMU imu, GyroSwerveDrive gyroSwerveDrive) {
        dX = stick_x;
        dY = stick_y;
        dZ = stick_z;
        m_gyro = imu;
        m_gyroSwerveDrive = gyroSwerveDrive;
        addRequirements( m_gyroSwerveDrive );
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      rawX = dX.getAsDouble();
      driveX = (Math.abs(rawX) - Constants.JOYSTICK_DEADZONE) * DEADZONE_MAP_DIVISOR;
      if(driveX <= 0.0) driveX = 0.0;
      else if(rawX <= 0.0) driveX = -driveX;

      rawY = dY.getAsDouble();
      driveY = (Math.abs(rawY) - Constants.JOYSTICK_DEADZONE) * DEADZONE_MAP_DIVISOR;
      if(driveY <= 0.0) driveY = 0.0;
      else if(rawY <= 0.0) driveY = -driveY;

      rawZ = dZ.getAsDouble();
      driveZ = (Math.abs(rawZ) - Constants.JOYSTICK_DEADZONE) * DEADZONE_MAP_DIVISOR;
      if(driveZ <= 0.0) driveZ = 0.0;
      else if(rawZ <= 0.0) driveZ = -driveZ;

      m_gyroSwerveDrive.gyroDrive( driveX, driveY, driveZ, Math.toRadians( m_gyro.getAngle () ) );

      SmartDashboard.putNumber( "Gyro ADIS", m_gyro.getAngle() );
    }

  
  @Override
  public void end(boolean interrupted) {
  }
  
  @Override
  public boolean isFinished() {
    return false;
  }
}
