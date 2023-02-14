package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import java.util.function.DoubleSupplier;

public class GyroSwerveDriveCommand extends CommandBase {
  double dX, dY, dZ, dZ2;
  ADIS16470_IMU m_gyro;
  GyroSwerveDrive m_gyroSwerveDrive;

  PIDController steerController =
      new PIDController(
          Constants.SWERVE_ROTATION_PID_CONSTANTS[0],
          Constants.SWERVE_ROTATION_PID_CONSTANTS[1],
          Constants.SWERVE_ROTATION_PID_CONSTANTS[2]);

  public GyroSwerveDriveCommand(
      DoubleSupplier stick_x,
      DoubleSupplier stick_y,
      DoubleSupplier stick_z,
      DoubleSupplier stick_z2,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive) {
    dX = applyDeadzone(stick_x.getAsDouble(), Constants.JOYSTICK_X_DEADZONE);
    dY = applyDeadzone(stick_y.getAsDouble(), Constants.JOYSTICK_Y_DEADZONE);
    dZ = applyDeadzone(stick_z.getAsDouble(), Constants.JOYSTICK_Z_DEADZONE);
    dZ2 = applyDeadzone(stick_z2.getAsDouble(), Constants.JOYSTICK_Z2_DEADZONE);
    m_gyro = imu;
    m_gyroSwerveDrive = gyroSwerveDrive;
    addRequirements(m_gyroSwerveDrive);
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  @Override
  public void execute() {
    if ((dX != 0.0) || (dY != 0.0) || (dZ != 0.0) || (dZ2 != 0.0)) {
      double steerControllerResult = 0.0;
      double steerAngle = Math.toDegrees(Math.atan2(dZ, dZ2) + Math.PI);
      if ((dZ != 0.0) || (dZ2 != 0.0)) {
        steerControllerResult = steerController.calculate(steerAngle, m_gyro.getAngle());
        steerControllerResult /= 360.0;
      }
      m_gyroSwerveDrive.gyroDrive(dX, dY, steerControllerResult, m_gyro.getAngle());
    }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
