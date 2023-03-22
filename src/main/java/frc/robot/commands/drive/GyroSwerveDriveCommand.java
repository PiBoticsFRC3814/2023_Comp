package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;

public class GyroSwerveDriveCommand extends CommandBase {
  DoubleSupplier dX, dY, dZ;
  double headingCorrection;
  IntSupplier povHat;
  boolean driveHeading;
  ADIS16470_IMU m_gyro;
  GyroSwerveDrive m_gyroSwerveDrive;

  PIDController turnController = new PIDController(
    Constants.TAG_ALIGN_ROT_PID[0],
     Constants.TAG_ALIGN_ROT_PID[1],
      Constants.TAG_ALIGN_ROT_PID[2]
  );

  public GyroSwerveDriveCommand(
      DoubleSupplier dX,
      DoubleSupplier dY,
      DoubleSupplier dZ,
      IntSupplier povHat,
      ADIS16470_IMU imu,
      GyroSwerveDrive gyroSwerveDrive) {
    this.dX = dX;
    this.dY = dY;
    this.dZ = dZ;
    this.povHat = povHat;
    m_gyro = imu;
    m_gyroSwerveDrive = gyroSwerveDrive;

    turnController.enableContinuousInput(0.0, 360.0);
    turnController.setTolerance(0.5, 1.0);

    addRequirements(m_gyroSwerveDrive);
  }

  @Override
  public void execute() {
    driveHeading = povHat.getAsInt() != -1;
    if(driveHeading){
      switch(povHat.getAsInt()){
        case 90:
          headingCorrection = MathUtil.clamp(turnController.calculate(m_gyro.getAngle() % 360.0, 270.0), -0.4, 0.4);
        case 180:
          headingCorrection = MathUtil.clamp(turnController.calculate(m_gyro.getAngle() % 360.0, 180.0), -0.4, 0.4);
        case 270:
          headingCorrection = MathUtil.clamp(turnController.calculate(m_gyro.getAngle() % 360.0, 90.0), -0.4, 0.4);
        case 0:
          headingCorrection = MathUtil.clamp(turnController.calculate(m_gyro.getAngle() % 360.0, 0.0), -0.4, 0.4);
        default:
          headingCorrection = dZ.getAsDouble();
      }
    }
    m_gyroSwerveDrive.alteredGyroDrive(
        dX.getAsDouble(),
          dY.getAsDouble(),
            driveHeading ? headingCorrection : dZ.getAsDouble(),
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
