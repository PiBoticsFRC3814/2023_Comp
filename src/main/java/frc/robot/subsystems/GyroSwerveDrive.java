package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroSwerveDrive extends SubsystemBase {
  private double[] speed = {0.0, 0.0, 0.0, 0.0};
  private double[] angle = {0.0, 0.0, 0.0, 0.0};

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
  };

  public GyroSwerveDrive() {}

  public void gyroDrive(double str, double fwd, double rot, double gyroAngle) {
    double intermediary = fwd * Math.cos(gyroAngle) + str * Math.sin(gyroAngle);
    str = -fwd * Math.sin(gyroAngle) + str * Math.cos(gyroAngle);
    computeSwerveInputs(str, intermediary, rot);
    setSetpoints();
  }

  public void drive(double str, double fwd, double rot) {
    computeSwerveInputs(str, fwd, rot);
    setSetpoints();
  }

  /*
   * Brake system
   */
  public void brakeAngle() {
    angle[0] = 0.25;
    angle[1] = -0.25;
    angle[2] = 0.25;
    angle[3] = -0.25;
    speed[0] = 0.0;
    speed[1] = 0.0;
    speed[2] = 0.0;
    speed[3] = 0.0;
    setSetpoints();
  }

  private double getDeltaAngle(double alpha, double beta) {
    return 1.0 - Math.abs(Math.abs(alpha - beta) % 2.0 - 1.0);
  }

  private void computeSwerveInputs(double str, double fwd, double rot) {
    double a = str - rot * (Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS);
    double b = str + rot * (Constants.SWERVE_FRAME_LENGTH / Constants.SWERVE_RADIUS);
    double c = fwd - rot * (Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS);
    double d = fwd + rot * (Constants.SWERVE_FRAME_WIDTH / Constants.SWERVE_RADIUS);

    speed[1] = Math.sqrt((a * a) + (d * d));
    speed[2] = Math.sqrt((a * a) + (c * c));
    speed[0] = Math.sqrt((b * b) + (d * d));
    speed[3] = Math.sqrt((b * b) + (c * c));

    angle[1] = Math.atan2(a, d) / Math.PI;
    angle[2] = Math.atan2(a, c) / Math.PI;
    angle[0] = Math.atan2(b, d) / Math.PI;
    angle[3] = Math.atan2(b, c) / Math.PI;
  }

  private void setSetpoints() {
    for (int i = 0; i < 4; i++) {
      double steerAngle = swerveMod[i].getSteerAngle();
      if (getDeltaAngle(angle[i], steerAngle) > 0.5) {
        angle[i] = Math.abs(Math.abs(angle[i] + 2.0) % 2.0) - 1.0;
        speed[i] = -speed[i];
      }
      swerveMod[i].drive(speed[i], angle[i]);
    }
  }

  public void WheelToCoast() {
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.setIdleMode(IdleMode.kCoast);
    }
  }

  public void WheelToBrake() {
    for (int i = 0; i < 4; i++) {
      swerveMod[i].driveMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  public void outputEncoderPos() {
    SmartDashboard.putNumber("Module 1 encoder", swerveMod[0].getSteerAngle());
    SmartDashboard.putNumber("Module 2 encoder", swerveMod[1].getSteerAngle());
    SmartDashboard.putNumber("Module 3 encoder", swerveMod[2].getSteerAngle());
    SmartDashboard.putNumber("Module 4 encoder", swerveMod[3].getSteerAngle());
  }
}
