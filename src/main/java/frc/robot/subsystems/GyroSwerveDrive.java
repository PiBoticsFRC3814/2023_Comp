package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class GyroSwerveDrive extends SubsystemBase {
  private double[] speed = {0.0, 0.0, 0.0, 0.0};
  private double[] angle = {0.0, 0.0, 0.0, 0.0};
  private RobotStates m_RobotStates;

  PIDController steerController = new PIDController(
          Constants.SWERVE_ROTATION_PID_CONSTANTS[0],
          Constants.SWERVE_ROTATION_PID_CONSTANTS[1],
          Constants.SWERVE_ROTATION_PID_CONSTANTS[2]
  );

  private SlewRateLimiter joystickSlewLimiterX;
  private SlewRateLimiter joystickSlewLimiterY;
  private SlewRateLimiter joystickSlewLimiterZ;

  private SwerveModule[] swerveMod = {
    new SwerveModule(0), new SwerveModule(1), new SwerveModule(2), new SwerveModule(3)
  };

  public GyroSwerveDrive(RobotStates robotStates) {
    steerController.enableContinuousInput(0, 360);
    m_RobotStates = robotStates;
    joystickSlewLimiterX = new SlewRateLimiter(Constants.JOYSTICK_X_SLEW_RATE);
    joystickSlewLimiterY = new SlewRateLimiter(Constants.JOYSTICK_Y_SLEW_RATE);
    joystickSlewLimiterZ = new SlewRateLimiter(Constants.JOYSTICK_Z_SLEW_RATE);
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  public void alteredGyroDrive(double dX, double dY, double dZ, double driveMultiplier, double gyroAngle){
    dX = -applyDeadzone(dX, Constants.JOYSTICK_X_DEADZONE);
    dY = -applyDeadzone(dY, Constants.JOYSTICK_Y_DEADZONE);
    dZ = -applyDeadzone(dZ, Constants.JOYSTICK_Z_DEADZONE) * 0.5;
    if ((dX != 0.0) || (dY != 0.0) || (dZ != 0.0)) {
      gyroDrive(
        joystickSlewLimiterX.calculate(dX * driveMultiplier),
         joystickSlewLimiterY.calculate(dY * driveMultiplier),
         joystickSlewLimiterZ.calculate(dZ * driveMultiplier),
          gyroAngle
      );
      m_RobotStates.inFrontOfCubeStation = false;
    } else{
      speed[0] = 0.0;
      speed[1] = 0.0;
      speed[2] = 0.0;
      speed[3] = 0.0;
      setSetpoints();
    }
  }

  public void gyroDrive(double str, double fwd, double rot, double gyroAngle) {
    double intermediary = fwd * Math.cos(gyroAngle) + str * Math.sin(gyroAngle);
    str = -fwd * Math.sin(gyroAngle) + str * Math.cos(gyroAngle);
    drive(str, intermediary, rot);
    setSetpoints();
  }

  public void drive(double str, double fwd, double rot) {
    if(str != 0.0 || fwd != 0.0 || rot != 0.0) computeSwerveInputs(str, fwd, rot);
    else{
      speed[0] = 0.0;
      speed[1] = 0.0;
      speed[2] = 0.0;
      speed[3] = 0.0;
    }
    setSetpoints();
  }

  public void drive(double[] inputs) {
    drive(inputs[0], inputs[1], inputs[2]);
  }

  public void driveAtHeading(double heading, double fwd, double str, double gyroAngle){
    drive(str, fwd, MathUtil.clamp(steerController.calculate(gyroAngle % 360.0, heading), -0.3, 0.3));
  }

  /*
   * Brake system
   */
  public void brakeAngle() {
    angle[0] = -0.25;
    angle[1] = 0.25;
    angle[2] = -0.25;
    angle[3] = 0.25;
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
