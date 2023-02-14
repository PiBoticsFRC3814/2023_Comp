package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class SwerveModule {
  public CANSparkMax driveMotor;

  public CANSparkMax steerMotor;
  private CANCoder steerAngleEncoder;
  private PIDController steerAnglePIDController;

  private final double[] steerAnglePIDConstants;
  public double position;
  private double angleOffset;

  /* the SwerveModule subsystem */
  public SwerveModule(int swerveModIndex) {
    driveMotor =
        new CANSparkMax(Constants.SWERVE_DRIVE_MOTOR_IDS[swerveModIndex], MotorType.kBrushless);
    driveMotor.setIdleMode(IdleMode.kBrake);
    driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERTED[swerveModIndex]);
    driveMotor.setOpenLoopRampRate(0.2);
    driveMotor.setSmartCurrentLimit(70, 50);

    steerMotor =
        new CANSparkMax(Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless);
    steerMotor.setIdleMode(IdleMode.kCoast);
    steerMotor.setInverted(Constants.STEER_MOTOR_INVERTED[swerveModIndex]);
    steerMotor.setSmartCurrentLimit(50, 40);

    steerAngleEncoder = new CANCoder(Constants.SWERVE_ENCODER_IDS[swerveModIndex]);

    steerAnglePIDConstants = Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex];
    steerAnglePIDController =
        new PIDController(
            steerAnglePIDConstants[0], steerAnglePIDConstants[1], steerAnglePIDConstants[2]);

    // Limit the PID Controller's input range between -1.0 and 1.0 and set the input
    // to be continuous.
    steerAnglePIDController.enableContinuousInput(-1.0, 1.0);
    steerAnglePIDController.setTolerance(Constants.SWERVE_PID_TOLERANCE);

    angleOffset = Constants.SWERVE_SETPOINT_OFFSET[swerveModIndex];
  }

  private static final double INVERSE_180 = 1.0 / 180.0;

  private double getOffsetSteerEncoderAngle(double angle) {
    return (Math.abs(angle + angleOffset) % 360.0 - 180.0) * INVERSE_180;
  }

  public double getSteerAngle() {
    return getOffsetSteerEncoderAngle(steerAngleEncoder.getAbsolutePosition());
  }

  // angle and speed should be from -1.0 to 1.0, like a joystick input
  public void drive(double speed, double angle) {
    // Calculate the turning motor output from the turning PID controller.
    steerMotor.set(
        MathUtil.clamp(steerAnglePIDController.calculate(getSteerAngle(), angle), -1.0, 1.0));

    driveMotor.set(speed);
  }

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
}
