package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
<<<<<<< Updated upstream
  public CANSparkMax driveMotor;

  public CANSparkMax steerMotor;
  private CANCoder steerAngleEncoder;
  private PIDController steerAnglePIDController;

  private final double[] steerAnglePIDConstants;
  public double position;
  private double angleOffset;

  private int index;

  /* the SwerveModule subsystem */
  public SwerveModule(int swerveModIndex) {
    driveMotor =
        new CANSparkMax(Constants.SWERVE_DRIVE_MOTOR_IDS[swerveModIndex], MotorType.kBrushless);
    driveMotor.setIdleMode(IdleMode.kCoast);
    driveMotor.setInverted(Constants.DRIVE_MOTOR_INVERTED[swerveModIndex]);
    driveMotor.setOpenLoopRampRate(0.2);
    driveMotor.setSmartCurrentLimit(70, 50);

    steerMotor =
        new CANSparkMax(Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless);
    steerMotor.setIdleMode(IdleMode.kBrake);
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
    index = swerveModIndex;
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
        SmartDashboard.putNumber("Angle Steer" + index, angle);

    driveMotor.set(speed);
  }

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
=======
	public CANSparkMax            driveMotor;
	private SparkMaxPIDController driveVelocityPIDController;

	public CANSparkMax            steerMotor;
	private CANCoder              steerAngleEncoder;
	private PIDController         steerAnglePIDController;

	public double                 position;
	private double                angleOffset;
	private int index;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		driveMotor.setIdleMode(IdleMode.kBrake);
		driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotor.setOpenLoopRampRate( 0.2 );
		driveMotor.setSmartCurrentLimit(70, 50);

		driveVelocityPIDController = driveMotor.getPIDController();
		driveVelocityPIDController.setP(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][0]);
		driveVelocityPIDController.setI(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][1]);
		driveVelocityPIDController.setD(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][2]);
		driveVelocityPIDController.setIZone(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][3]);
		driveVelocityPIDController.setFF(Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex][4]);

		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.setIdleMode(IdleMode.kBrake);
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotor.setSmartCurrentLimit(50, 40);

		steerAngleEncoder = new CANCoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );

		steerAnglePIDController = new PIDController( 
			Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][0],
				Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][1],
					Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex][2]
		);

        // Limit the PID Controller's input range between -1.0 and 1.0 and set the input
		// to be continuous.
        steerAnglePIDController.enableContinuousInput( -1.0, 1.0 );
		steerAnglePIDController.setTolerance( Constants.SWERVE_PID_TOLERANCE );

		index = swerveModIndex;
	}

	private static final double INVERSE_180 = 1.0 / 180.0; 

	private double getOffsetSteerEncoderAngle(double angle) {
		return (Math.abs(angle + Constants.SWERVE_SETPOINT_OFFSET[index]) % 360.0 - 180.0) * INVERSE_180;
	}

	public double getSteerAngle() {
		return getOffsetSteerEncoderAngle(steerAngleEncoder.getAbsolutePosition());
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.
		double turnOutput = steerAnglePIDController.calculate( getSteerAngle(), angle );
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );
		driveVelocityPIDController.setReference(Constants.MAX_DRIVETRAIN_SPEED * speed, CANSparkMax.ControlType.kVelocity);
		//SmartDashboard.putData("Steer PID Controller " + index, steerAnglePIDController);
		//SmartDashboard.putNumber("Velocity" + index, driveMotor.getEncoder().getVelocity());
	}

    public void initDefaultCommand() {
			// NOTE: no default command unless running swerve modules seperately
    }
>>>>>>> Stashed changes
}
