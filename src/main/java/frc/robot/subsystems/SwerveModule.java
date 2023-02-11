package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
	public CANSparkMax            driveMotor;
	private SparkMaxPIDController driveVelocityPIDController;

	public CANSparkMax            steerMotor;
	private CANCoder              steerAngleEncoder;
	private PIDController         steerAnglePIDController;

	public double                 position;
	private double                angleOffset;
	private double                maxCurrent = 0;
	
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

		angleOffset = Constants.SWERVE_SETPOINT_OFFSET[swerveModIndex];
	}

	private static final double INVERSE_180 = 1.0 / 180.0; 

	private double getOffsetSteerEncoderAngle(double angle) {
		return (Math.abs(angle + angleOffset) % 360.0 - 180.0) * INVERSE_180;
	}

	private double maxCurrent(double nowCurrent){
		maxCurrent = Math.max(nowCurrent, maxCurrent);
		return maxCurrent;
	}

	public double getSteerAngle() {
		return getOffsetSteerEncoderAngle(steerAngleEncoder.getAbsolutePosition());
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.
		double turnOutput = steerAnglePIDController.calculate( getSteerAngle(), angle );
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );
		driveVelocityPIDController.setReference(Constants.MAX_DRIVETRAIN_SPEED, CANSparkMax.ControlType.kVelocity);
	}

    public void initDefaultCommand() {
			// NOTE: no default command unless running swerve modules seperately
    }
}
