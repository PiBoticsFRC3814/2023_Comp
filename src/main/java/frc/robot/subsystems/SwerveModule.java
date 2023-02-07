package frc.robot.subsystems;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import java.sql.DriverAction;

import com.ctre.phoenix.sensors.CANCoder;

public class SwerveModule {
	public CANSparkMax            driveMotor;
	private SparkMaxPIDController driveVelocityPidController;
	public RelativeEncoder        driveVelocityEncoder; 

	public CANSparkMax            steerMotor;
	public RelativeEncoder        steerVelocityEncoder;
	private CANCoder              steerAngleEncoder;
	private SparkMaxPIDController steerVelocityPidController;
	private PIDController         steerAnglePIDController;

	private final double[]        steerAnglePIDConstants;
	// private final double[]        driveVelocityPIDConstants;
	public double                 position;
	private double                angleOffset;
	private double                maxCurrent = 0;
	
	/* the SwerveModule subsystem */
	public SwerveModule( int swerveModIndex ) {
		driveMotor = new CANSparkMax( Constants.SWERVE_DRIVE_MOTOR_IDS[ swerveModIndex ], MotorType.kBrushless );
		driveMotor.setIdleMode(IdleMode.kCoast);
		driveMotor.setInverted( Constants.DRIVE_MOTOR_INVERTED[swerveModIndex] );
		driveMotor.setOpenLoopRampRate( 0.2 );
		driveMotor.setSmartCurrentLimit(70, 50);
		//TODO: Add PID for driveMotor

		driveVelocityEncoder = driveMotor.getEncoder();
		//if (wheelMotorInverted)	WheelEncoder.setInverted(true);
		driveVelocityEncoder.setPositionConversionFactor( Constants.drvDistPerPulseRev );
		driveVelocityEncoder.setMeasurementPeriod(20);

		// driveVelocityPIDConstants = Constants.SWERVE_DRIVE_PID_CONSTANTS[swerveModIndex];
		// driveVelocityPidController.setP( driveVelocityPIDConstants[0] );
		// driveVelocityPidController.setI( driveVelocityPIDConstants[1] );
		// driveVelocityPidController.setD( driveVelocityPIDConstants[2] );
		// driveVelocityPidController.setIZone( driveVelocityPIDConstants[3] );
		// driveVelocityPidController.setFF( driveVelocityPIDConstants[4] );
		// driveVelocityPidController.setOutputRange( driveVelocityPIDConstants[5], driveVelocityPIDConstants[6] );

		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.setIdleMode(IdleMode.kCoast);
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotor.setSmartCurrentLimit(50, 40);

		steerAngleEncoder = new CANCoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );

		steerAnglePIDConstants = Constants.SWERVE_STEER_PID_CONSTANTS[swerveModIndex];
		steerAnglePIDController = new PIDController( steerAnglePIDConstants[0], steerAnglePIDConstants[1], steerAnglePIDConstants[2] );

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
		double position = getSteerAngle();
		final var turnOutput = steerAnglePIDController.calculate( position, angle );
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );

		driveMotor.set( speed );
	}

    public void initDefaultCommand() {
			// NOTE: no default command unless running swerve modules seperately
    }
}
