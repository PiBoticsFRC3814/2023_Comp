package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
	public CANSparkMax            driveMotor;
	private SparkMaxPIDController driveVelocityPIDController;

	public CANSparkMax            steerMotor;
	private CANCoder              steerAngleEncoder;
	private PIDController         steerAnglePIDController;

	public double                 position;
	private int 				  index;
	public RelativeEncoder 		  driveEncoder;
	
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

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(Constants.DRIVE_POSITION_CONVERSION);

		steerMotor = new CANSparkMax( Constants.SWERVE_STEER_MOTOR_IDS[swerveModIndex], MotorType.kBrushless );
		steerMotor.setIdleMode(IdleMode.kBrake);
		steerMotor.setInverted( Constants.STEER_MOTOR_INVERTED[swerveModIndex] );
		steerMotor.setSmartCurrentLimit(50, 40);

		steerAngleEncoder = new CANCoder( Constants.SWERVE_ENCODER_IDS[swerveModIndex] );
		steerAngleEncoder.configMagnetOffset(Constants.SWERVE_SETPOINT_OFFSET[swerveModIndex]);

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

	public void resetModule(){
		driveEncoder.setPosition(0.0);
	}

	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(driveEncoder.getPosition(), Rotation2d.fromDegrees(steerAngleEncoder.getAbsolutePosition()));
	}

	// angle and speed should be from -1.0 to 1.0, like a joystick input
	public void drive( double speed, double angle ) {
	    // Calculate the turning motor output from the turning PID controller.

		//* Delete slash for tuning offset
		double turnOutput = steerAnglePIDController.calculate( steerAngleEncoder.getAbsolutePosition(), angle );
		steerMotor.set( MathUtil.clamp( turnOutput, -1.0, 1.0 ) );
		driveVelocityPIDController.setReference(Constants.MAX_DRIVETRAIN_SPEED * speed, CANSparkMax.ControlType.kVelocity);
		//driveMotor.set(speed);
		//*/SmartDashboard.putNumber("Angle Module " + index, steerAngleEncoder.getAbsolutePosition());
	}

  public void initDefaultCommand() {
    // NOTE: no default command unless running swerve modules seperately
  }
}
