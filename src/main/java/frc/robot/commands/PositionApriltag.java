// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.GyroSwerveDrive;
import frc.robot.subsystems.Limelight;

public class PositionApriltag extends CommandBase {

  private GyroSwerveDrive drivetrain;
  private Limelight limelight;
  private ADIS16470_IMU gyro;

  private boolean start;

  private PIDController strController = new PIDController(Constants.TAG_ALIGN_STR_PID[0], Constants.TAG_ALIGN_STR_PID[1], Constants.TAG_ALIGN_STR_PID[2]);
  private PIDController fwdController = new PIDController(Constants.TAG_ALIGN_FWD_PID[0], Constants.TAG_ALIGN_FWD_PID[1], Constants.TAG_ALIGN_FWD_PID[2]);
  private PIDController rotController = new PIDController(Constants.TAG_ALIGN_ROT_PID[0], Constants.TAG_ALIGN_ROT_PID[1], Constants.TAG_ALIGN_ROT_PID[2]);

  /** Creates a new AutoPosition. */
  public PositionApriltag(GyroSwerveDrive drivetrain, Limelight limelight, ADIS16470_IMU gyro, double goalX, double goalY, double goalZ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrain;
    this.limelight = limelight;
    this.gyro = gyro;

    strController.setIntegratorRange(-0.2, 0.2);
    fwdController.setIntegratorRange(-0.2, 0.2);
    rotController.setIntegratorRange(-0.2, 0.2);

    strController.setTolerance(0.03, 0.05);
    fwdController.setTolerance(0.03, 0.05);
    rotController.setTolerance(0.03, 0.05);

    rotController.enableContinuousInput(0.0, 360.0);

    strController.setSetpoint(goalX);
    fwdController.setSetpoint(goalY);
    rotController.setSetpoint(goalZ);

    addRequirements(drivetrain, limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    start = false;
    strController.reset();
    fwdController.reset();
    rotController.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.targetInView && !start){
      drivetrain.resetOdometry(limelight.targetPose2d);
      start = true;
    }

    if(start){
      double correctionX = -MathUtil.clamp(strController.calculate(drivetrain.getPose().getY()), -0.2, 0.2);
      double correctionY = MathUtil.clamp(fwdController.calculate(drivetrain.getPose().getX()), -0.2, 0.2);
      double correctionZ = MathUtil.clamp(rotController.calculate(gyro.getAngle() % 360.0), -0.2, 0.2);
      drivetrain.drive(correctionX, correctionY, correctionZ);
      System.out.println("" + drivetrain.getPose().getY());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.motorZero();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
