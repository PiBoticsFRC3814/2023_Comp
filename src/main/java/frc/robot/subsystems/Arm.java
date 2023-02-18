// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX shoulder1;
  private WPI_TalonSRX shoulder2;
  private WPI_TalonSRX extend;
  private DutyCycleEncoder shoulderEncoder;

  public boolean extendAtPos;
  public boolean shoulderAtPos;

  private DoubleSolenoid armBrake;

  private PIDController angleController;

  boolean switch1;
  boolean switch2;
  boolean switch3;
  boolean switch4;

  public Arm() {
    extend = new WPI_TalonSRX(Constants.EXTEND_ID);
    extend.setNeutralMode(NeutralMode.Brake);

    armBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ARM_ID_OPEN, Constants.ARM_ID_CLOSE);
    armBrake.set(DoubleSolenoid.Value.kReverse);
    shoulder1 = new WPI_TalonSRX(Constants.SHOULDER_ID_1);
    shoulder2 = new WPI_TalonSRX(Constants.SHOULDER_ID_2);
    shoulder1.setInverted(false);
    shoulder2.setInverted(true);
    shoulder1.setNeutralMode(NeutralMode.Brake);
    shoulder2.setNeutralMode(NeutralMode.Brake);
    extend.setNeutralMode(NeutralMode.Brake);
    shoulder1.configPeakCurrentLimit(70, 1);
    shoulder2.configPeakCurrentLimit(70, 1);
    extend.configPeakCurrentLimit(70, 1);
    shoulderEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER_PORT);

    angleController =
        new PIDController(
            Constants.ARM_ANGLE_PID_CONSTANTS[0],
            Constants.ARM_ANGLE_PID_CONSTANTS[1],
            Constants.ARM_ANGLE_PID_CONSTANTS[2]);

    extendAtPos = false;
    shoulderAtPos = false;
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  public void ArmDirectControl(DoubleSupplier passedArm, DoubleSupplier passedExtend) {
    double armSpeed = applyDeadzone(passedArm.getAsDouble(), Constants.JOYSTICK_X_DEADZONE);
    double extendSpeed = applyDeadzone(passedExtend.getAsDouble(), Constants.JOYSTICK_X_DEADZONE);
    if (armSpeed != 0.0) armBrake.set(DoubleSolenoid.Value.kForward);
    else armBrake.set(DoubleSolenoid.Value.kReverse);
    shoulder1.set(armSpeed);
    shoulder2.set(armSpeed);
    extend.set(extendSpeed);
  }

  public void ArmAngle(double angle) {
    shoulder1.set(angleController.calculate(shoulderEncoder.getAbsolutePosition(), angle));
    shoulder2.set(angleController.calculate(shoulderEncoder.getAbsolutePosition(), angle));
    shoulderAtPos = !angleController.atSetpoint();

    if (!shoulderAtPos) armBrake.set(DoubleSolenoid.Value.kReverse);
    else armBrake.set(DoubleSolenoid.Value.kForward);
  }

  public void ArmDistance(int position) {
    extend.set(-1.0);
    extendAtPos = false;
    while (!extendAtPos) {
      switch (position) {
        case 0:
          extend.set(Constants.EXTEND_SPEED);
          if (switch1) {
            extend.set(0.0);
            extendAtPos = true;
          }

        case 1:
          extend.set(Constants.EXTEND_SPEED);
          if (switch2) {
            extend.set(0.0);
            extendAtPos = true;
          }
        case 2:
          extend.set(Constants.EXTEND_SPEED);
          if (switch3) {
            extend.set(0.0);
            extendAtPos = true;
          }
        case 3:
          extend.set(Constants.EXTEND_SPEED);
          if (switch4) {
            extend.set(0.0);
            extendAtPos = true;
          }
      }
    }
  }
}
