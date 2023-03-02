// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private WPI_TalonSRX shoulder1;
  private WPI_TalonSRX shoulder2;
  private CANSparkMax extend;
  private DutyCycleEncoder shoulderEncoder;

  public boolean extendAtPos;
  public boolean shoulderAtPos;

  private DoubleSolenoid armBrake;

  private PIDController angleController;

  private RelativeEncoder extendEncoder;
  private SparkMaxPIDController extendController;
  private SparkMaxLimitSwitch extendHomeSwitch;

  private Double extendOffset;
  private boolean extendIsHomed;

  private DigitalInput switch1;
  private DigitalInput switch2;
  private DigitalInput switch3;
  private DigitalInput switch4;

  public Arm() {
    extend = new CANSparkMax(Constants.EXTEND_ID, MotorType.kBrushless);
    extend.setIdleMode(IdleMode.kBrake);
    extend = new CANSparkMax(Constants.EXTEND_ID, MotorType.kBrushless);
    extend.setIdleMode(IdleMode.kBrake);

    armBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.ARM_ID_OPEN, Constants.ARM_ID_CLOSE);
    armBrake.set(DoubleSolenoid.Value.kForward);
    shoulder1 = new WPI_TalonSRX(Constants.SHOULDER_ID_1);
    shoulder2 = new WPI_TalonSRX(Constants.SHOULDER_ID_2);
    shoulder1.setInverted(false);
    shoulder2.setInverted(true);
    shoulder1.setNeutralMode(NeutralMode.Brake);
    shoulder2.setNeutralMode(NeutralMode.Brake);

    shoulder1.configPeakCurrentLimit(70, 1);
    shoulder2.configPeakCurrentLimit(70, 1);

    shoulderEncoder = new DutyCycleEncoder(Constants.ARM_ENCODER_PORT);

    extendEncoder = extend.getEncoder();

    extendController = extend.getPIDController();
    extendController.setP(Constants.EXTEND_PID_CONSTANTS[0]);
    extendController.setI(Constants.EXTEND_PID_CONSTANTS[1]);
    extendController.setD(Constants.EXTEND_PID_CONSTANTS[2]);
    extendController.setIZone(Constants.EXTEND_PID_CONSTANTS[3]);
    extendController.setFF(Constants.EXTEND_PID_CONSTANTS[4]);
    extendController.setOutputRange(Constants.EXTEND_PID_CONSTANTS[5], Constants.EXTEND_PID_CONSTANTS[6]);

    extendHomeSwitch = extend.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);

    extendOffset = 0.0;
    extendIsHomed = false;

    switch1 = new DigitalInput(3);
    switch2 = new DigitalInput(2);
    switch3 = new DigitalInput(1);
    switch4 = new DigitalInput(0);

    angleController =
        new PIDController(
            Constants.ARM_ANGLE_PID_CONSTANTS[0],
            Constants.ARM_ANGLE_PID_CONSTANTS[1],
            Constants.ARM_ANGLE_PID_CONSTANTS[2]);

    angleController.disableContinuousInput();
    angleController.setTolerance(0.02);
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
    shoulder1.set(armSpeed * 0.3);
    shoulder2.set(armSpeed * 0.3);
    extend.set(extendSpeed);
  }

  public double GetArmAngle(){
    return shoulderEncoder.getAbsolutePosition();
  }

  public void ArmAngle(double angle) {
    double correction = -angleController.calculate(shoulderEncoder.getAbsolutePosition(), angle);
    shoulder1.set(correction);
    shoulder2.set(correction);
    shoulderAtPos = angleController.atSetpoint();

    if (Math.abs(correction) <= 0.1) armBrake.set(DoubleSolenoid.Value.kForward);
    else armBrake.set(DoubleSolenoid.Value.kReverse);
  }



  public void ArmDistance(int position) {
    extendAtPos = false;
    if(extendIsHomed){
      extend.set(-Constants.EXTEND_HOME_SPEED);
      while(!extendAtPos){
        if(!extendHomeSwitch.isPressed()){
          extend.set(0.0);
          extendOffset = extendEncoder.getPosition();
          extendIsHomed = true;
          break;
        }
      }
    }
    extendController.setReference(1.0 + extendOffset, ControlType.kPosition);

    //switch now runs off currentPods instead of position, make it go from currentPos to position
    //dont think we need it anymore, but i dont want to delete it just in case
    /* int posOrNeg;
    switch (currentPos) {
      case 0:
        posOrNeg = position - currentPos;
        break;
      case 1:
        while(!extendAtPos){
          extend.set(Constants.EXTEND_SPEED);
          if (!switch2.get()) {
            extend.set(0.0);
            break;
          }
        }
        break;

      case 2:
      while(!extendAtPos){
        extend.set(Constants.EXTEND_SPEED);
        if (!switch3.get()) {
          extend.set(0.0);
        break;
      }
    }
      break;

      case 3:
      while(!extendAtPos){
        extend.set(Constants.EXTEND_SPEED);
        if (!switch4.get()) {
          extend.set(0.0);
        break;
      }
    }
      break;
    }
    extendAtPos = true;*/
  }
}
