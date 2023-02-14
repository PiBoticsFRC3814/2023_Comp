// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Arm;
import java.util.function.DoubleSupplier;

public class DirectArmCommand extends CommandBase {
  /** Creates a new ArmCommand. */
  Arm m_arm;

  double armAngleSpeed;
  double armExtendSpeed;

  public DirectArmCommand(Arm arm, DoubleSupplier angleSpeed, DoubleSupplier extendSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_arm = arm;
    armAngleSpeed = applyDeadzone(angleSpeed.getAsDouble(), Constants.JOYSTICK_X_DEADZONE);
    armExtendSpeed = applyDeadzone(extendSpeed.getAsDouble(), Constants.JOYSTICK_Y_DEADZONE);
    addRequirements(arm);
  }

  private double applyDeadzone(double input, double deadzone) {
    if (Math.abs(input) < deadzone) return 0.0;
    double result = (Math.abs(input) - deadzone) / (1.0 - deadzone);
    return (input < 0.0 ? -result : result);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_arm.ArmDirectControl(armAngleSpeed, armExtendSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
