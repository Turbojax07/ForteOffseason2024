// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class IntakeIOReplay implements IntakeIO {
  public void processInputs(final IntakeIOInputsAutoLogged inputs) {}

  @Override
  public void setPivotPID(double kP, double kI, double kD) {}

  @Override
  public void setRollerPID(double kP, double kI, double kD) {}

  @Override
  public void setPivotVoltage(double volts) {}

  @Override
  public void setRollerVoltage(double volts) {}

  @Override
  public void setPivotTarget(double angle, ArmFeedforward ff) {}

  public void setRollerRPM(int rpm, SimpleMotorFeedforward ff) {}
}
