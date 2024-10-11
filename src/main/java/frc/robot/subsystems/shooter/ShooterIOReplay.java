// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class ShooterIOReplay implements ShooterIO {

  @Override
  public void processInputs(ShooterIOInputsAutoLogged inputs) {}

  @Override
  public void setLeftVoltage(double volts) {}

  @Override
  public void setRightVoltage(double volts) {}

  @Override
  public void setRPM(int rpm, SimpleMotorFeedforward ff, double differential) {}

  @Override
  public void setLeftRPM(int rpm, SimpleMotorFeedforward ff) {}

  @Override
  public void setRightRPM(int rpm, SimpleMotorFeedforward ff) {}

  @Override
  public void setShooterPID(double kP, double kI, double kD) {}
}
