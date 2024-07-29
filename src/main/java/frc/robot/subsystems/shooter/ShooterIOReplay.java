// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public class ShooterIOReplay implements ShooterIO {

    @Override
    public void processInputs(ShooterIOInputsAutoLogged inputs) {}

    @Override
    public void setPivotVoltage(double volts) {}

    @Override
	public void setFeederVoltage(double volts) {}

    @Override
    public void setLeftVoltage(double volts) {}

    @Override
    public void setRightVoltage(double volts) {}

    @Override
    public void setPivotTarget(double angle, ArmFeedforward ff) {}

    @Override
    public void setFeederRPM(int asInt, SimpleMotorFeedforward feederFF) {}

    @Override
    public void setLeftRPM(int rpm, SimpleMotorFeedforward ff) {}

    @Override
    public void setRightRPM(int rpm, SimpleMotorFeedforward ff) {}

    @Override
    public void setPivotPID(double kP, double kI, double kD) {}

    @Override
    public void setFeederPID(double kP, double kI, double kD) {}

    @Override
    public void setShooterPID(double kP, double kI, double kD) {}

    
    
} 
