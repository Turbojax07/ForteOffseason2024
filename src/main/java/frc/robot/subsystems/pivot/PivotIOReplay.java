// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.ArmFeedforward;

/** Add your docs here. */
public class PivotIOReplay implements PivotIO {

    @Override
    public void processInputs(PivotIOInputsAutoLogged inputs) {}

    @Override
    public void setPivotVoltage(double volts) {}

    @Override
    public void setPivotTarget(double angle, ArmFeedforward ff) {}

    @Override
    public void setPivotPID(double kP, double kI, double kD) {}
    
} 
