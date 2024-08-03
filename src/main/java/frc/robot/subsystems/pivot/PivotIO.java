// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public Rotation2d pivotPosition = new Rotation2d();
        public double pivotRelativeEncoder = 0.0;
        public Rotation2d pivotTargetPosition = new Rotation2d();
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;
    }

    public abstract void processInputs(final PivotIOInputsAutoLogged inputs);

    public abstract void setPivotVoltage(double volts);

    public abstract void setPivotTarget(double angle, ArmFeedforward ff);

    public abstract void setPivotPID(double kP, double kI, double kD);
    
} 