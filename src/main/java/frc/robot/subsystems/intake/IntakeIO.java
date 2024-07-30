// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public Rotation2d pivotPosition = new Rotation2d();
        public Rotation2d pivotTargetPosition = new Rotation2d();
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public double rollerSpeedRPM = 0.0;
        public double rollerTargetRPM = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
    }

    public abstract void processInputs(final IntakeIOInputsAutoLogged inputs);

    public abstract void setPivotVoltage(double volts);

    public abstract void setRollerVoltage(double volts);

    public abstract void setPivotTarget(double angle, ArmFeedforward ff);

    public abstract void setRollerRPM(int rpm, SimpleMotorFeedforward ff);
    
    public abstract void setPivotPID(double kP, double kI, double kD);

    public abstract void setRollerPID(double kP, double kI, double kD);

}
