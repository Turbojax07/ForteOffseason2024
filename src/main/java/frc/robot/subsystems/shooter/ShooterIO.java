// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
        public Rotation2d pivotPosition = new Rotation2d();
        public double pivotRelativeEncoder = 0.0;
        public Rotation2d pivotTargetPosition = new Rotation2d();
        public double pivotVelocityRadPerSec = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public double leftSpeedRPM = 0.0;
        public double leftTargetRPM = 0.0;
        public double leftAppliedVolts = 0.0;
        public double leftCurrentAmps = 0.0;
        public double leftTempCelsius = 0.0;

        public double rightSpeedRPM = 0.0;
        public double rightTargetRPM = 0.0;
        public double rightAppliedVolts = 0.0;
        public double rightCurrentAmps = 0.0;
        public double rightTempCelsius = 0.0;
    }

    public abstract void processInputs(final ShooterIOInputsAutoLogged inputs);

    public abstract void setPivotVoltage(double volts);

    public abstract void setLeftVoltage(double volts);

    public abstract void setRightVoltage(double volts);

    public abstract void setPivotTarget(double angle, ArmFeedforward ff);

    public abstract void setLeftRPM(int rpm, SimpleMotorFeedforward ff);

    public abstract void setRightRPM(int rpm, SimpleMotorFeedforward ff);

    public abstract void setPivotPID(double kP, double kI, double kD);

    public abstract void setShooterPID(double kP, double kI, double kD);

    
} 
