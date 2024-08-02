// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

/** Add your docs here. */
public interface ShooterIO {
    @AutoLog
    public static class ShooterIOInputs {
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

    public abstract void setLeftVoltage(double volts);

    public abstract void setRightVoltage(double volts);

    public abstract void setRPM(int rpm, SimpleMotorFeedforward ff, double differential);

    public abstract void setLeftRPM(int rpm, SimpleMotorFeedforward ff);

    public abstract void setRightRPM(int rpm, SimpleMotorFeedforward ff);

    public abstract void setShooterPID(double kP, double kI, double kD);
    
} 
