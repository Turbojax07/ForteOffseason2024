// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double climberPositionMeters = 0.0;
        public double climberTargetMeters = 0.0;
        public double climberVelocityMetersPerSecond = 0.0;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
        public double climberTempCelsius = 0.0;
    }

    public abstract void processInputs(final ClimberIOInputsAutoLogged inputs);

    public abstract void setTargetMeters(final double meters);

    public abstract void setVoltage(final double voltage);

    public abstract void stop();

    public abstract void setPID(double kP, double kI, double kD);

    public void setSimpleFF(double kFF);

    public void setFF(double kS, double kG, double kV, double kA);

    public abstract void resetEncoder(final double position);

    public abstract void resetEncoder();
    
}