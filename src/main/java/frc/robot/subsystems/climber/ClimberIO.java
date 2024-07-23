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
        public double climberVelocityMetersPerSecond = 0.0;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
        public double climberTempCelsius = 0.0;
    }

    public default void updateInputs(final ClimberIOInputsAutoLogged inputs) {}

    public default void setTarget(final double meters) {}

    public default void setVoltage(final double voltage) {}

    public default void stop() {
        setVoltage(0);
    }

    public default void resetEncoder(final double position) {}

    public default void resetEncoder() {
        resetEncoder(0.0);
    }
    
}