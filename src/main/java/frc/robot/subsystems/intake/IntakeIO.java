// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs {
        public double pivotAngleDegrees = 0.0;
        public double pivotSetpointDegrees = 0.0;
        public double pivotVelocityDegreesPerSecond = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotCurrentAmps = 0.0;
        public double pivotTempCelsius = 0.0;

        public double rollerSpeedRPM = 0.0;
        public double rollerSetpointRPM = 0.0;
        public double rollerAppliedVolts = 0.0;
        public double rollerCurrentAmps = 0.0;
        public double rollerTempCelsius = 0.0;
    }

    public default void updateInputs(final IntakeIOInputsAutoLogged inputs) {} 

    public default void setRollerPercent(double percent) {}

    public default void setDeployAngle(double angle) {}

    public default void resetAngle() {}
}
