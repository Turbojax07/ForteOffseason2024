package frc.robot.subsystems.climber;

import frc.robot.subsystems.climber.ClimbIOInputsAutoLogged;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ClimbIO {
    @AutoLog
    public static class ClimbIOInputs {
        public double climberPositionMeters = 0.0;
        public double climberTargetMeters = 0.0;
        public double climberVelocityMetersPerSecond = 0.0;
        public double climberAppliedVolts = 0.0;
        public double climberCurrentAmps = 0.0;
        public double climberTempCelsius = 0.0;
        public double climberTargetSpeed = 0.0;
    }

    public abstract void processInputs(final ClimbIOInputsAutoLogged inputs);

    public abstract void setTargetMeters(final double meters);

    public abstract void setVoltage(final double voltage);

    public abstract void setOpenLoopDutyCycle(final double dutyCycle);

    public abstract void stop();

    public abstract void setPID(double kP, double kI, double kD);

    public void setSimpleFF(double kFF);

    public void setFF(double kS, double kG, double kV, double kA);

    public abstract void resetEncoder(final double position);

    public abstract void resetEncoder();
    
}