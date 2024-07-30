package frc.robot.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
        public double speedRPM = 0.0;
        public double targetRPM = 0.0;
        public double appliedVolts = 0.0;
        public double currentAmps = 0.0;
        public double tempCelsius = 0.0;
    }
    
    public abstract void processInputs(final FeederIOInputsAutoLogged inputs);

    public abstract void setVoltage(double volts);

    public abstract void setRPM(int rpm, SimpleMotorFeedforward ff);

    public abstract void setPID(double kP, double kI, double kD);
}
