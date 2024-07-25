package frc.robot.subsystems.climber;

public class ClimberIOReplay implements ClimberIO {
    public void updateInputs(final ClimberIOInputsAutoLogged inputs) {}

    public void setTargetMeters(final double meters) {}

    public void setVoltage(final double voltage) {}

    public void stop() {
        setVoltage(0);
    }

    public void setPID(double kP, double kI, double kD) {}

    public void setFF(double kFF) {}

    public void resetEncoder(final double position) {}

    public void resetEncoder() {
        resetEncoder(0.0);
    }

    @Override
    public void setSimpleFF(double kFF) {}

    @Override
    public void setFF(double kS, double kG, double kV, double kA) {}
}
