package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.util.LoggedTunableNumber;

import static frc.robot.Constants.*;

public class ClimberIOSparkMax implements ClimberIO {
    private final CANSparkMax motor = new CANSparkMax(RobotMap.Climber.climber, MotorType.kBrushless);
    private final RelativeEncoder encoder = motor.getEncoder();
    private LoggedTunableNumber kP = new LoggedTunableNumber("Climber/kP", ClimberConstants.kPReal);
    private LoggedTunableNumber kD = new LoggedTunableNumber("Climber/kD", ClimberConstants.kDReal);
    private LoggedTunableNumber kFF = new LoggedTunableNumber("Climber/kFF", ClimberConstants.kFFReal);
    
    private final SparkPIDController pid = motor.getPIDController();

    public ClimberIOSparkMax() {

        motor.restoreFactoryDefaults();
        motor.setCANTimeout(250);
        motor.setInverted(false);
        motor.enableVoltageCompensation(12.0);
        motor.setSmartCurrentLimit(30);

        encoder.setPositionConversionFactor(ClimberConstants.encoderConversion);
        encoder.setVelocityConversionFactor(ClimberConstants.encoderConversion / 60);

        pid.setP(kP.get());
        pid.setD(kD.get());
        pid.setFF(kFF.get());
        pid.setOutputRange(-1, 1);

        motor.burnFlash();
    }

    @Override
    public void updateInputs(final ClimberIOInputsAutoLogged inputs) {
        inputs.climberPositionMeters = encoder.getPosition();
        inputs.climberVelocityMetersPerSecond = encoder.getVelocity();
        inputs.climberAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput(); 
        inputs.climberCurrentAmps = motor.getOutputCurrent();
        inputs.climberTempCelsius = motor.getMotorTemperature();
    }

    @Override
    public void setTarget(final double meters) {
        pid.setReference(meters, ControlType.kPosition);
    }

    @Override
    public void setVoltage(final double voltage) {
        motor.setVoltage(voltage);
    }

    @Override
    public void resetEncoder(final double position) {
        encoder.setPosition(position);
    }
}