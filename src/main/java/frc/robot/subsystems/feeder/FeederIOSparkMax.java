package frc.robot.subsystems.feeder;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.RobotMap;

public class FeederIOSparkMax implements FeederIO {
  private final CANSparkMax feeder = new CANSparkMax(RobotMap.Intake.feeder, MotorType.kBrushless);
  private final RelativeEncoder enc = feeder.getEncoder();
  private final SparkPIDController pid = feeder.getPIDController();

  public FeederIOSparkMax() {
    feeder.restoreFactoryDefaults();

    feeder.setSmartCurrentLimit(FeederConstants.currentLimit);
    feeder.setInverted(true);

    feeder.setIdleMode(IdleMode.kBrake);

    enc.setVelocityConversionFactor(1.0);

    feeder.burnFlash();
  }

  @Override
  public void processInputs(FeederIOInputsAutoLogged inputs) {
    inputs.speedRPM = enc.getVelocity();
    inputs.appliedVolts = feeder.getAppliedOutput() * feeder.getBusVoltage();
    inputs.currentAmps = feeder.getOutputCurrent();
    inputs.tempCelsius = feeder.getMotorTemperature();
  }

  @Override
  public void setVoltage(double volts) {
    feeder.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setRPM(int rpm, SimpleMotorFeedforward ff) {
    rpm = MathUtil.clamp(rpm, 0, 5880);
    pid.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    pid.setP(kP);
    pid.setI(kI);
    pid.setD(kD);
    feeder.burnFlash();
  }
}
