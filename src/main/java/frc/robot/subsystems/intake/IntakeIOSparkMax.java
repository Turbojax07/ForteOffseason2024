// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class IntakeIOSparkMax implements IntakeIO {

  private final CANSparkMax roller = new CANSparkMax(RobotMap.Intake.roller, MotorType.kBrushless);
  private final CANSparkMax pivot = new CANSparkMax(RobotMap.Intake.pivot, MotorType.kBrushless);
  private final RelativeEncoder rollerEnc = roller.getEncoder();
  private final RelativeEncoder pivotEnc = pivot.getEncoder();
  private final SparkAbsoluteEncoder pivotAbs = pivot.getAbsoluteEncoder(Type.kDutyCycle);

  private final SparkPIDController rollerPID = roller.getPIDController();
  private final SparkPIDController pivotPID = pivot.getPIDController();

  public IntakeIOSparkMax() {
    roller.restoreFactoryDefaults();
    pivot.restoreFactoryDefaults();

    roller.setSmartCurrentLimit(IntakeConstants.rollerCurrentLimit);
    pivot.setSmartCurrentLimit(IntakeConstants.pivotCurrentLimit);

    roller.setIdleMode(IdleMode.kCoast);
    pivot.setIdleMode(IdleMode.kCoast);

    rollerEnc.setVelocityConversionFactor(1.0);

    pivotPID.setFeedbackDevice(pivotAbs);
    pivotPID.setOutputRange(-12, 12);

    pivotPID.setPositionPIDWrappingEnabled(false);
    pivotAbs.setInverted(IntakeConstants.pivotInvert);
    pivot.setInverted(IntakeConstants.pivotInvert);
    pivotAbs.setPositionConversionFactor(IntakeConstants.pivotAbsConversion);
    pivotAbs.setVelocityConversionFactor(IntakeConstants.pivotAbsConversion / 60.0);
    pivotAbs.setZeroOffset(IntakeConstants.pivotOffset);

    pivotEnc.setPositionConversionFactor(IntakeConstants.pivotEncConversion);
    pivotEnc.setVelocityConversionFactor(IntakeConstants.pivotEncConversion / 60.0);
    pivotEnc.setPosition(pivotAbs.getPosition());

    roller.burnFlash();
    pivot.burnFlash();
  }

  @Override
  public void processInputs(IntakeIOInputsAutoLogged inputs) {
    inputs.pivotPosition = Rotation2d.fromRadians(pivotAbs.getPosition());
    inputs.pivotVelocityRadPerSec = pivotEnc.getVelocity();
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.pivotCurrentAmps = pivot.getOutputCurrent();
    inputs.pivotTempCelsius = pivot.getMotorTemperature();

    inputs.rollerSpeedRPM = rollerEnc.getVelocity();
    inputs.rollerAppliedVolts = roller.getAppliedOutput() * roller.getBusVoltage();
    inputs.rollerCurrentAmps = roller.getOutputCurrent();
    inputs.rollerTempCelsius = roller.getMotorTemperature();
  }

  public void setPivotTarget(double angle, ArmFeedforward ff) {
    pivotPID.setReference(angle, ControlType.kPosition, 0, ff.calculate(angle, 0));
  }

  @Override
  public void setRollerRPM(int rpm, SimpleMotorFeedforward ff) {
    rpm = MathUtil.clamp(rpm, -5880, 5880);
    rollerPID.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);
  }

  public void setPivotPID(double kP, double kI, double kD) {
    pivotPID.setP(kP);
    pivotPID.setI(kI);
    pivotPID.setD(kD);
    pivot.burnFlash();
  }

  public void setRollerPID(double kP, double kI, double kD) {
    rollerPID.setP(kP);
    rollerPID.setI(kI);
    rollerPID.setD(kD);
    roller.burnFlash();
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setRollerVoltage(double volts) {
    roller.setVoltage(MathUtil.clamp(volts, -12, 12));
  }
}
