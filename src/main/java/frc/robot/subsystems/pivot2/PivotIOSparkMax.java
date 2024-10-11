// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot2;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class PivotIOSparkMax implements PivotIO {
  private final CANSparkMax pivot = new CANSparkMax(RobotMap.Shooter.pivot, MotorType.kBrushless);
  private final RelativeEncoder pivotEnc = pivot.getEncoder();
  private final ThroughboreEncoder pivotAbs;

  private Debouncer stallDebouncer =
      new Debouncer(ShooterConstants.stallTimeout, DebounceType.kRising);

  public PivotIOSparkMax() {
    pivot.restoreFactoryDefaults();
    pivotAbs =
        new ThroughboreEncoder(
            pivot.getAbsoluteEncoder(), pivot.getAbsoluteEncoder().getPosition());

    pivot.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);
    pivot.setIdleMode(IdleMode.kCoast);

    pivotAbs.abs.setInverted(true);
    pivot.setInverted(true);
    pivotAbs.abs.setPositionConversionFactor(ShooterConstants.pivotAbsConversion);
    pivotAbs.abs.setVelocityConversionFactor(ShooterConstants.pivotAbsConversion / 60.0);

    pivotEnc.setPositionConversionFactor(ShooterConstants.pivotEncConversion);
    pivotEnc.setVelocityConversionFactor(ShooterConstants.pivotEncConversion / 60.0);

    pivotEnc.setPosition(pivotAbs.abs.getPosition());

    pivot.burnFlash();

    if ((Math.abs(pivotAbs.getPosition()) % (2 * Math.PI)) > 0.1) {
      pivotAbs.setOffset(pivotAbs.getOffset() + pivotAbs.getPosition());
    }
  }

  @Override
  public void processInputs(PivotIOInputsAutoLogged inputs) {
    inputs.pivotPosition = Rotation2d.fromRadians(pivotAbs.getPosition());
    inputs.pivotAbsolutePosition = Rotation2d.fromRadians(pivotAbs.getPosition());
    inputs.pivotRelativeEncoder = Rotation2d.fromRadians(pivotEnc.getPosition());
    inputs.pivotVelocityRadPerSec = pivotEnc.getVelocity();
    inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
    inputs.pivotCurrentAmps = pivot.getOutputCurrent();
    inputs.pivotTempCelsius = pivot.getMotorTemperature();
    inputs.pivotOffset = pivotAbs.getOffset();
    inputs.pivotStalled =
        stallDebouncer.calculate(
            (pivot.getOutputCurrent() > 15) && (Math.abs(pivotEnc.getVelocity()) < 0.02));
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivot.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void resetEncoder() {
    pivotAbs.setOffset(pivotAbs.abs.getPosition());
  }
}
