// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class PivotIOSparkMax implements PivotIO {
    private final CANSparkMax pivot = new CANSparkMax(RobotMap.Shooter.pivot, MotorType.kBrushless);
    private final RelativeEncoder pivotEnc = pivot.getEncoder();
    private final SparkAbsoluteEncoder pivotAbs = pivot.getAbsoluteEncoder(Type.kDutyCycle);
    private final SparkPIDController pivotPID = pivot.getPIDController();
    private final double offset;

    public PivotIOSparkMax() {

        pivot.restoreFactoryDefaults();

        pivot.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);

        pivot.setIdleMode(IdleMode.kCoast);
        pivotPID.setFeedbackDevice(pivotAbs);
		pivotPID.setOutputRange(-12, 12);
		pivotPID.setPositionPIDWrappingEnabled(false);
        pivotAbs.setInverted(true);
		pivot.setInverted(false);
        pivotAbs.setPositionConversionFactor(ShooterConstants.pivotAbsConversion);
        pivotAbs.setVelocityConversionFactor(ShooterConstants.pivotAbsConversion / 60.0);
        offset = pivotAbs.getPosition() - .19;

		pivotEnc.setPositionConversionFactor(ShooterConstants.pivotEncConversion);
        pivotEnc.setVelocityConversionFactor(ShooterConstants.pivotEncConversion / 60.0);
        pivotEnc.setPosition(pivotAbs.getPosition());

        pivot.burnFlash();
    }

	@Override
	public void processInputs(PivotIOInputsAutoLogged inputs) {
		inputs.pivotPosition = Rotation2d.fromRadians(pivotAbs.getPosition() - offset) ;
		inputs.pivotVelocityRadPerSec = pivotEnc.getVelocity();
		inputs.pivotAppliedVolts = pivot.getAppliedOutput() * pivot.getBusVoltage();
		inputs.pivotCurrentAmps = pivot.getOutputCurrent();
		inputs.pivotTempCelsius = pivot.getMotorTemperature();
	}

	@Override
	public void setPivotVoltage(double volts) {
		pivot.setVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setPivotTarget(double angle, ArmFeedforward ff) {
		pivotPID.setReference(angle, ControlType.kPosition, 0, ff.calculate(angle, 0));
	}

	@Override
	public void setPivotPID(double kP, double kI, double kD) {
		pivotPID.setP(kP);
		pivotPID.setI(kI);
		pivotPID.setD(kD);
		pivot.burnFlash();
		}

}