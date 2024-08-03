// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants.RobotMap;

/** Add your docs here. */
public class ShooterIOSparkMax implements ShooterIO {
	private final CANSparkMax left = new CANSparkMax(RobotMap.Shooter.left, MotorType.kBrushless);
	private final RelativeEncoder leftEnc = left.getEncoder();
	private final SparkPIDController leftPID = left.getPIDController();

	private final CANSparkMax right = new CANSparkMax(RobotMap.Shooter.right, MotorType.kBrushless);
	private final RelativeEncoder rightEnc = right.getEncoder();
	private final SparkPIDController rightPID = right.getPIDController();

	public ShooterIOSparkMax() {
		left.restoreFactoryDefaults();
		right.restoreFactoryDefaults();

		left.setSmartCurrentLimit(40);
		right.setSmartCurrentLimit(40);

		left.setIdleMode(IdleMode.kCoast);
		right.setIdleMode(IdleMode.kCoast);

		left.setInverted(true);

		left.burnFlash();
		right.burnFlash();
	}

	@Override
	public void processInputs(ShooterIOInputsAutoLogged inputs) {
		inputs.leftSpeedRPM = leftEnc.getVelocity();
		inputs.leftAppliedVolts = left.getAppliedOutput() * left.getBusVoltage();
		inputs.leftCurrentAmps = left.getOutputCurrent();
		inputs.leftTempCelsius = left.getMotorTemperature();

		inputs.rightSpeedRPM = rightEnc.getVelocity();
		inputs.rightAppliedVolts = right.getAppliedOutput() * left.getBusVoltage();
		inputs.rightCurrentAmps = right.getOutputCurrent();
		inputs.rightTempCelsius = right.getMotorTemperature();
	}

	@Override
	public void setLeftVoltage(double volts) {
		left.setVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setRightVoltage(double volts) {
		right.setVoltage(MathUtil.clamp(volts, -12, 12));
	}

	public void setRPM(int rpm, SimpleMotorFeedforward ff, double differential) {
		setLeftRPM(rpm, ff);
		setRightRPM((int) (rpm * differential), ff);
	}

	@Override
	public void setLeftRPM(int rpm, SimpleMotorFeedforward ff) {
		rpm = MathUtil.clamp(rpm, -5880, 5880);
		leftPID.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);

	}

	@Override
	public void setRightRPM(int rpm, SimpleMotorFeedforward ff) {
		rpm = MathUtil.clamp(rpm, -5880, 5880);
		rightPID.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);

	}

	@Override
	public void setShooterPID(double kP, double kI, double kD) {
		leftPID.setP(kP);
		leftPID.setI(kI);
		leftPID.setD(kD);
		left.burnFlash();

		rightPID.setP(kP);
		rightPID.setI(kI);
		rightPID.setD(kD);
		right.burnFlash();
	}

}
