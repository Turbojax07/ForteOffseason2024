// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.ArbFFUnits;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.RobotMap;
import frc.robot.Constants.ShooterConstants;

/** Add your docs here. */
public class ShooterIOSparkMax implements ShooterIO {
    private final CANSparkMax pivot = new CANSparkMax(RobotMap.Shooter.pivot, MotorType.kBrushless);
    private final RelativeEncoder pivotEnc = pivot.getEncoder();
    private final SparkAbsoluteEncoder pivotAbs = pivot.getAbsoluteEncoder(Type.kDutyCycle);
    private final SparkPIDController pivotPID = pivot.getPIDController();
    
    private final CANSparkMax left = new CANSparkMax(RobotMap.Shooter.left, MotorType.kBrushless);
    private final RelativeEncoder leftEnc = left.getEncoder();
    private final SparkPIDController leftPID = left.getPIDController();

    private final CANSparkMax right = new CANSparkMax(RobotMap.Shooter.right, MotorType.kBrushless);
    private final RelativeEncoder rightEnc = right.getEncoder();
    private final SparkPIDController rightPID = right.getPIDController(); 

    public ShooterIOSparkMax() {
        pivot.restoreFactoryDefaults();
        left.restoreFactoryDefaults();
        right.restoreFactoryDefaults();

        pivot.setSmartCurrentLimit(ShooterConstants.pivotCurrentLimit);

        pivot.setIdleMode(IdleMode.kBrake);
        left.setIdleMode(IdleMode.kCoast);
        right.setIdleMode(IdleMode.kCoast);

        pivotPID.setFeedbackDevice(pivotAbs);
        pivotAbs.setInverted(true);
		pivot.setInverted(true);
        pivotAbs.setPositionConversionFactor(ShooterConstants.pivotAbsConversion);
        pivotAbs.setVelocityConversionFactor(ShooterConstants.pivotAbsConversion / 60.0);
        pivotAbs.setZeroOffset(ShooterConstants.pivotOffset);

        pivotEnc.setPosition(pivotAbs.getPosition());

        pivot.burnFlash();
        left.burnFlash();
        right.burnFlash();
    }

    @Override
    public void processInputs(ShooterIOInputsAutoLogged inputs) {
        inputs.pivotPosition = Rotation2d.fromRadians(pivotAbs.getPosition());
        inputs.pivotRelativeEncoder = pivotEnc.getPosition();
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
	public void setLeftVoltage(double volts) {
		left.setVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setRightVoltage(double volts) {
		right.setVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setPivotTarget(double angle, ArmFeedforward ff) {
		pivotPID.setReference(angle, ControlType.kPosition, 0, ff.calculate(angle, 0));
	}

	@Override
	public void setLeftRPM(int rpm, SimpleMotorFeedforward ff) {
		rpm = MathUtil.clamp(rpm, 0, 5880);
		leftPID.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);
	}

	@Override
	public void setRightRPM(int rpm, SimpleMotorFeedforward ff) {
		rpm = MathUtil.clamp(rpm, 0, 5880);
		rightPID.setReference(rpm, ControlType.kVelocity, 0, ff.calculate(rpm), ArbFFUnits.kVoltage);
	}

	@Override
	public void setPivotPID(double kP, double kI, double kD) {
		pivotPID.setP(kP);
		pivotPID.setI(kI);
		pivotPID.setD(kD);
		pivot.burnFlash();
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
