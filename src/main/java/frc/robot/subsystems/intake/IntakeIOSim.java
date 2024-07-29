// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOSim implements IntakeIO {
	private FlywheelSim rollerSim = new FlywheelSim(DCMotor.getNEO(1), 1, IntakeConstants.rollerMOI);
	private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), IntakeConstants.pivotRatio, IntakeConstants.pivotMOI, Units.inchesToMeters(IntakeConstants.pivotLength), IntakeConstants.up, IntakeConstants.down, true, IntakeConstants.up);
	private ProfiledPIDController pivotPID = new ProfiledPIDController(IntakeConstants.kPPivotSim, 0.0, 0.0, new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocity, IntakeConstants.maxPivotAccel));
	private PIDController rollerPID = new PIDController(IntakeConstants.kPRollerSim, 0.0, 0);
	

	@Override
	public void processInputs(IntakeIOInputsAutoLogged inputs) {
		rollerSim.update(Constants.loopPeriodSecs);
		pivotSim.update(Constants.loopPeriodSecs);

		inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
		inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotSim.getVelocityRadPerSec());
		inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();

		inputs.rollerSpeedRPM = rollerSim.getAngularVelocityRPM();
		inputs.rollerCurrentAmps = rollerSim.getCurrentDrawAmps();
	}

	@Override
	public void setPivotTarget(double angle, ArmFeedforward ff) {
		setPivotVoltage(pivotPID.calculate(pivotSim.getAngleRads(), angle) + ff.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity));
	}

	@Override
	public void setRollerRPM(int rpm, SimpleMotorFeedforward ff) {
		setRollerVoltage(rollerPID.calculate(rollerSim.getAngularVelocityRadPerSec() / (Math.PI * 2), rpm) + ff.calculate(rpm));
	}

	@Override
	public void setPivotPID(double kP, double kI, double kD) {
		pivotPID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocity, IntakeConstants.maxPivotAccel));
	}

	@Override
	public void setRollerPID(double kP, double kI, double kD) {
		rollerPID = new PIDController(kP, kI, kD);
	}

	@Override
	public void setPivotVoltage(double volts) {
		pivotSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setRollerVoltage(double volts) {
		rollerSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
	}

	}
