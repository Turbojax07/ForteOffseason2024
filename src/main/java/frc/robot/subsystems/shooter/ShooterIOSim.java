package frc.robot.subsystems.shooter;

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
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), ShooterConstants.pivotRatio, ShooterConstants.pivotMOI, Units.inchesToMeters(ShooterConstants.pivotLength), ShooterConstants.down, ShooterConstants.up, true, ShooterConstants.down);
    private FlywheelSim leftSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, ShooterConstants.shooterMOI);
    private FlywheelSim rightSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, ShooterConstants.shooterMOI);

    private ProfiledPIDController pivotPID = new ProfiledPIDController(ShooterConstants.kPPivotSim, 0.0, 0.0, new TrapezoidProfile.Constraints(ShooterConstants.maxPivotVelocity, ShooterConstants.maxPivotAccel));
    private PIDController leftPID = new PIDController(ShooterConstants.kPShooterSim, 0, 0);
    private PIDController rightPID = new PIDController(ShooterConstants.kPShooterSim, 0, 0);

	@Override
	public void processInputs(ShooterIOInputsAutoLogged inputs) {
		pivotSim.update(Constants.loopPeriodSecs);
		leftSim.update(Constants.loopPeriodSecs);
		rightSim.update(Constants.loopPeriodSecs);

		inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotSim.getVelocityRadPerSec());
        inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();

        inputs.leftSpeedRPM = leftSim.getAngularVelocityRPM();
        inputs.leftCurrentAmps = leftSim.getCurrentDrawAmps();

        inputs.rightSpeedRPM = rightSim.getAngularVelocityRPM();
		inputs.rightCurrentAmps = rightSim.getCurrentDrawAmps();
	}

	@Override
	public void setPivotVoltage(double volts) {
		pivotSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setLeftVoltage(double volts) {
		leftSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setRightVoltage(double volts) {
		rightSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setPivotTarget(double angle, ArmFeedforward ff) {
		setPivotVoltage(pivotPID.calculate(pivotSim.getAngleRads(), angle) + ff.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity));
	}
    
	@Override
	public void setLeftRPM(int rpm, SimpleMotorFeedforward ff) {
		setLeftVoltage(leftPID.calculate(leftSim.getAngularVelocityRadPerSec() / (Math.PI * 2), rpm) + ff.calculate(rpm));
	}

	@Override
	public void setRightRPM(int rpm, SimpleMotorFeedforward ff) {
		setRightVoltage(rightPID.calculate(rightSim.getAngularVelocityRadPerSec() / (Math.PI * 2), rpm) + ff.calculate(rpm));
	}

    @Override
    public void setPivotPID(double kP, double kI, double kD) {
        pivotPID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(ShooterConstants.maxPivotVelocity, ShooterConstants.maxPivotAccel));
    }

    @Override
    public void setShooterPID(double kP, double kI, double kD) {
        leftPID = new PIDController(kP, kI, kD);
		rightPID = new PIDController(kP, kI, kD);
    }


}
