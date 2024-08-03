package frc.robot.subsystems.pivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;

public class PivotIOSim implements PivotIO {
    private SingleJointedArmSim pivotSim = new SingleJointedArmSim(DCMotor.getNEO(1), ShooterConstants.pivotRatio, ShooterConstants.pivotMOI, Units.inchesToMeters(ShooterConstants.pivotLength), ShooterConstants.down, ShooterConstants.up, true, ShooterConstants.down);
    

    private ProfiledPIDController pivotPID = new ProfiledPIDController(ShooterConstants.kPPivotSim, 0.0, 0.0, new TrapezoidProfile.Constraints(ShooterConstants.maxPivotVelocity, ShooterConstants.maxPivotAccel));
    

	@Override
	public void processInputs(PivotIOInputsAutoLogged inputs) {
		pivotSim.update(Constants.loopPeriodSecs);
		
		inputs.pivotPosition = Rotation2d.fromRadians(pivotSim.getAngleRads());
        inputs.pivotVelocityRadPerSec = Units.rotationsToRadians(pivotSim.getVelocityRadPerSec());
        inputs.pivotCurrentAmps = pivotSim.getCurrentDrawAmps();
	}

	@Override
	public void setPivotVoltage(double volts) {
		pivotSim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
	}

	@Override
	public void setPivotTarget(double angle, ArmFeedforward ff) {
		setPivotVoltage(pivotPID.calculate(pivotSim.getAngleRads(), angle) + ff.calculate(pivotPID.getSetpoint().position, pivotPID.getSetpoint().velocity));

	}

	@Override
	public void setPivotPID(double kP, double kI, double kD) {
		pivotPID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(IntakeConstants.maxPivotVelocity, IntakeConstants.maxPivotAccel));

	}
}