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
	public void resetEncoder() {
	}
}