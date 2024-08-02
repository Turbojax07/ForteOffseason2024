package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;

public class ShooterIOSim implements ShooterIO {
    private FlywheelSim leftSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, ShooterConstants.shooterMOI);
    private FlywheelSim rightSim = new FlywheelSim(DCMotor.getNEO(1), 1.0, ShooterConstants.shooterMOI);

    private PIDController leftPID = new PIDController(ShooterConstants.kPShooterSim, 0, 0);
    private PIDController rightPID = new PIDController(ShooterConstants.kPShooterSim, 0, 0);

	@Override
	public void processInputs(ShooterIOInputsAutoLogged inputs) {
		leftSim.update(Constants.loopPeriodSecs);
		rightSim.update(Constants.loopPeriodSecs);

        inputs.leftSpeedRPM = leftSim.getAngularVelocityRPM();
        inputs.leftCurrentAmps = leftSim.getCurrentDrawAmps();

        inputs.rightSpeedRPM = rightSim.getAngularVelocityRPM();
		inputs.rightCurrentAmps = rightSim.getCurrentDrawAmps();
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
	public void setRPM(int rpm, SimpleMotorFeedforward ff, double differential) {
		setLeftRPM(rpm, ff);
		setRightRPM((int) (rpm * differential), ff);
	}

	@Override
	public void setLeftRPM(int rpm, SimpleMotorFeedforward ff) {
		setLeftVoltage(ff.calculate(rpm));
	}

	@Override
	public void setRightRPM(int rpm, SimpleMotorFeedforward ff) {
		setRightVoltage(ff.calculate(rpm));

	}

	@Override
	public void setShooterPID(double kP, double kI, double kD) {
		leftPID = new PIDController(kP, kI, kD);
		rightPID = new PIDController(kP, kI, kD);
	}

}
