package frc.robot.subsystems.feeder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;
import frc.robot.Constants.FeederConstants;

public class FeederIOSim implements FeederIO {
    	private FlywheelSim sim = new FlywheelSim(DCMotor.getNEO(1), FeederConstants.ratio, FeederConstants.MOI);
    	private PIDController pid = new PIDController(FeederConstants.kPSim, 0.0, 0);


    @Override
    public void processInputs(FeederIOInputsAutoLogged inputs) {
        sim.update(Constants.loopPeriodSecs);

		inputs.speedRPM = sim.getAngularVelocityRPM();
		inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
       sim.setInputVoltage(MathUtil.clamp(volts, -12, 12));
    }

    @Override
    public void setRPM(int rpm, SimpleMotorFeedforward ff) {
        setVoltage(ff.calculate(rpm));
    }

    @Override
    public void setPID(double kP, double kI, double kD) {
        pid = new PIDController(kP, kI, kD);
    }
    
}
