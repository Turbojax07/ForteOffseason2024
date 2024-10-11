package frc.robot.subsystems.climber;

import static frc.robot.Constants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ClimbIOSim implements ClimbIO {
  private ElevatorSim sim =
      new ElevatorSim(
          DCMotor.getNEO(1),
          ClimbConstants.gearRatio,
          1,
          ClimbConstants.spoolRadius,
          ClimbConstants.minHeight,
          ClimbConstants.maxHeight,
          true,
          0);

  private double appliedVolts = 0.0;
  private ProfiledPIDController pid =
      new ProfiledPIDController(
          ClimbConstants.kPSim,
          ClimbConstants.kISim,
          ClimbConstants.kDSim,
          new Constraints(ClimbConstants.maxVelocity, ClimbConstants.maxAccel));

  private ElevatorFeedforward ff =
      new ElevatorFeedforward(
          ClimbConstants.kSSim, ClimbConstants.kGSim, ClimbConstants.kVSim, ClimbConstants.kASim);

  @Override
  public void processInputs(final ClimbIOInputsAutoLogged inputs) {
    if (DriverStation.isDisabled()) {
      stop();
    }

    sim.update(0.02);
    inputs.climberPositionMeters = sim.getPositionMeters();
    inputs.climberVelocityMetersPerSecond = sim.getVelocityMetersPerSecond();
    inputs.climberAppliedVolts = appliedVolts;
    inputs.climberCurrentAmps = sim.getCurrentDrawAmps();
  }

  @Override
  public void setTargetMeters(final double meters) {
    setVoltage(
        pid.calculate(sim.getPositionMeters(), meters) + ff.calculate(pid.getSetpoint().velocity));
  }

  public void setOpenLoopDutyCycle(final double dutyCycle) {
    setVoltage(dutyCycle * 12);
  }

  @Override
  public void setVoltage(final double voltage) {
    appliedVolts = voltage;
    sim.setInputVoltage(MathUtil.clamp(voltage, -12.0, 12.0));
  }

  @Override
  public void resetEncoder(final double position) {
    sim.setState(position, 0.0);
  }

  @Override
  public void stop() {}

  @Override
  public void setPID(double kP, double kI, double kD) {}

  @Override
  public void resetEncoder() {}

  @Override
  public void setSimpleFF(double kFF) {}

  @Override
  public void setFF(double kS, double kG, double kV, double kA) {}
}
