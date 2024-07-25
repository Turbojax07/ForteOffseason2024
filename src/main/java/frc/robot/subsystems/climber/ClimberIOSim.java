package frc.robot.subsystems.climber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

import static frc.robot.Constants.*;

public class ClimberIOSim implements ClimberIO {
   private ElevatorSim sim = new ElevatorSim(
    DCMotor.getNEO(1),
    ClimberConstants.gearRatio,
    1,
    ClimberConstants.spoolRadius,
    ClimberConstants.minHeight,
    ClimberConstants.maxHeight,
    true, 
    0
   );

   private double appliedVolts = 0.0;
   private ProfiledPIDController pid = new ProfiledPIDController(
    ClimberConstants.kPSim, 
    ClimberConstants.kISim, 
    ClimberConstants.kDSim, 
    new Constraints(
        ClimberConstants.maxVelocity, 
        ClimberConstants.maxAccel
        )
        );
    private ElevatorFeedforward ff = new ElevatorFeedforward(
        ClimberConstants.kSSim, 
        ClimberConstants.kGSim, 
        ClimberConstants.kVSim, 
        ClimberConstants.kASim
        );

   @Override
   public void updateInputs(final ClimberIOInputsAutoLogged inputs) {
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
        pid.calculate(sim.getPositionMeters(), meters)
        + ff.calculate(pid.getSetpoint().velocity)
       );
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
public void stop() {
    // TODO Auto-generated method stub
}

@Override
public void setPID(double kP, double kI, double kD) {
    // TODO Auto-generated method stub
}


@Override
public void resetEncoder() {
    // TODO Auto-generated method stub
}

@Override
public void setSimpleFF(double kFF) {
    // TODO Auto-generated method stub
}

@Override
public void setFF(double kS, double kG, double kV, double kA) {
    // TODO Auto-generated method stub
}

}