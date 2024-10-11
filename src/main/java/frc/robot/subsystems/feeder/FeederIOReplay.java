package frc.robot.subsystems.feeder;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class FeederIOReplay implements FeederIO {

  @Override
  public void processInputs(FeederIOInputsAutoLogged inputs) {}

  @Override
  public void setVoltage(double volts) {}

  @Override
  public void setRPM(int rpm, SimpleMotorFeedforward ff) {}

  @Override
  public void setPID(double kP, double kI, double kD) {}
}
