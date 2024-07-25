package frc.robot.subsystems.drive;

public class ModuleIOReplay implements ModuleIO {
    /** Updates the set of loggable inputs. */
  public void updateInputs(ModuleIOInputsAutoLogged inputs) {}

  /** Run the drive motor at the specified voltage. */
  public void runDriveVolts(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public void runTurnVolts(double volts) {}

  /** Run to drive velocity setpoint with feedforward */
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {}

  /** Run to turn position setpoint */
  public void runTurnPositionSetpoint(double angleRads) {}

  /** Configure drive PID */
  public void setDrivePIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {}

  /** Configure turn PID */
  public void setTurnPID(double kP, double kI, double kD) {}

  /** Enable or disable brake mode on the drive motor. */
  public void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public void setTurnBrakeMode(boolean enable) {}

  /** Disable output to all motors */
  public void stop() {}

  public String getModuleName() { return null; } 
}
