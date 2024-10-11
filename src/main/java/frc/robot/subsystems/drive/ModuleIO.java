// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    public boolean driveMotorConnected = true;
    public boolean turnMotorConnected = true;
    public boolean hasCurrentControl = false;

    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double[] driveCurrentAmps = new double[] {};

    public Rotation2d turnAbsolutePosition = new Rotation2d();
    public Rotation2d turnPosition = new Rotation2d();
    public Rotation2d targetPosition = new Rotation2d();
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public Rotation2d absoluteEncoderPosition = new Rotation2d();
    public double absoluteEncoderReadingVolts = 0.0;

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public abstract void processInputs(ModuleIOInputsAutoLogged inputs);

  /** Run the drive motor at the specified voltage. */
  public abstract void runDriveVoltage(double volts);

  /** Run the turn motor at the specified voltage. */
  public abstract void runTurnVoltage(double volts);

  /** Run to drive velocity setpoint with feedforward */
  public abstract void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward);

  /** Run to turn position setpoint */
  public abstract void runTurnPositionSetpoint(double angleRads);

  /** Configure drive PID */
  public abstract void setDrivePIDFF(
      double kP, double kI, double kD, double kS, double kV, double kA);

  /** Configure turn PID */
  public abstract void setTurnPID(double kP, double kI, double kD);

  /** Enable or disable brake mode on the drive motor. */
  public abstract void setDriveBrakeMode(boolean enable);

  /** Enable or disable brake mode on the turn motor. */
  public abstract void setTurnBrakeMode(boolean enable);

  /** Disable output to all motors */
  public abstract void stop();

  public abstract String getModuleName();
}
