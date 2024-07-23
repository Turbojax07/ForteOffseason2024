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
    public double turnVelocityRadPerSec = 0.0;
    public double turnAppliedVolts = 0.0;
    public double[] turnCurrentAmps = new double[] {};

    public double[] odometryTimestamps = new double[] {};
    public double[] odometryDrivePositionsRad = new double[] {};
    public Rotation2d[] odometryTurnPositions = new Rotation2d[] {};
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ModuleIOInputsAutoLogged inputs) {}

  /** Run the drive motor at the specified voltage. */
  public default void runDriveVolts(double volts) {}

  /** Run the turn motor at the specified voltage. */
  public default void runTurnVolts(double volts) {}

  /** Run to drive velocity setpoint with feedforward */
  default void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {}

  /** Run to turn position setpoint */
  default void runTurnPositionSetpoint(double angleRads) {}

  /** Configure drive PID */
  default void setDrivePID(double kP, double kI, double kD) {}

  /** Configure turn PID */
  default void setTurnPID(double kP, double kI, double kD) {}

  /** Enable or disable brake mode on the drive motor. */
  public default void setDriveBrakeMode(boolean enable) {}

  /** Enable or disable brake mode on the turn motor. */
  public default void setTurnBrakeMode(boolean enable) {}

  /** Disable output to all motors */
  public default void stop() {}

  public default String getModuleName() { return null; }
}
