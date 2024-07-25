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

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.util.Alert;
import frc.util.LoggedTunableNumber;

import org.littletonrobotics.junction.Logger;

public class Module {
  public record ModuleConstants(
      String prefix, int driveID, int turnID, int cancoderID, Rotation2d cancoderOffset) {}

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  // private SimpleMotorFeedforward driveFeedforward;
  
  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double velocitySetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private static final String[] moduleNames = new String[] {"FL", "FR", "BL", "BR"};
  
  private LoggedTunableNumber kPDrive = new LoggedTunableNumber("Drive/kPDrive");
  private LoggedTunableNumber kDDrive = new LoggedTunableNumber("Drive/kDDrive");
  private LoggedTunableNumber kSDrive = new LoggedTunableNumber("Drive/kSDrive");
  private LoggedTunableNumber kVDrive = new LoggedTunableNumber("Drive/kVDrive");
  private LoggedTunableNumber kADrive = new LoggedTunableNumber("Drive/kADrive");

  private LoggedTunableNumber kPTurn = new LoggedTunableNumber("Drive/kPTurn");
  private LoggedTunableNumber kDTurn = new LoggedTunableNumber("Drive/kDTurn");

  private final Alert driveMotorDisconnected;
  private final Alert turnMotorDisconnected;

  private SwerveModuleState lastSetpoint = new SwerveModuleState();

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    driveMotorDisconnected =
        new Alert(moduleNames[index] + " drive motor disconnected!", Alert.AlertType.WARNING);
    turnMotorDisconnected =
        new Alert(moduleNames[index] + " turn motor disconnected!", Alert.AlertType.WARNING);

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    switch (Constants.currentMode) {
      case REAL:
        kPDrive.initDefault(DriveConstants.kPDriveReal);
        kDDrive.initDefault(DriveConstants.kDDriveReal);
        kSDrive.initDefault(DriveConstants.kSDriveReal);
        kVDrive.initDefault(DriveConstants.kVDriveReal);
        kADrive.initDefault(DriveConstants.kADriveReal);

        kPTurn.initDefault(DriveConstants.kPTurnReal);
        kDTurn.initDefault(DriveConstants.kDTurnReal);
        break;

      case REPLAY:
        kPDrive.initDefault(DriveConstants.kPDriveReplay);
        kDDrive.initDefault(DriveConstants.kDDriveReplay);
        kSDrive.initDefault(DriveConstants.kSDriveReplay);
        kVDrive.initDefault(DriveConstants.kVDriveReplay);
        kADrive.initDefault(DriveConstants.kADriveReplay);

        kPTurn.initDefault(DriveConstants.kPTurnReplay);
        kDTurn.initDefault(DriveConstants.kDTurnReplay);
        break;

      case SIM:
        kPDrive.initDefault(DriveConstants.kPDriveSim);
        kDDrive.initDefault(DriveConstants.kDDriveSim);
        kSDrive.initDefault(DriveConstants.kSDriveSim);
        kVDrive.initDefault(DriveConstants.kVDriveSim);
        kADrive.initDefault(DriveConstants.kADriveSim);

        kPTurn.initDefault(DriveConstants.kPTurnSim);
        kDTurn.initDefault(DriveConstants.kDTurnSim);
        break;

      default:
        kPDrive.initDefault(0.0);
        kDDrive.initDefault(0.0);
        kSDrive.initDefault(0.0);
        kVDrive.initDefault(0.0);
        kADrive.initDefault(0.0);

        kPTurn.initDefault(0.0);
        kDTurn.initDefault(0.0);
        break;
    }    

    io.setDrivePIDFF(kPDrive.get(), 0, kDDrive.get(), kSDrive.get(), kVDrive.get(), kADrive.get());
    io.setTurnPID(kPTurn.get(), 0, kDTurn.get());
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);

    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setDrivePIDFF(kPDrive.get(), 0, kDDrive.get(), kSDrive.get(), kVDrive.get(), kADrive.get()), kPDrive, kDDrive, kSDrive, kVDrive, kADrive);
    LoggedTunableNumber.ifChanged(
        hashCode(), () -> io.setTurnPID(kPTurn.get(), 0, kDTurn.get()), kPTurn, kDTurn);

    driveMotorDisconnected.set(!inputs.driveMotorConnected);
    turnMotorDisconnected.set(!inputs.turnMotorConnected);
  }

  public void periodic() {
    Logger.processInputs(String.format("Drive/%s Module", io.getModuleName()), inputs);
    Logger.recordOutput(
        String.format("Drive/%s Module/Voltage Available", io.getModuleName()),
        Math.abs(inputs.driveAppliedVolts - RoboRioDataJNI.getVInVoltage()));



    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.wheelRadius;
      Rotation2d angle =
          inputs.odometryTurnPositions[i].plus(
              turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState runSetpoint(SwerveModuleState setpoint) {

    final var optimizedState = SwerveModuleState.optimize(setpoint, getAngle());
    inputs.targetPosition = optimizedState.angle;
    io.runTurnPositionSetpoint(optimizedState.angle.getRadians());
    io.runDriveVelocitySetpoint(optimizedState.speedMetersPerSecond * Math.cos(optimizedState.angle.minus(inputs.turnPosition).getRadians()), (optimizedState.speedMetersPerSecond - lastSetpoint.speedMetersPerSecond) / 0.02);

    angleSetpoint = optimizedState.angle;
  
    velocitySetpoint = optimizedState.speedMetersPerSecond;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.runDriveVolts(volts);
    velocitySetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.runTurnVolts(0.0);
    io.runDriveVolts(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    velocitySetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * DriveConstants.wheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * DriveConstants.wheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityRadPerSec;
  }
}
