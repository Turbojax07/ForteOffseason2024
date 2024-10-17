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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SimConstants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private final String name;

  private DCMotorSim driveSim =
      new DCMotorSim(DCMotor.getKrakenX60(1), DriveConstants.driveRatio, DriveConstants.driveMOI);
  private DCMotorSim turnSim =
      new DCMotorSim(DCMotor.getNEO(1), DriveConstants.turnRatio, DriveConstants.turnMOI);

  private final PIDController driveFeedback =
      new PIDController(0.0, 0.0, 0.0, SimConstants.loopTime);
  private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0, 0);

  private final PIDController turnFeedback =
      new PIDController(0.0, 0.0, 0.0, SimConstants.loopTime);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;
  private final Rotation2d turnAbsoluteInitPosition = new Rotation2d(Math.random() * 2.0 * Math.PI);

  private boolean driveCoast = false;
  private boolean turnCoast = false;

  public ModuleIOSim(final String name) {
    this.name = name;
  }

  @Override
  public void processInputs(ModuleIOInputsAutoLogged inputs) {
    driveSim.update(SimConstants.loopTime);
    turnSim.update(SimConstants.loopTime);

    inputs.drivePositionRad = driveSim.getAngularPositionRad();
    inputs.driveVelocityRadPerSec = driveSim.getAngularVelocityRadPerSec();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = new double[] {Math.abs(driveSim.getCurrentDrawAmps())};

    inputs.turnAbsolutePosition =
        new Rotation2d(turnSim.getAngularPositionRad()).plus(turnAbsoluteInitPosition);
    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = new double[] {Math.abs(turnSim.getCurrentDrawAmps())};

    inputs.odometryTimestamps = new double[] {Timer.getFPGATimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void runDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void runTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }

  @Override
  public void runDriveVelocitySetpoint(double velocityRadsPerSec, double feedForward) {
    runDriveVoltage(
        //     feedForward
        //         + driveFeedback.calculate(driveSim.getAngularVelocityRadPerSec(),
        // velocityRadsPerSec));
        Math.signum(velocityRadsPerSec)
            * Math.pow((velocityRadsPerSec / DriveConstants.maxLinearVelocity), 2)
            * 12.0);
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    runTurnVoltage(turnFeedback.calculate(turnSim.getAngularPositionRad(), angleRads));
  }

  @Override
  public void setDrivePIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    driveFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnFeedback.setPID(kP, kI, kD);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveCoast = !enable;
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnCoast = !enable;
  }

  @Override
  public void stop() {
    runDriveVoltage(0.0);
    runTurnVoltage(0.0);
  }

  @Override
  public String getModuleName() {
    return name;
  }
}
