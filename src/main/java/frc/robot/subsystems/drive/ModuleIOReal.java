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

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.RobotMap;
import org.littletonrobotics.junction.Logger;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOReal implements ModuleIO {
  private final String name;

  private final TalonFX driveTalon;
  private final TalonFXConfiguration driveConfig = new TalonFXConfiguration();
  private final CANSparkMax turnSparkMax;

  private final boolean driveInverted;
  private final boolean turnInverted;
  private final RelativeEncoder turnRelativeEncoder;
  private final AnalogEncoder absoluteEncoder;
  private final SparkPIDController turnPID;

  // private final Queue<Double> timestampQueue;

  private final StatusSignal<Double> drivePosition;
  // private final Queue<Double> drivePositionQueue;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrent;
  private int multiplier;

  private int iLoveRev = 0;

  private final VelocityVoltage driveCurrentVelocity =
      new VelocityVoltage(0.0).withEnableFOC(false);

  // private final Queue<Double> turnPositionQueue;

  private Rotation2d absoluteEncoderOffset;

  public ModuleIOReal(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(RobotMap.Drive.frontLeftDrive);
        driveInverted = RobotMap.Drive.frontLeftDriveInvert;
        turnSparkMax = new CANSparkMax(RobotMap.Drive.frontLeftTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.frontLeftTurnInvert;
        absoluteEncoder = new AnalogEncoder(RobotMap.Drive.frontLeftEncoder);
        absoluteEncoderOffset =
            new Rotation2d(RobotMap.Drive.frontLeftOffset); // MUST BE CALIBRATED
        name = "FrontLeft";
        multiplier = 1;
        break;
      case 1:
        driveTalon = new TalonFX(RobotMap.Drive.frontRightDrive);
        driveInverted = RobotMap.Drive.frontRightDriveInvert;
        turnSparkMax = new CANSparkMax(RobotMap.Drive.frontRightTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.frontRightTurnInvert;
        absoluteEncoder = new AnalogEncoder(RobotMap.Drive.frontRightEncoder);
        absoluteEncoderOffset =
            new Rotation2d(RobotMap.Drive.frontRightOffset); // MUST BE CALIBRATED
        name = "FrontRight";
        multiplier = 1;
        break;
      case 2:
        driveTalon = new TalonFX(RobotMap.Drive.backLeftDrive);
        driveInverted = RobotMap.Drive.backLeftDriveInvert;
        turnSparkMax = new CANSparkMax(RobotMap.Drive.backLeftTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.backLeftTurnInvert;
        absoluteEncoder = new AnalogEncoder(RobotMap.Drive.backLeftEncoder);
        absoluteEncoderOffset = new Rotation2d(RobotMap.Drive.backLeftOffset); // MUST BE CALIBRATED
        name = "BackLeft";
        multiplier = 1;
        break;
      case 3:
        driveTalon = new TalonFX(RobotMap.Drive.backRightDrive);
        driveInverted = RobotMap.Drive.backRightDriveInvert;
        turnSparkMax = new CANSparkMax(RobotMap.Drive.backRightTurn, MotorType.kBrushless);
        turnInverted = RobotMap.Drive.backRightTurnInvert;
        absoluteEncoder = new AnalogEncoder(RobotMap.Drive.backRightEncoder);
        absoluteEncoderOffset =
            new Rotation2d(RobotMap.Drive.backRightOffset); // MUST BE CALIBRATED
        name = "BackRight";
        multiplier = 1;
        break;
      default:
        throw new RuntimeException("Invalid module index");
    }

    driveConfig.CurrentLimits.SupplyCurrentLimit = DriveConstants.driveSupplyCurrent;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveConfig.CurrentLimits.StatorCurrentLimit = DriveConstants.driveStatorCurrent;
    driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;

    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.PeakForwardDutyCycle = 1.0;
    driveConfig.MotorOutput.PeakReverseDutyCycle = -1.0;

    driveConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    driveConfig.Feedback.SensorToMechanismRatio = DriveConstants.driveConversion;
    driveTalon.setPosition(0);

    driveConfig.MotionMagic.MotionMagicCruiseVelocity = DriveConstants.maxLinearVelocity;
    driveConfig.MotionMagic.MotionMagicAcceleration = DriveConstants.maxLinearAccel;
    driveConfig.MotionMagic.MotionMagicJerk = DriveConstants.maxLinearVelocity / 0.1;

    driveConfig.Slot0.kP = DriveConstants.kPDriveReal;
    driveConfig.Slot0.kI = 0.0;
    driveConfig.Slot0.kD = DriveConstants.kDDriveReal;

    driveConfig.Slot0.kS = DriveConstants.kSDriveReal;
    driveConfig.Slot0.kV = DriveConstants.kVDriveReal;
    driveConfig.Slot0.kA = DriveConstants.kADriveReal;

    driveTalon.setInverted(driveInverted);

    driveTalon.getConfigurator().apply(driveConfig);
    setDriveBrakeMode(true);

    turnRelativeEncoder = turnSparkMax.getEncoder();

    absoluteEncoder.setDistancePerRotation(2 * Math.PI);
    turnPID = turnSparkMax.getPIDController();

    turnSparkMax.restoreFactoryDefaults();
    turnSparkMax.setCANTimeout(250);

    for (int i = 0; i < 30; i++) {
      turnPID.setFeedbackDevice(turnRelativeEncoder);
      turnRelativeEncoder.setPositionConversionFactor(DriveConstants.turnConversion);
      turnRelativeEncoder.setVelocityConversionFactor((2 * Math.PI) / 60.0);

      turnSparkMax.setInverted(turnInverted);
      turnSparkMax.setSmartCurrentLimit(30);
      turnSparkMax.enableVoltageCompensation(12.0);

      turnRelativeEncoder.setMeasurementPeriod(10);
      turnRelativeEncoder.setAverageDepth(2);

      turnSparkMax.setPeriodicFramePeriod(
          PeriodicFrame.kStatus2, (int) (1000.0 / DriveConstants.odometeryFrequency));
    }

    turnRelativeEncoder.setPosition(getAbsoluteEncoder());

    turnPID.setPositionPIDWrappingEnabled(true);
    turnPID.setPositionPIDWrappingMinInput(-Math.PI);
    turnPID.setPositionPIDWrappingMaxInput(Math.PI);

    turnSparkMax.burnFlash();
    turnSparkMax.setCANTimeout(0);

    // timestampQueue = HybridOdometryThread.getInstance().makeTimestampQueue();

    drivePosition = driveTalon.getPosition();
    // drivePositionQueue =
    //     HybridOdometryThread.getInstance().registerSignal(driveTalon, driveTalon.getPosition());
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrent = driveTalon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(DriveConstants.odometeryFrequency, drivePosition);
    BaseStatusSignal.setUpdateFrequencyForAll(50.0, driveVelocity, driveAppliedVolts, driveCurrent);
    driveTalon.optimizeBusUtilization();

    // turnPositionQueue =
    //     HybridOdometryThread.getInstance()
    //         .registerSignal(
    //             () -> {
    //               double value = turnRelativeEncoder.getPosition();
    //               if (turnSparkMax.getLastError() == REVLibError.kOk) {
    //                 return OptionalDouble.of(value);
    //               } else {
    //                 return OptionalDouble.empty();
    //               }
    //             });
  }

  @Override
  public void processInputs(ModuleIOInputsAutoLogged inputs) {
    if (iLoveRev % 25 == 0) {
      turnRelativeEncoder.setPosition(getAbsoluteEncoder());
    }

    BaseStatusSignal.refreshAll(drivePosition, driveVelocity, driveAppliedVolts, driveCurrent);

    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble()) / DriveConstants.driveRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble()) / DriveConstants.driveRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = new double[] {driveCurrent.getValueAsDouble()};

    inputs.turnAbsolutePosition = Rotation2d.fromRadians(absoluteEncoder.getAbsolutePosition());
    inputs.turnPosition = Rotation2d.fromRadians(normalizeAngle(turnRelativeEncoder.getPosition()));
    inputs.turnVelocityRadPerSec = turnRelativeEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    // inputs.odometryTimestamps =
    //     timestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    // inputs.odometryDrivePositionsRad =
    //     drivePositionQueue.stream()
    //         .mapToDouble(
    //             (Double value) -> Units.rotationsToRadians(value) / DriveConstants.driveRatio)
    //         .toArray();
    // inputs.odometryTurnPositions =
    //     turnPositionQueue.stream()
    //         .map((Double value) -> Rotation2d.fromRotations(value / DriveConstants.turnRatio))
    //         .toArray(Rotation2d[]::new);
    // timestampQueue.clear();
    // drivePositionQueue.clear();
    // turnPositionQueue.clear();

    Logger.recordOutput("Test/" + name + "/GetAbsoluteEncoder", getAbsoluteEncoder());
    iLoveRev++;
  }

  @Override
  public void runDriveVoltage(double volts) {
    driveTalon.setControl(new VoltageOut(MathUtil.clamp(volts, -12, 12)));
  }

  @Override
  public void runTurnVoltage(double volts) {
    turnSparkMax.setVoltage(MathUtil.clamp(volts, -12, 12));
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    var config = new MotorOutputConfigs();
    config.Inverted = InvertedValue.CounterClockwise_Positive;
    config.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(config);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public String getModuleName() {
    return name;
  }

  @Override
  public void runDriveVelocitySetpoint(
      final double metersPerSecond, final double metersPerSecondSquared) {
    // Doesnt actually refresh drive velocity signal, but should be cached
    if (metersPerSecond == 0
        && metersPerSecondSquared == 0
        && MathUtil.isNear(0.0, driveVelocity.getValueAsDouble(), 0.1)) {
      runDriveVoltage(0.0);
    } else {
      runDriveVoltage(
          Math.signum(metersPerSecond)
              * Math.pow((metersPerSecond / DriveConstants.maxLinearVelocity), 2)
              * 12.0
              * multiplier);
    }
  }

  @Override
  public void runTurnPositionSetpoint(double angleRads) {
    // inputs.targetPosition = getAdjustedAngle(angleRads);
    turnPID.setReference(angleRads, ControlType.kPosition);
  }

  @Override
  public void setDrivePIDFF(double kP, double kI, double kD, double kS, double kV, double kA) {
    driveConfig.Slot0.kP = kP;
    driveConfig.Slot0.kI = kI;
    driveConfig.Slot0.kD = kD;
    driveConfig.Slot0.kS = kS;
    driveConfig.Slot0.kV = kV;
    driveConfig.Slot0.kA = kA;
    driveTalon.getConfigurator().apply(driveConfig, 0.01);
  }

  @Override
  public void setTurnPID(double kP, double kI, double kD) {
    turnPID.setP(kP);
    turnPID.setI(kI);
    turnPID.setD(kD);
    turnSparkMax.burnFlash();
  }

  public double getAbsoluteEncoder() {
    return normalizeAngle(
        (absoluteEncoder.getAbsolutePosition() * 2 * Math.PI) - absoluteEncoderOffset.getRadians());
  }

  public double normalizeAngle(double radians) {
    return (radians + Math.PI) % (2 * Math.PI) - Math.PI;
  }

  @Override
  public void stop() {
    var driveRequest = driveTalon.getAppliedControl();
    if (driveRequest instanceof VoltageOut) {
      driveTalon.setControl(new NeutralOut());
    }
    runTurnVoltage(0);
  }
}
