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

package frc.robot;

import java.io.File;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.RobotMap;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIOReplay;
import frc.robot.subsystems.climber.ClimberIOSim;
import frc.robot.subsystems.climber.ClimberIOSparkMax;
import frc.robot.subsystems.drive4.SwerveSubsystem;
import frc.robot.subsystems.feeder.Feeder;
import frc.robot.subsystems.feeder.FeederIOReplay;
import frc.robot.subsystems.feeder.FeederIOSim;
import frc.robot.subsystems.feeder.FeederIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOReplay;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.shooter.BeambreakIOReal;
import frc.robot.subsystems.shooter.BeambreakIOReplay;
import frc.robot.subsystems.shooter.BeambreakIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOReplay;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final SwerveSubsystem m_drive = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
      "swerve/swerve"));;
  private final Climber m_climber;
  private final Intake m_intake;
  private final Feeder m_feeder;
  private final Shooter m_shooter;
  private final Visualizer m_visualizer;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);

  // final CommandXboxController driverXbox = new CommandXboxController(0);
  // private final SwerveSubsystem drivebase

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        // m_drive =
        // new Drive(
        // new GyroIOPigeon2Phoenix6(),
        // new ModuleIOReal(0),
        // new ModuleIOReal(1),
        // new ModuleIOReal(2),
        // new ModuleIOReal(3)
        // );
        m_climber = new Climber(new ClimberIOSparkMax());
        m_intake = new Intake(new IntakeIOSparkMax());
        m_feeder = new Feeder(new FeederIOSparkMax());
        m_shooter = new Shooter(
            new ShooterIOSparkMax(),
            new BeambreakIOReal(RobotMap.Shooter.feederBeambreak),
            new BeambreakIOReal(RobotMap.Shooter.shooterBeambreak));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        // m_drive =
        // new Drive(
        // new GyroIOReplay() {},
        // new ModuleIOSim("FrontLeft"),
        // new ModuleIOSim("FrontRight"),
        // new ModuleIOSim("BackLeft"),
        // new ModuleIOSim("BackRight"));
        m_climber = new Climber(new ClimberIOSim());
        m_intake = new Intake(new IntakeIOSim());
        m_feeder = new Feeder(new FeederIOSim());
        m_shooter = new Shooter(
            new ShooterIOSparkMax(),
            new BeambreakIOSim(RobotMap.Shooter.feederBeambreak),
            new BeambreakIOSim(RobotMap.Shooter.shooterBeambreak));
        break;

      default:
        // Replayed robot, disable IO implementations
        // m_drive =
        // new Drive(
        // new GyroIOReplay() {},
        // new ModuleIOReplay() {},
        // new ModuleIOReplay() {},
        // new ModuleIOReplay() {},
        // new ModuleIOReplay() {});
        m_climber = new Climber(new ClimberIOReplay());
        m_intake = new Intake(new IntakeIOReplay());
        m_feeder = new Feeder(new FeederIOReplay());
        m_shooter = new Shooter(
            new ShooterIOReplay() {
            },
            new BeambreakIOReplay() {
            },
            new BeambreakIOReplay() {
            });
        break;

    }
    m_visualizer = new Visualizer(m_climber, m_intake, m_shooter);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // m_drive.setDefaultCommand(
    // m_drive.runVoltageTeleopFieldRelative(
    // () ->
    // new ChassisSpeeds(
    // -teleopAxisAdjustment(controller.getLeftY()) *
    // DriveConstants.maxLinearVelocity,
    // -teleopAxisAdjustment(controller.getLeftX()) *
    // DriveConstants.maxLinearVelocity,
    // -teleopAxisAdjustment(controller.getRightX())
    // * DriveConstants.maxLinearVelocity)));
    // controller.x().onTrue(Commands.runOnce(m_drive::stopWithX, m_drive));
    // controller.y().onTrue(Commands.runOnce(m_drive::resetOffsetsCmd, m_drive));
    // controller
    // .b()
    // .onTrue(
    // Commands.runOnce(
    // () ->
    // m_drive.setPose(
    // new Pose2d(m_drive.getPose().getTranslation(), new Rotation2d())),
    // m_drive)
    // .ignoringDisable(true));

    // controller.start().onTrue(m_climber.runCurrentHoming());
    // controller
    // .leftBumper()
    // .onTrue(
    // m_climber.setExtensionCmd(() -> ClimberConstants.maxHeight));
    // controller
    // .leftTrigger(0.1)
    // .onTrue(
    // m_climber.setExtensionCmd(() -> ClimberConstants.minHeight));
    // }

    m_drive.setDefaultCommand(
        m_drive.driveCommand(
            () -> MathUtil.applyDeadband(controller.getLeftY(), 0.01),
            () -> MathUtil.applyDeadband(controller.getLeftX(), 0.01),
            () -> controller.getRightX() * 0.5));

    controller.a().whileTrue(
        m_intake.setIntakeDown(false)
    ).onFalse(m_intake.setIntakeUp());

    controller.b().whileTrue(
        m_intake.setIntakeDown(true)
    ).onFalse(m_intake.setIntakeUp());

    controller.y().whileTrue(
        m_intake.setRollerRPM(() -> 2000)).onFalse(
            m_intake.setRollerRPM(() -> 0));
  }

  public void robotPeriodic() {
    m_visualizer.periodic();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null;
  }
}
