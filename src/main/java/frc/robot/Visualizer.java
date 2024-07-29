package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Visualizer extends SubsystemBase {
    private Mechanism2d m_main;

    private MechanismLigament2d m_climberMech;
    private MechanismLigament2d m_intakeMech;
    private MechanismLigament2d m_shooterMech;

    private MechanismLigament2d m_climberTarget;
    private MechanismLigament2d m_intakeTarget;
    private MechanismLigament2d m_shooterTarget;

    private MechanismRoot2d m_climberRoot;
    private MechanismRoot2d m_intakeRoot;
    private MechanismRoot2d m_shooterRoot;

    private Climber m_climber;
    private Intake m_intake;
    private Shooter m_shooter;

    public Visualizer(Climber climber, Intake intake, Shooter shooter) {
        m_climber = climber;
        m_intake = intake;
        m_shooter = shooter;

        m_main = new Mechanism2d(Units.inchesToMeters(48.0), Units.inchesToMeters(48.0));
        m_climberRoot = m_main.getRoot("climber_base", Units.inchesToMeters(24+3.0), Units.inchesToMeters(3.75));
        m_intakeRoot = m_main.getRoot("intake_pivot", Units.inchesToMeters(24-8.835736), Units.inchesToMeters(10.776049));
        m_shooterRoot = m_main.getRoot("shooter_pivot", Units.inchesToMeters(24-3.485625), Units.inchesToMeters(11.5));
        m_main.getRoot("Robot", Units.inchesToMeters(24-(26.5/2)), Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 15, new Color8Bit(Color.kYellow))
        );

        m_climberMech = m_climberRoot.append(new MechanismLigament2d("Climber", Units.inchesToMeters(17.25), 90, 8, new Color8Bit(Color.kFirstRed)));
        m_climberTarget = m_climberRoot.append(new MechanismLigament2d("Climber Target", Units.inchesToMeters(17.25), 90, 2, new Color8Bit(Color.kRed)));

        m_intakeMech = m_intakeRoot.append(new MechanismLigament2d("Intake", Units.inchesToMeters(14.914264), 83.649627, 8, new Color8Bit(Color.kFirstBlue)));
        m_intakeTarget = m_intakeRoot.append(new MechanismLigament2d("Intake Target", Units.inchesToMeters(14.914264), 83.649627, 2, new Color8Bit(Color.kBlue)));

        m_shooterMech = m_shooterRoot.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(13.1001837), 11.313120, 8, new Color8Bit(Color.kWhite)));
        m_shooterTarget = m_shooterRoot.append(new MechanismLigament2d("Shooter Target", Units.inchesToMeters(13.1001837), 11.313120, 2, new Color8Bit(Color.kBeige)));

        SmartDashboard.putData("Climb Up", (Sendable) m_climber.setExtensionCmd(() -> ClimberConstants.maxHeight));
        SmartDashboard.putData("Climb Down", (Sendable) m_climber.setExtensionCmd(() -> 0));

        SmartDashboard.putData("Intake Up", (Sendable) m_intake.setPivotTarget(() -> IntakeConstants.up));
        SmartDashboard.putData("Intake Down", (Sendable) m_intake.setPivotTarget(() -> IntakeConstants.down));

        SmartDashboard.putData("Shooter 0", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.down));
        SmartDashboard.putData("Shooter 15", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.down + Units.degreesToRadians(15)));
        SmartDashboard.putData("Shooter 30", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.down + Units.degreesToRadians(30)));
        SmartDashboard.putData("Shooter 45", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.down + Units.degreesToRadians(45)));
        SmartDashboard.putData("Shooter 60", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.down + Units.degreesToRadians(60)));
        SmartDashboard.putData("Shooter 75", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.down + Units.degreesToRadians(75)));
        SmartDashboard.putData("Shooter 90", (Sendable) m_shooter.setPivotTarget(() -> ShooterConstants.up));
        SmartDashboard.putData("Test", (Sendable) m_shooter.setPivotVoltage(() -> 3));
        SmartDashboard.putData("NegTest", (Sendable) m_shooter.setPivotVoltage(() -> -3));

    }
    
    public void periodic() {
        m_climberMech.setLength((m_climber.getExtensionMeters()) + Units.inchesToMeters(13.5));
        m_climberTarget.setLength((m_climber.getTargetMeters()) + Units.inchesToMeters(13.5));

        m_intakeMech.setAngle(Units.radiansToDegrees(m_intake.getAngleRadians()));
        m_intakeTarget.setAngle(Units.radiansToDegrees(m_intake.getTargetRadians()));

        m_shooterMech.setAngle(Units.radiansToDegrees(m_shooter.getAngleRadians()));
        m_shooterTarget.setAngle(Units.radiansToDegrees(m_shooter.getTargetRadians()));

        Logger.recordOutput("Visualizer/FullRobot", m_main);
    }
}
