package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.climber.Climb;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot2.Pivot;
import org.littletonrobotics.junction.AutoLogOutput;

public class Visualizer extends SubsystemBase {
  @AutoLogOutput private Mechanism2d m_main;

  private MechanismLigament2d m_climberMech;
  private MechanismLigament2d m_intakeMech;
  private MechanismLigament2d m_pivotMech;

  private MechanismLigament2d m_climberTarget;
  private MechanismLigament2d m_intakeTarget;
  private MechanismLigament2d m_pivotTarget;
  private MechanismLigament2d m_pivotRelative;

  private MechanismRoot2d m_climberRoot;
  private MechanismRoot2d m_intakeRoot;
  private MechanismRoot2d m_pivotRoot;

  private Climb m_climber;
  private Intake m_intake;
  private Pivot m_pivot;

  public Visualizer(Climb climber, Intake intake, Pivot pivot) {
    m_climber = climber;
    m_intake = intake;
    m_pivot = pivot;

    m_main = new Mechanism2d(Units.inchesToMeters(48.0), Units.inchesToMeters(48.0));
    m_climberRoot =
        m_main.getRoot("climber_base", Units.inchesToMeters(24 + 3.0), Units.inchesToMeters(3.75));
    m_intakeRoot =
        m_main.getRoot(
            "intake_pivot", Units.inchesToMeters(24 - 8.835736), Units.inchesToMeters(10.776049));
    m_pivotRoot =
        m_main.getRoot(
            "pivot_pivot", Units.inchesToMeters(24 - 3.485625), Units.inchesToMeters(11.5));
    m_main
        .getRoot("Robot", Units.inchesToMeters(24 - (26.5 / 2)), Units.inchesToMeters(7))
        .append(
            new MechanismLigament2d(
                "frame", Units.inchesToMeters(26), 0, 2, new Color8Bit(Color.kYellow)));

    m_climberMech =
        m_climberRoot.append(
            new MechanismLigament2d(
                "Climb", Units.inchesToMeters(17.25), 90, 4, new Color8Bit(Color.kFirstRed)));
    m_climberTarget =
        m_climberRoot.append(
            new MechanismLigament2d(
                "Climb Target", Units.inchesToMeters(17.25), 90, 2, new Color8Bit(Color.kRed)));

    m_intakeMech =
        m_intakeRoot.append(
            new MechanismLigament2d(
                "Intake",
                Units.inchesToMeters(14.914264),
                83.649627,
                4,
                new Color8Bit(Color.kFirstBlue)));
    m_intakeTarget =
        m_intakeRoot.append(
            new MechanismLigament2d(
                "Intake Target",
                Units.inchesToMeters(14.914264),
                83.649627,
                2,
                new Color8Bit(Color.kBlue)));

    m_pivotMech =
        m_pivotRoot.append(
            new MechanismLigament2d(
                "Shooter", Units.inchesToMeters(13.1001837), 12, 4, new Color8Bit(Color.kWhite)));
    m_pivotRelative =
        m_pivotRoot.append(
            new MechanismLigament2d(
                "Shooter Relative",
                Units.inchesToMeters(13.1001837),
                12,
                4,
                new Color8Bit(Color.kGray)));
    m_pivotTarget =
        m_pivotRoot.append(
            new MechanismLigament2d(
                "Shooter Target",
                Units.inchesToMeters(13.1001837),
                12,
                2,
                new Color8Bit(Color.kBeige)));

    SmartDashboard.putData(
        "Intake Up", (Sendable) m_intake.setPivotTarget(() -> IntakeConstants.up));
    SmartDashboard.putData(
        "Intake Down", (Sendable) m_intake.setPivotTarget(() -> IntakeConstants.down));

    SmartDashboard.putData(
        "Shooter 0", (Sendable) m_pivot.setPivotTarget(() -> ShooterConstants.down));
    SmartDashboard.putData(
        "Shooter 56", (Sendable) m_pivot.setPivotTarget(() -> ShooterConstants.up));

    SmartDashboard.putData("Pos", (Sendable) m_pivot.setPivotVoltage(() -> 5));
    SmartDashboard.putData("NegPos", (Sendable) m_pivot.setPivotVoltage(() -> -5));
  }

  public void periodic() {
    m_climberMech.setLength((m_climber.getExtensionMeters()) + Units.inchesToMeters(13.5));
    m_climberTarget.setLength((m_climber.getTargetMeters()) + Units.inchesToMeters(13.5));

    m_intakeMech.setAngle(Units.radiansToDegrees(m_intake.getAngleRadians()));
    m_intakeTarget.setAngle(Units.radiansToDegrees(m_intake.getTargetRadians()));

    m_pivotMech.setAngle(Units.radiansToDegrees(m_pivot.getAngleRadians()));
    m_pivotRelative.setAngle(Units.radiansToDegrees(m_pivot.getRelativeRadians()));
    m_pivotTarget.setAngle(Units.radiansToDegrees(m_pivot.getTargetRadians()));

    // Logger.recordOutput("Visualizer/FullRobot", m_main);
  }
}
