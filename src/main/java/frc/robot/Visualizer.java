package frc.robot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class Visualizer {
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
        m_climberRoot = m_main.getRoot("climber_base", Units.inchesToMeters(3.0), Units.inchesToMeters(3.75));
        m_intakeRoot = m_main.getRoot("intake_pivot", Units.inchesToMeters(-8.835736), Units.inchesToMeters(10.776049));
        m_shooterRoot = m_main.getRoot("shooter_pivot", Units.inchesToMeters(-3.485625), Units.inchesToMeters(-11.5));
        m_main.getRoot("Robot", 0, Units.inchesToMeters(7)).append(
            new MechanismLigament2d("frame", Units.inchesToMeters(26+7.5), 0, 30, new Color8Bit(Color.kBlue))
        );

        m_climberMech = m_climberRoot.append(new MechanismLigament2d("Climber", Units.inchesToMeters(17.25), 0, 10, new Color8Bit(Color.kFirstRed)));
        m_climberTarget = m_climberRoot.append(new MechanismLigament2d("Climber Target", Units.inchesToMeters(17.25), 0, 2, new Color8Bit(Color.kRed)));

        m_intakeMech = m_intakeRoot.append(new MechanismLigament2d("Intake", Units.inchesToMeters(14.914264), 83.649627, 10, new Color8Bit(Color.kFirstBlue)));
        m_intakeTarget = m_intakeRoot.append(new MechanismLigament2d("Intake Target", Units.inchesToMeters(14.914264), 83.649627, 2, new Color8Bit(Color.kBlue)));

        m_shooterMech = m_shooterRoot.append(new MechanismLigament2d("Shooter", Units.inchesToMeters(13.1001837), 11.313120, 10, new Color8Bit(Color.kWhite)));
        m_intakeTarget = m_intakeRoot.append(new MechanismLigament2d("Shooter Target", Units.inchesToMeters(13.1001837), 11.313120, 2, new Color8Bit(Color.kBeige)));
    }
    
    public void periodic() {
        m_climberMech.setLength((m_climber.getExtensionMeters()) + Units.inchesToMeters(13.5));
        m_climberTarget.setLength((m_climber.getTargetMeters()) + Units.inchesToMeters(13.5));

        m_intakeMech.setAngle(m_intake.getAngleDegrees());
        m_intakeTarget.setAngle(m_intake.getTargetDegrees());

        // TODO
        // m_shooterMech.setAngle(m_shooter.getAngleDegrees());
        // m_shooterTarget.setAngle(m_shooter.getTargetDegrees());

        Logger.recordOutput("FullRobot", m_main);
    }
}
