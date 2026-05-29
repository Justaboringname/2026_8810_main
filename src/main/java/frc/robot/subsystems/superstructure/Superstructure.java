package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShotCalculator.ShooterSetpoint;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * 顶层协调器(6328 风格的 thin goal-switch)。当前只覆盖 shooter 垂直切片:spinup + hood + feed-gate。
 *
 * <h2>为什么是 additive(并存而非替换)</h2>
 *
 * 旧的 Aimbot/AutonShoot 仍然可用、未删除。本类<b>默认 IDLE 且 hasControl=false 时完全不碰任何电机</b>,
 * 所以在没人按新按键之前,机器人行为与重构前 100% 一致。只有显式 {@code setDesiredGoal(非IDLE)} 后才接管 shooter+feeder。验证通过后,再单独一个
 * commit 把旧命令收编进来——这次不删。
 *
 * <h2>安全不变量(全部由 {@link ShotGate} 纯函数 + JUnit 背书)</h2>
 *
 * <ul>
 *   <li>disabled → 强制 IDLE,清零所有输出;
 *   <li>喂球当且仅当 {@code wantShoot && enabled && alliancePresent && shot.isValid() && atSpeed &&
 *       atAngle};
 *   <li>联盟未分配时距离按 Blue 默认 → 不开火(防瞄错目标)。
 * </ul>
 *
 * <h2>已知边界(v1)</h2>
 *
 * <ul>
 *   <li><b>不</b>控制底盘朝向(操作手手动瞄准,或仍用旧 Aimbot);drive/auto 收编是后续切片。
 *   <li>hood 上电归位(Robot.autonomousInit/teleopInit 的 runHoodHoming)与本类抢 hood:本类仅在 hasControl 后命令
 *       hood,而归位发生在 init(此时 hasControl=false 不会冲突)。<b>归位期间别按射球键。</b>
 *   <li>无球检测传感器(全机器人无 beam-break/CANrange),feeder 仅靠 shooter/hood 就绪门控,不检测球到位。
 * </ul>
 */
public class Superstructure extends SubsystemBase {

  /** 顶层目标。每个值对应一组对各机构的请求。 */
  public enum SuperGoal {
    /** 不接管任何机构(additive:让旧命令完全拥有电机)。 */
    IDLE,
    /** 飞轮转到目标转速 + hood 到角,但不喂球(预热)。 */
    SPINUP,
    /** 在 spinup 基础上,就绪门控全满足时喂球。 */
    SHOOT
  }

  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;
  private final AimingParameters aiming = new AimingParameters();

  private SuperGoal desiredGoal = SuperGoal.IDLE;
  /** 是否已被新路径接管。false = 纯 additive,periodic 不碰电机。 */
  private boolean hasControl = false;

  // 喂球电压(dashboard 可调,沿用 RobotContainer 旧默认 12V)
  private final LoggedTunableNumber feedIndexerVolts =
      new LoggedTunableNumber("Superstructure/FeedIndexerVolts", 12.0);
  private final LoggedTunableNumber feedBeltVolts =
      new LoggedTunableNumber("Superstructure/FeedBeltVolts", 12.0);

  /** 飞轮到速容差(rps),与 ShooterSubsystem.isAtSetSpeed 的硬编码 1.0 对齐。 */
  private static final double SHOOTER_TOLERANCE_RPS = 1.0;

  /** hood 到角容差(度)。 */
  private static final double HOOD_TOLERANCE_DEG = 1.5;

  public Superstructure(ShooterSubsystem shooter, FeederSubsystem feeder) {
    this.shooter = shooter;
    this.feeder = feeder;
  }

  /** 请求顶层目标。任何非 IDLE 目标会让本类接管 shooter+feeder。 */
  public void setDesiredGoal(SuperGoal goal) {
    desiredGoal = goal;
    if (goal != SuperGoal.IDLE) {
      hasControl = true;
    }
  }

  public SuperGoal getDesiredGoal() {
    return desiredGoal;
  }

  /** hood 是否到目标角。getHoodAngle 是机构 rotations,×360 转度与 shot 的度比较。 */
  private boolean hoodAtAngle(double targetDeg) {
    double measuredDeg = shooter.getHoodAngle() * 360.0;
    return ShotGate.isNear(measuredDeg, targetDeg, HOOD_TOLERANCE_DEG);
  }

  private void commandIdleOutputs() {
    shooter.setShooterVoltage(0.0);
    feeder.setIndexerVoltage(0.0);
    feeder.setBeltVoltage(0.0);
  }

  @Override
  public void periodic() {
    boolean enabled = !DriverStation.isDisabled();
    boolean alliancePresent = aiming.alliancePresent();
    ShooterSetpoint shot = aiming.currentShot();

    boolean shooterAtSpeed = shooter.isAtSetSpeed(shot.flywheelRps());
    boolean hoodAtAngle = hoodAtAngle(shot.hoodAngleDeg());
    boolean feeding = false;

    if (!enabled) {
      // disabled:强制 IDLE,若曾接管则清零输出(fail-safe)
      if (hasControl) {
        commandIdleOutputs();
      }
      desiredGoal = SuperGoal.IDLE;
    } else if (!hasControl) {
      // 纯 additive:尚未接管,完全不碰电机,让旧命令拥有机构
    } else {
      switch (desiredGoal) {
        case SPINUP:
        case SHOOT:
          shooter.setShooterRps(shot.flywheelRps());
          shooter.setHoodAngle(shot.hoodAngleDeg());
          feeding =
              ShotGate.shouldFeed(
                  desiredGoal == SuperGoal.SHOOT,
                  enabled,
                  alliancePresent,
                  shooterAtSpeed,
                  hoodAtAngle,
                  shot);
          if (feeding) {
            feeder.setIndexerVoltage(feedIndexerVolts.get());
            feeder.setBeltVoltage(feedBeltVolts.get());
          } else {
            feeder.setIndexerVoltage(0.0);
            feeder.setBeltVoltage(0.0);
          }
          break;
        case IDLE:
        default:
          commandIdleOutputs();
          break;
      }
    }

    // 日志(AdvantageScope 可见,便于上车诊断为什么没喂球)
    Logger.recordOutput("Superstructure/DesiredGoal", desiredGoal.toString());
    Logger.recordOutput("Superstructure/HasControl", hasControl);
    Logger.recordOutput("Superstructure/Enabled", enabled);
    Logger.recordOutput("Superstructure/AlliancePresent", alliancePresent);
    Logger.recordOutput("Superstructure/DistanceMeters", aiming.distanceMeters());
    Logger.recordOutput("Superstructure/TargetFlywheelRps", shot.flywheelRps());
    Logger.recordOutput("Superstructure/TargetHoodDeg", shot.hoodAngleDeg());
    Logger.recordOutput("Superstructure/ShotValid", shot.isValid());
    Logger.recordOutput("Superstructure/ShooterAtSpeed", shooterAtSpeed);
    Logger.recordOutput("Superstructure/HoodAtAngle", hoodAtAngle);
    Logger.recordOutput("Superstructure/Feeding", feeding);
    Logger.recordOutput(
        "Superstructure/MeasuredHoodDeg",
        MathUtil.applyDeadband(shooter.getHoodAngle() * 360.0, 0));
  }
}
