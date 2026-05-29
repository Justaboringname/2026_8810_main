package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShotCalculator.ShooterSetpoint;
import frc.robot.subsystems.superstructure.AimingParameters;
import frc.robot.subsystems.superstructure.ShotGate;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * 距离驱动的自动射球命令(旧 Aimbot 的忠实下沉版:去掉底盘转向 + 去掉 intake swing,加入 isValid/联盟 fail-safe)。
 *
 * <p>取代旧 Aimbot 的 shooter+feeder 部分。做成 {@code Command} 且 {@code addRequirements(shooter, feeder)},
 * 因此 WPILib 调度器会自动与 povLeft(手动飞轮)、povDown(手动喂球)等手动绑定<b>互锁</b>——不会出现 periodic 写电机与命令对打的抖动(这正是放弃
 * SubsystemBase.periodic 写法的原因)。
 *
 * <p>飞轮/hood 设定值与喂球门控全部复用已测纯逻辑: {@link
 * frc.robot.subsystems.ShooterSubsystem.ShotCalculator}(距离→setpoint + isValid)+ {@link
 * ShotGate}(喂球当且仅当 想射 && enabled && 联盟已分配 && shot.isValid() && 飞轮到速 && hood到角)。
 *
 * <p><b>本切片不转底盘</b>:驾驶员手动瞄准 HUB(与旧 Aimbot 自动对准不同)。底盘对准是后续切片。 用 {@code whileTrue} 绑定:按住 = 持续瞄准射击,松开
 * = {@link #end} 清零飞轮与喂球。
 */
public class AimAndShoot extends Command {
  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;
  private final AimingParameters aiming;

  // 喂球电压(dashboard 可调,沿用旧默认 12V)
  private final LoggedTunableNumber feedIndexerVolts =
      new LoggedTunableNumber("AimAndShoot/FeedIndexerVolts", 12.0);
  private final LoggedTunableNumber feedBeltVolts =
      new LoggedTunableNumber("AimAndShoot/FeedBeltVolts", 12.0);

  /** hood 到角容差(度)。 */
  private static final double HOOD_TOLERANCE_DEG = 1.5;

  /** 构造注入子系统(避免 getInstance 顺序陷阱)。 */
  public AimAndShoot(ShooterSubsystem shooter, FeederSubsystem feeder, AimingParameters aiming) {
    this.shooter = shooter;
    this.feeder = feeder;
    this.aiming = aiming;
    addRequirements(shooter, feeder);
  }

  /** hood 是否到目标角。getHoodAngle 是机构 rotations,×360 转度与 shot 的度比较。 */
  private boolean hoodAtAngle(double targetDeg) {
    double measuredDeg = shooter.getHoodAngle() * 360.0;
    return ShotGate.isNear(measuredDeg, targetDeg, HOOD_TOLERANCE_DEG);
  }

  @Override
  public void execute() {
    boolean enabled = !DriverStation.isDisabled();
    boolean alliancePresent = aiming.alliancePresent();
    ShooterSetpoint shot = aiming.currentShot();

    // 始终命令飞轮 + hood 到设定值(预热),即使门控未通过也先转起来
    shooter.setShooterRps(shot.flywheelRps());
    shooter.setHoodAngle(shot.hoodAngleDeg());

    boolean shooterAtSpeed = shooter.isAtSetSpeed(shot.flywheelRps());
    boolean hoodAtAngle = hoodAtAngle(shot.hoodAngleDeg());
    boolean feeding =
        ShotGate.shouldFeed(true, enabled, alliancePresent, shooterAtSpeed, hoodAtAngle, shot);

    if (feeding) {
      feeder.setIndexerVoltage(feedIndexerVolts.get());
      feeder.setBeltVoltage(feedBeltVolts.get());
    } else {
      feeder.setIndexerVoltage(0.0);
      feeder.setBeltVoltage(0.0);
    }

    // 日志(AdvantageScope 可见,便于上车诊断为什么没喂球)
    Logger.recordOutput("AimAndShoot/DistanceMeters", aiming.distanceMeters());
    Logger.recordOutput("AimAndShoot/TargetFlywheelRps", shot.flywheelRps());
    Logger.recordOutput("AimAndShoot/TargetHoodDeg", shot.hoodAngleDeg());
    Logger.recordOutput("AimAndShoot/ShotValid", shot.isValid());
    Logger.recordOutput("AimAndShoot/AlliancePresent", alliancePresent);
    Logger.recordOutput("AimAndShoot/ShooterAtSpeed", shooterAtSpeed);
    Logger.recordOutput("AimAndShoot/HoodAtAngle", hoodAtAngle);
    Logger.recordOutput("AimAndShoot/Feeding", feeding);
    Logger.recordOutput("AimAndShoot/MeasuredHoodDeg", shooter.getHoodAngle() * 360.0);
  }

  @Override
  public void end(boolean interrupted) {
    // 清零:disable / 松开 / 被其他命令打断时都不留飞轮转着(旧 Aimbot.end 同语义)
    shooter.setShooterVoltage(0.0);
    feeder.setIndexerVoltage(0.0);
    feeder.setBeltVoltage(0.0);
  }

  @Override
  public boolean isFinished() {
    return false; // whileTrue 绑定,松手即被调度器中断 → end() 清零
  }
}
