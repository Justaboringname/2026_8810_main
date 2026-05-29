package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AimAndShoot;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;

/**
 * 顶层协调器(非 SubsystemBase 的工厂/持有者)。
 *
 * <h2>为什么不是 SubsystemBase</h2>
 *
 * 现有代码所有执行路径(旧 Aimbot/AutonShoot/AutonTrench、povLeft 手动飞轮、povDown 手动喂球)都是 {@code Command} 且 {@code
 * addRequirements(...)}。WPILib 调度器只在<b>命令之间</b>按 requirement 仲裁, <b>不会</b>仲裁某个 subsystem 自己的 {@code
 * periodic()}。所以若本类做成 SubsystemBase 并在 periodic 里 写电机,就会和那些手动命令每 20ms 对打(抖动)。因此本类不写电机,只:
 *
 * <ul>
 *   <li>持有 {@link AimingParameters}(距离 + dashboard 可调旋钮);
 *   <li>生产带 requirement 的 {@link AimAndShoot} 命令,交给调度器与手动绑定互锁。
 * </ul>
 *
 * 安全逻辑仍由纯函数 {@link ShotGate} + {@link frc.robot.subsystems.ShooterSubsystem.ShotCalculator} 承担(都有
 * JUnit)。命令只是“谁来调用它们”。
 *
 * <h2>本切片边界</h2>
 *
 * AimAndShoot <b>不转底盘</b>(驾驶员手动瞄准,与旧 Aimbot 不同);<b>不</b>摆 intake(swing 用不到,已去除)。 底盘对准 / 收编 auto
 * 是后续切片。无球检测传感器,feeder 仅靠 shooter/hood 就绪门控。
 */
public class Superstructure {
  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;
  private final AimingParameters aiming = new AimingParameters();

  public Superstructure(ShooterSubsystem shooter, FeederSubsystem feeder) {
    this.shooter = shooter;
    this.feeder = feeder;
  }

  /** 距离 + 可调旋钮的瞄准参数源(供命令与日志读取)。 */
  public AimingParameters aiming() {
    return aiming;
  }

  /**
   * 生产“瞄准距离 → spinup + hood + 就绪门控喂球”的命令。requires shooter+feeder,自动与手动绑定互锁。 用 {@code whileTrue}
   * 绑定:松开即结束,{@link AimAndShoot#end} 清零所有输出。
   */
  public Command aimAndShoot() {
    return new AimAndShoot(shooter, feeder, aiming);
  }
}
