package frc.robot.subsystems.superstructure;

import frc.robot.subsystems.ShooterSubsystem.ShotCalculator.ShooterSetpoint;

/**
 * 纯函数喂球门控(fail-safe 的核心安全属性,做成无 HAL 纯逻辑以便 JUnit 直接测)。
 *
 * <p>这是整个 superstructure 里最重要的一条安全规则:<b>只有当飞轮到速、hood 到角、且 setpoint 本身可信
 * (距离在标定范围内、未超物理上限)、且当前确实想射、且机器人 enabled,才允许喂球。</b>任一不满足都不喂—— 宁可不射,也不要半速 / 朝错目标乱抛(4-ball dumper
 * 散布大,乱射会丢球甚至犯规)。
 *
 * <p>advisor 提醒的两个坑都在这里挡住:
 *
 * <ul>
 *   <li>{@code shot.isValid()} 必须真正 AND 进门控,而不是只挂在 record 上当摆设;
 *   <li>{@code alliancePresent} 为 false 时(DS 还没给联盟),distanceToGoal 会默认按 Blue 算 → 可能瞄错目标, 此时一律不开火。
 * </ul>
 */
public final class ShotGate {
  private ShotGate() {}

  /**
   * 是否允许此刻喂球。
   *
   * @param wantShoot 操作手/auto 此刻是否请求射球
   * @param enabled 机器人是否 enabled(disabled 一律 false)
   * @param alliancePresent DriverStation 是否已分配联盟(未分配则距离/目标不可信)
   * @param shooterAtSpeed 飞轮是否到目标转速
   * @param hoodAtAngle hood 是否到目标角
   * @param shot 当前距离算出的 setpoint(含 isValid)
   */
  public static boolean shouldFeed(
      boolean wantShoot,
      boolean enabled,
      boolean alliancePresent,
      boolean shooterAtSpeed,
      boolean hoodAtAngle,
      ShooterSetpoint shot) {
    return wantShoot
        && enabled
        && alliancePresent
        && shot != null
        && shot.isValid()
        && shooterAtSpeed
        && hoodAtAngle;
  }

  /**
   * 数值就绪判定:|measured - target| 在容差内。提取成纯函数便于在子系统 atGoal() 与测试间复用。
   *
   * @param measured 实测值
   * @param target 目标值
   * @param tolerance 容差(同单位,非负)
   */
  public static boolean isNear(double measured, double target, double tolerance) {
    return Math.abs(measured - target) <= tolerance;
  }
}
