package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.ShooterSubsystem.ShotCalculator;
import frc.robot.subsystems.ShooterSubsystem.ShotCalculator.ShooterSetpoint;
import org.junit.jupiter.api.Test;

/** ShotGate 纯逻辑测试 —— 验证最重要的安全属性:任一前置条件不满足都不喂球。 每个 case 只翻一个条件为 false,确认门控关死(单变量隔离,不靠"编译通过")。 */
class ShotGateTest {

  private static ShooterSetpoint validShot() {
    // 3.0m,默认旋钮 → isValid=true
    return ShotCalculator.calculate(3.0);
  }

  private static ShooterSetpoint invalidShot() {
    // 越界距离 → isValid=false
    return ShotCalculator.calculate(9.0);
  }

  @Test
  void allConditionsMetAllowsFeed() {
    assertTrue(ShotGate.shouldFeed(true, true, true, true, true, validShot()), "全满足应允许喂球");
  }

  @Test
  void noShootRequestBlocks() {
    assertFalse(ShotGate.shouldFeed(false, true, true, true, true, validShot()), "未请求射球不应喂");
  }

  @Test
  void disabledBlocks() {
    assertFalse(ShotGate.shouldFeed(true, false, true, true, true, validShot()), "disabled 不应喂");
  }

  @Test
  void noAllianceBlocks() {
    // advisor 指出的坑:联盟未分配时距离按 Blue 默认算,可能瞄错目标 → 不开火
    assertFalse(ShotGate.shouldFeed(true, true, false, true, true, validShot()), "联盟未分配不应喂");
  }

  @Test
  void shooterNotAtSpeedBlocks() {
    assertFalse(ShotGate.shouldFeed(true, true, true, false, true, validShot()), "飞轮未到速不应喂");
  }

  @Test
  void hoodNotAtAngleBlocks() {
    assertFalse(ShotGate.shouldFeed(true, true, true, true, false, validShot()), "hood 未到角不应喂");
  }

  @Test
  void invalidSetpointBlocks() {
    // isValid 必须真正 AND 进门控(否则只是装饰)
    assertFalse(
        ShotGate.shouldFeed(true, true, true, true, true, invalidShot()), "不可信 setpoint 不应喂");
  }

  @Test
  void nullSetpointBlocks() {
    assertFalse(ShotGate.shouldFeed(true, true, true, true, true, null), "null setpoint 不应喂");
  }

  @Test
  void isNearWithinToleranceTrue() {
    assertTrue(ShotGate.isNear(50.4, 50.0, 0.5), "容差内应 true");
    assertTrue(ShotGate.isNear(50.0, 50.0, 0.0), "完全相等零容差应 true");
  }

  @Test
  void isNearOutsideToleranceFalse() {
    assertFalse(ShotGate.isNear(50.6, 50.0, 0.5), "容差外应 false");
  }

  // ---- auto 7 参重载:多一个"转向到位"前置门 ----

  @Test
  void autoAllConditionsMetAllowsFeed() {
    assertTrue(
        ShotGate.shouldFeed(true, true, true, true, true, true, validShot()), "auto 全满足应允许喂球");
  }

  @Test
  void autoNotAimedBlocks() {
    // auto 自动转底盘:没对准 HUB 时绝不喂(否则朝错方向喷一坨)
    assertFalse(ShotGate.shouldFeed(true, true, true, false, true, true, validShot()), "底盘未对准不应喂");
  }

  @Test
  void autoOtherGatesStillApply() {
    // 7 参版同样受其余门约束:未到速 / 越界 setpoint 仍不喂(即使已对准)
    assertFalse(
        ShotGate.shouldFeed(true, true, true, true, false, true, validShot()), "已对准但未到速不应喂");
    assertFalse(
        ShotGate.shouldFeed(true, true, true, true, true, true, invalidShot()),
        "已对准但 setpoint 不可信不应喂");
  }

  @Test
  void sixArgOverloadDoesNotGateOnAim() {
    // teleop 6 参版等价于 7 参版 aimedAtTarget=true(驾驶员手动瞄,不门控朝向)
    assertEquals(
        ShotGate.shouldFeed(true, true, true, true, true, validShot()),
        ShotGate.shouldFeed(true, true, true, true, true, true, validShot()),
        "6 参版应等价于 7 参版 aimed=true");
  }
}
