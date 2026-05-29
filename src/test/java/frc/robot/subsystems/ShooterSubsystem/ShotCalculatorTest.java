package frc.robot.subsystems.ShooterSubsystem;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/** ShotCalculator 纯逻辑测试(无 HAL,直接 ./gradlew test 跑)。 */
class ShotCalculatorTest {

  private static final double EPS = 1e-6;

  @Test
  void launchAngleDecreasesWithDistance() {
    // sim:距离越远弹道越平,发射角应单调下降
    double near = ShotCalculator.launchAngleDeg(1.5);
    double far = ShotCalculator.launchAngleDeg(5.0);
    assertTrue(near > far, "发射角应随距离下降: near=" + near + " far=" + far);
    // 量级 sanity:落在 sim sweep 的 ~56–73° 区间
    assertTrue(near > 60 && near < 80, "近距发射角越界: " + near);
    assertTrue(far > 50 && far < 65, "远距发射角越界: " + far);
  }

  @Test
  void hoodAngleIncreasesWithDistance() {
    // hood = offset - launch,launch 随距离降 → hood 随距离升(与老手调表方向一致)
    double near = ShotCalculator.calculate(1.5).hoodAngleDeg();
    double far = ShotCalculator.calculate(5.0).hoodAngleDeg();
    assertTrue(far > near, "hood 角应随距离上升: near=" + near + " far=" + far);
  }

  @Test
  void flywheelRpsIncreasesAndIsInSimRange() {
    double near = ShotCalculator.calculate(1.5).flywheelRps();
    double far = ShotCalculator.calculate(5.0).flywheelRps();
    assertTrue(far > near, "飞轮转速应随距离上升");
    // sim 量级 ≈ 24–33 RPS(scale=1.0)
    assertTrue(near > 20 && near < 30, "近距飞轮 RPS 越界: " + near);
    assertTrue(far > 28 && far < 40, "远距飞轮 RPS 越界: " + far);
  }

  @Test
  void distanceIsClampedOutsideSweepRange() {
    // 范围外应夹到端点,不外推
    assertEquals(
        ShotCalculator.calculate(MIN()).hoodAngleDeg(),
        ShotCalculator.calculate(0.5).hoodAngleDeg(),
        EPS,
        "近端外推未被夹住");
    assertEquals(
        ShotCalculator.calculate(MAX()).flywheelRps(),
        ShotCalculator.calculate(9.0).flywheelRps(),
        EPS,
        "远端外推未被夹住");
  }

  @Test
  void inRangeSetpointIsValid() {
    // sweep 范围内、默认旋钮:应是可信 setpoint
    assertTrue(ShotCalculator.calculate(3.0).isValid(), "范围内应 isValid=true");
    assertTrue(ShotCalculator.calculate(MIN()).isValid(), "近端应 isValid=true");
    assertTrue(ShotCalculator.calculate(MAX()).isValid(), "远端应 isValid=true");
  }

  @Test
  void outOfRangeSetpointIsInvalid() {
    // fail-safe:距离越界 → isValid=false(superstructure 据此拒绝喂球)
    assertTrue(!ShotCalculator.calculate(0.5).isValid(), "过近应 isValid=false");
    assertTrue(!ShotCalculator.calculate(9.0).isValid(), "过远应 isValid=false");
  }

  @Test
  void excessiveFlywheelScaleIsInvalid() {
    // 飞轮 scale 大到超物理上限(90 RPS)→ 不可信,且输出被夹住
    ShotCalculator.ShooterSetpoint sp = ShotCalculator.calculate(5.0, 77.0, 5.0);
    assertTrue(!sp.isValid(), "超物理上限应 isValid=false");
    assertTrue(sp.flywheelRps() <= ShotCalculator.MAX_FLYWHEEL_RPS + EPS, "飞轮 RPS 应被夹到上限内");
  }

  @Test
  void flywheelScaleIsLinear() {
    double d = 3.0;
    double base =
        ShotCalculator.calculate(d, ShotCalculator.DEFAULT_HOOD_LAUNCH_ANGLE_AT_ZERO_DEG, 1.0)
            .flywheelRps();
    double doubled =
        ShotCalculator.calculate(d, ShotCalculator.DEFAULT_HOOD_LAUNCH_ANGLE_AT_ZERO_DEG, 2.0)
            .flywheelRps();
    assertEquals(base * 2.0, doubled, EPS, "flywheelRpsScale 应线性缩放");
  }

  @Test
  void hoodOffsetShiftsHoodAngle() {
    double d = 3.0;
    double base = ShotCalculator.calculate(d, 77.0, 1.0).hoodAngleDeg();
    double shifted = ShotCalculator.calculate(d, 82.0, 1.0).hoodAngleDeg();
    // offset +5 → hood +5(未触顶时)
    assertEquals(base + 5.0, shifted, 1e-3, "hood offset 应平移 hood 角");
  }

  @Test
  void hoodIsClampedToMechanicalRange() {
    // 给一个极大的 offset,hood 应被夹到 HOOD_MAX
    double hood = ShotCalculator.calculate(3.0, 500.0, 1.0).hoodAngleDeg();
    assertEquals(ShotCalculator.HOOD_MAX_DEG, hood, EPS, "hood 未夹到上限");
    // 给一个极小 offset,hood 应被夹到 HOOD_MIN
    double hoodLow = ShotCalculator.calculate(3.0, 0.0, 1.0).hoodAngleDeg();
    assertEquals(ShotCalculator.HOOD_MIN_DEG, hoodLow, EPS, "hood 未夹到下限");
  }

  private static double MIN() {
    return ShotCalculator.MIN_DISTANCE_M;
  }

  private static double MAX() {
    return ShotCalculator.MAX_DISTANCE_M;
  }
}
