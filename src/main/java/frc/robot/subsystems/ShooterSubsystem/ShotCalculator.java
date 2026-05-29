package frc.robot.subsystems.ShooterSubsystem;

import frc.robot.util.util8810;

/**
 * 把"机器人到 HUB 的距离"换算成 shooter 设定值(飞轮转速 + hood 角度)的纯函数工具。
 *
 * <p>数据来源:队伍自建的 frc-shooter-sim **V3.1** 弹道优化器(w_tof=1.1,vortex-shedding 物理标定)。 sim 对 1.5–5.0 m 做
 * sweep 后拟合出三条关于距离 d(米)的二次多项式(见 sim 仓库 lookup_polynomial.json)。本类把这些多项式搬进机器人代码,作为上车标定前的**物理起点**。
 *
 * <h2>⚠️ 上车必须标定的两个旋钮</h2>
 *
 * <ol>
 *   <li><b>hood 角度约定映射</b>:sim 输出的是相对水平的<b>发射角</b>(56°~73°,距离越远越平); 机器人 hood 编码器读的是相对机械零位(hardstop
 *       归零)的<b>hood 角</b>(老手调表里是 0°~23°, 距离越远越大)。两者关系是 {@code hoodAngle = HOOD_LAUNCH_ANGLE_AT_ZERO
 *       - launchAngle}。 用老表端点 + sim 反算出 offset ≈ 74.5°~79.3°(均值 ~77°,中段比端点略低,说明两套模型形状 不完全一致),所以
 *       {@link #DEFAULT_HOOD_LAUNCH_ANGLE_AT_ZERO_DEG} 只是估计值,上车要校。
 *   <li><b>飞轮量级</b>:sim 飞轮 ≈ 24~33 RPS(1.5~5 m),而老手调表是 50~63 RPS,差近 2×。 sim
 *       的绝对值依赖未标定参数(Cd=0.5、slip=0.18、gear=1.0 都是估计),所以飞轮输出乘了一个 {@link
 *       #DEFAULT_FLYWHEEL_RPS_SCALE}。若上车发现打不到/打太近,优先调这个 scale (老表暗示真实可能在 ~1.7–2.0×
 *       附近,但请以实测为准,别盲信老表——它本身没验证过)。
 * </ol>
 *
 * <p>本类刻意做成<b>无状态纯函数</b>(不读 NetworkTables、不碰硬件),方便 JUnit 直接测。 子系统持有 {@code LoggedTunableNumber} 后,把
 * offset / scale 当参数传进 {@link #calculate(double, double, double)} 即可在 dashboard 实时调。
 */
public final class ShotCalculator {
  private ShotCalculator() {}

  // ---- frc-shooter-sim V3.1 多项式系数 (lookup_polynomial.json) ----
  // 发射角(度,相对水平):launchAngleDeg(d) = a*d^2 + b*d + c
  private static final double LAUNCH_A = 0.8787950872656836;
  private static final double LAUNCH_B = -10.512939495798362;
  private static final double LAUNCH_C = 86.67105265029092;

  // 飞轮转速(RPM):flywheelRpm(d) = a*d^2 + b*d + c   (gear 1:1,所以 = 电机 RPM)
  private static final double FLY_A = 3.2941176470588833;
  private static final double FLY_B = 131.7310924369742;
  private static final double FLY_C = 1221.7033613445387;

  // 出射速度(m/s):仅用于日志/参考,控制走转速闭环
  private static final double VEXIT_A = 0.014441241111829099;
  private static final double VEXIT_B = 0.5738762184873961;
  private static final double VEXIT_C = 5.330965232708467;

  /** sim sweep 的有效距离范围;范围外会被夹到端点(外推不可信)。 */
  public static final double MIN_DISTANCE_M = 1.5;

  public static final double MAX_DISTANCE_M = 5.0;

  /** hood 机械零位(hardstop)对应的发射角估计值,度。⚠️ 上车标定。 */
  public static final double DEFAULT_HOOD_LAUNCH_ANGLE_AT_ZERO_DEG = 77.0;

  /** 飞轮转速缩放,补偿 sim 绝对量级未标定。⚠️ 上车标定。 */
  public static final double DEFAULT_FLYWHEEL_RPS_SCALE = 1.0;

  /** hood 机械角安全范围(度)。HOOD_MAX 为粗估,确认真实机械上限后更新。 */
  public static final double HOOD_MIN_DEG = 0.0;

  public static final double HOOD_MAX_DEG = 30.0;

  /**
   * 飞轮转速安全上限(RPS)。Kraken X60 自由转速 6000 RPM = 100 RPS;留余量取 90 RPS。 算出的 flywheelRps 超过此值视为不可信
   * setpoint(isValid=false),superstructure 据此拒绝喂球。
   */
  public static final double MAX_FLYWHEEL_RPS = 90.0;

  /**
   * shooter 设定值。
   *
   * @param flywheelRps 飞轮目标转速(rotations/sec,= ShooterSubsystem.setShooterRps 的入参)
   * @param hoodAngleDeg hood 目标角度(机械度,0 = hardstop;= ShooterSubsystem.setHoodAngle 的入参)
   * @param isValid 该 setpoint 是否可信。false 表示距离越界被夹、或飞轮转速超物理上限——
   *     <b>调用方(superstructure)必须据此拒绝喂球(fail-safe),而不是半速乱射。</b>
   */
  public record ShooterSetpoint(double flywheelRps, double hoodAngleDeg, boolean isValid) {}

  /** sim 拟合的发射角(度,相对水平)。 */
  public static double launchAngleDeg(double distanceMeters) {
    double d = clampDistance(distanceMeters);
    return LAUNCH_A * d * d + LAUNCH_B * d + LAUNCH_C;
  }

  /** sim 拟合的飞轮转速(RPM)。 */
  public static double flywheelRpm(double distanceMeters) {
    double d = clampDistance(distanceMeters);
    return FLY_A * d * d + FLY_B * d + FLY_C;
  }

  /** sim 拟合的出射速度(m/s),仅供参考/日志。 */
  public static double exitVelocityMps(double distanceMeters) {
    double d = clampDistance(distanceMeters);
    return VEXIT_A * d * d + VEXIT_B * d + VEXIT_C;
  }

  /**
   * 用默认(未标定)旋钮换算。生产中请用带 tunable 参数的重载。
   *
   * @param distanceMeters 机器人到 HUB 的水平距离(米)
   */
  public static ShooterSetpoint calculate(double distanceMeters) {
    return calculate(
        distanceMeters, DEFAULT_HOOD_LAUNCH_ANGLE_AT_ZERO_DEG, DEFAULT_FLYWHEEL_RPS_SCALE);
  }

  /**
   * 把距离换算成 shooter 设定值。
   *
   * @param distanceMeters 机器人到 HUB 的水平距离(米),范围外夹到 [{@value #MIN_DISTANCE_M}, {@value
   *     #MAX_DISTANCE_M}]
   * @param hoodLaunchAngleAtZeroDeg hood 机械零位对应的发射角(度),见类注释的标定说明
   * @param flywheelRpsScale 飞轮转速缩放,见类注释的标定说明
   */
  public static ShooterSetpoint calculate(
      double distanceMeters, double hoodLaunchAngleAtZeroDeg, double flywheelRpsScale) {
    // 距离在 sweep 范围外 → setpoint 由外推得到,不可信
    boolean inRange = distanceMeters >= MIN_DISTANCE_M && distanceMeters <= MAX_DISTANCE_M;

    double launchDeg = launchAngleDeg(distanceMeters);
    double rawHoodDeg = hoodLaunchAngleAtZeroDeg - launchDeg;
    double hoodDeg = util8810.clamp(rawHoodDeg, HOOD_MIN_DEG, HOOD_MAX_DEG);

    double rawFlywheelRps = (flywheelRpm(distanceMeters) / 60.0) * flywheelRpsScale;
    double flywheelRps = util8810.clamp(rawFlywheelRps, 0.0, MAX_FLYWHEEL_RPS);

    // fail-safe:任一被夹(距离越界 / hood 越界 / 飞轮超上限)都标记不可信
    boolean isValid =
        inRange
            && rawHoodDeg >= HOOD_MIN_DEG
            && rawHoodDeg <= HOOD_MAX_DEG
            && rawFlywheelRps <= MAX_FLYWHEEL_RPS;

    return new ShooterSetpoint(flywheelRps, hoodDeg, isValid);
  }

  private static double clampDistance(double distanceMeters) {
    return util8810.clamp(distanceMeters, MIN_DISTANCE_M, MAX_DISTANCE_M);
  }
}
