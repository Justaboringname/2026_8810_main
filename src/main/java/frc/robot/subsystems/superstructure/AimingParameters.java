package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem.ShotCalculator;
import frc.robot.subsystems.ShooterSubsystem.ShotCalculator.ShooterSetpoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;

/**
 * 把实时输入(机器人到 HUB 距离 + 两个 dashboard 可调旋钮)喂给纯函数 {@link ShotCalculator},产出当前 shot setpoint。
 *
 * <p>所有弹道数学都在 {@link ShotCalculator}(纯函数、有 JUnit);本类只负责接活的运行时输入:
 *
 * <ul>
 *   <li>距离:复用 {@link Drive#distanceToGoal}(Drive.periodic 每周期已算好,基于 pose + 联盟目标);
 *   <li>hood offset / 飞轮 scale:做成 {@link LoggedTunableNumber},上车在 AdvantageScope/dashboard 直接拧,
 *       <b>不用重新编译</b>(见 ShotCalculator 类注释的上车标定说明:sim 绝对量级未标定,scale 大概率要 >1)。
 * </ul>
 */
public class AimingParameters {
  private final LoggedTunableNumber hoodLaunchAngleAtZero =
      new LoggedTunableNumber(
          "Aiming/HoodLaunchAngleAtZeroDeg", ShotCalculator.DEFAULT_HOOD_LAUNCH_ANGLE_AT_ZERO_DEG);
  private final LoggedTunableNumber flywheelRpsScale =
      new LoggedTunableNumber("Aiming/FlywheelRpsScale", ShotCalculator.DEFAULT_FLYWHEEL_RPS_SCALE);

  /** 当前距离 + 当前旋钮算出的 shot setpoint(含 isValid)。 */
  public ShooterSetpoint currentShot() {
    return ShotCalculator.calculate(
        Drive.distanceToGoal, hoodLaunchAngleAtZero.get(), flywheelRpsScale.get());
  }

  /** 当前机器人到 HUB 的距离(米),仅供日志。 */
  public double distanceMeters() {
    return Drive.distanceToGoal;
  }

  /** DriverStation 是否已分配联盟。false 时距离按 Blue 默认算,目标可能错 → 不应开火。 */
  public boolean alliancePresent() {
    return DriverStation.getAlliance().isPresent();
  }

  /** 当前联盟是否 Blue(未分配按 Blue 默认,与 Drive.distanceToGoal 的默认一致)。 */
  public boolean isBlue() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
  }

  /**
   * 机器人朝向 HUB 的目标航向(field frame,弧度)。纯函数,便于 JUnit:与旧 Aimbot 的 atan2 逐字一致。
   *
   * @param robotX 机器人 X(米,field frame)
   * @param robotY 机器人 Y(米,field frame)
   * @param blue true=蓝方目标点,false=红方
   */
  public static double targetHeadingRadians(double robotX, double robotY, boolean blue) {
    Translation2d goal =
        blue ? Constants.aimconstants.bluegoalpos : Constants.aimconstants.redgoalpos;
    return Math.atan2(goal.getY() - robotY, goal.getX() - robotX);
  }
}
