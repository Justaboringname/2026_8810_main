package frc.robot.subsystems.ShooterSubsystem;

import org.littletonrobotics.junction.AutoLog;

/**
 * Shooter 硬件抽象层(AdvantageKit IO 模式)。
 *
 * <p>硬件:飞轮 = 1 个 leader TalonFX(motor1)+ 2 个硬件 Follower(motor2 Aligned / motor3 Opposed), 单
 * VelocityTorqueCurrentFOC 闭环;hood = 1 个 TalonFX(MotionMagic 位置闭环 + 上电电流归位)。
 *
 * <p>setter 单位保持子系统约定(rps / volts / mechanism-rotations),不泄漏 TalonFX 类型。inputs 仿照 drive 层 {@code
 * ModuleIOInputs}:每个电机带 {@code *Connected} 布尔 + 速度/电压/电流/温度, 这样某个 Follower 掉线/堵转能在 AdvantageScope
 * 里单独看到(旧版把三电机平均会掩盖死 Follower)。
 */
public interface ShooterIO {
  public default void ShooterSetRps(double rps) {}

  public default void ShooterSetV(double voltage) {}

  public default void HoodSetAngle(double angle) {}

  public default void HoodSetZero() {}

  public default void HoodSetV(double voltage) {}

  @AutoLog
  public class ShooterIOInputs {
    // ---- 飞轮:聚合量(向后兼容,ShooterSubsystem 仍读这两个) ----
    /** 三个飞轮电机的平均速度(rps)。用于 isAtSetSpeed 门控。 */
    public double ShooterRPS = 0;

    /** 三个飞轮电机的平均 supply 电流(A)。 */
    public double ShooterCurrentAMPS = 0;

    // ---- 飞轮:每电机诊断(死 Follower 检测) ----
    public boolean Shooter1Connected = false;
    public boolean Shooter2Connected = false;
    public boolean Shooter3Connected = false;
    public double Shooter1VelocityRPS = 0;
    public double Shooter2VelocityRPS = 0;
    public double Shooter3VelocityRPS = 0;
    public double Shooter1AppliedVolts = 0;
    public double Shooter2AppliedVolts = 0;
    public double Shooter3AppliedVolts = 0;
    public double Shooter1StatorAmps = 0;
    public double Shooter2StatorAmps = 0;
    public double Shooter3StatorAmps = 0;
    public double Shooter1TempC = 0;
    public double Shooter2TempC = 0;
    public double Shooter3TempC = 0;

    // ---- hood ----
    public boolean HoodConnected = false;

    /** hood 机构位置(rotations,= getPosition,已含 SensorToMechanismRatio)。 */
    public double HoodAngle = 0;

    public double HoodVelocityRPS = 0;
    public double HoodVoltageV = 0;

    /** hood supply 电流(A)。归位逻辑用它判堵转。 */
    public double HoodCurrentAMPS = 0;

    public double HoodStatorAmps = 0;
    public double HoodTempC = 0;
  }

  public default void updateInputs(ShooterIOInputs inputs) {}
}
