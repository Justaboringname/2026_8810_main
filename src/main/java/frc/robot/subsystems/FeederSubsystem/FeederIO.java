package frc.robot.subsystems.FeederSubsystem;

import org.littletonrobotics.junction.AutoLog;

/**
 * Feeder 硬件抽象层(AdvantageKit IO 模式)。硬件:indexer 滚轮 + belt 滚轮,各一个 TalonFX。
 *
 * <p>inputs 仿照 drive 层 {@code ModuleIOInputs} / shooter:每个滚轮带 {@code *Connected} 布尔 + 速度/电压/supply
 * 电流/stator 电流/温度,便于在 AdvantageScope 单独诊断卡死或掉线。setter 单位保持子系统约定。
 */
public interface FeederIO {
  public default void BeltSetRps(double RPS) {}

  public default void IndexerSetRps(double RPS) {}

  public default void BeltSetV(double voltage) {}

  public default void IndexerSetV(double voltage) {}

  @AutoLog
  public class FeederIOInputs {
    public boolean BeltConnected = false;
    public boolean IndexerConnected = false;

    public double BeltVelocityRPS = 0;
    public double IndexerVelocityRPS = 0;

    /** supply 电流(向后兼容旧字段名)。 */
    public double BeltCurrentAMPS = 0;

    public double IndexerCurrentAMPS = 0;

    public double BeltStatorAMPS = 0;
    public double IndexerStatorAMPS = 0;
    public double BeltVoltageV = 0;
    public double IndexerVoltageV = 0;
    public double BeltTempC = 0;
    public double IndexerTempC = 0;
  }

  public default void updateInputs(FeederIOInputs inputs) {}
}
