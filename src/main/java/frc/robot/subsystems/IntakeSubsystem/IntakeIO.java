package frc.robot.subsystems.IntakeSubsystem;

import org.littletonrobotics.junction.AutoLog;

/**
 * Intake 硬件抽象层(AdvantageKit IO 模式)。硬件:intake 滚轮(velocity/voltage)+ pivot 摆臂 (MotionMagic 位置 +
 * voltage 归位)。
 *
 * <p>inputs 仿照 drive/shooter:每电机带 {@code *Connected} + 速度/电压/supply 电流/stator 电流/温度。 setter
 * 命名/单位保持旧约定(子系统其它代码依赖),不在本轮改名以免扩大改动面。
 */
public interface IntakeIO {
  public default void intakesetrps(double RPS) {}

  public default void IntakesetV(double voltage) {}

  public default void Pivotsetangle(double degrees) {}

  public default void pivotsetV(double voltage) {}

  public default void pivotSetZero() {}

  @AutoLog
  public class IntakeIOInputs {
    public boolean intakeConnected = false;
    public boolean pivotConnected = false;

    public double intakeVelocityRPS = 0;
    public double PivotAngledegrees = 0;
    public double PivotVelocityRPS = 0;

    /** supply 电流(向后兼容旧字段名)。 */
    public double intakeCurrentAMPS = 0;

    public double pivotCurrentAMPS = 0;

    public double intakeStatorAMPS = 0;
    public double pivotStatorAMPS = 0;
    public double intakeVoltageV = 0;
    public double pivotVoltageV = 0;
    public double intakeTempC = 0;
    public double pivotTempC = 0;
  }

  public default void updateInputs(IntakeIOInputs inputs) {}
}
