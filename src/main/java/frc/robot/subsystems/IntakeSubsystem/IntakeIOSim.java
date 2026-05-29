package frc.robot.subsystems.IntakeSubsystem;

/**
 * Intake 的 sim 实现 —— <b>仅用于 WPILib 仿真</b>(Constants.currentMode==SIM)。一阶滞后逼近,逻辑测试用,非标定物理。
 *
 * <p>pivot 归位:PivotInit 用 setPivotVoltage(-2) 跑到电流超阈值退出;sim 里施加电压时报超阈值电流,让归位在 sim 能完成。
 */
public class IntakeIOSim implements IntakeIO {
  private static final double DT = 0.02;
  private static final double ROLLER_TAU = 0.1;
  private static final double PIVOT_TAU = 0.25;
  private static final double KRAKEN_FREE_RPS = 100.0;

  private double intakeTarget = 0.0;
  private double intakeRps = 0.0;
  private double pivotTargetDeg = 0.0;
  private double pivotDeg = 0.0;
  private double pivotVolts = 0.0;

  @Override
  public void intakesetrps(double RPS) {
    intakeTarget = RPS;
  }

  @Override
  public void IntakesetV(double voltage) {
    intakeTarget = voltage / 12.0 * KRAKEN_FREE_RPS;
  }

  @Override
  public void Pivotsetangle(double degrees) {
    pivotTargetDeg = degrees;
    pivotVolts = 0.0;
  }

  @Override
  public void pivotsetV(double voltage) {
    pivotVolts = voltage;
  }

  @Override
  public void pivotSetZero() {
    pivotDeg = 0.0;
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    intakeRps += (intakeTarget - intakeRps) * DT / ROLLER_TAU;

    // pivot:归位电压模式 → 按电压方向匀速移动;否则一阶逼近目标角
    if (pivotVolts != 0.0) {
      pivotDeg += pivotVolts / 12.0 * 90.0 * DT; // 满压约 90°/s,sim 手感
    } else {
      pivotDeg += (pivotTargetDeg - pivotDeg) * DT / PIVOT_TAU;
    }

    inputs.intakeConnected = true;
    inputs.pivotConnected = true;
    inputs.intakeVelocityRPS = intakeRps;
    inputs.PivotAngledegrees = pivotDeg;
    inputs.PivotVelocityRPS = 0.0;
    inputs.intakeVoltageV = intakeRps / KRAKEN_FREE_RPS * 12.0;
    inputs.pivotVoltageV = pivotVolts;
    inputs.intakeStatorAMPS = Math.abs(intakeTarget - intakeRps) * 2.0;
    // PivotInit 靠电流阈值退出:施加归位电压时报超阈值电流,让归位在 sim 完成
    inputs.pivotStatorAMPS = (pivotVolts != 0.0) ? 10.0 : 0.0;
    inputs.intakeCurrentAMPS = inputs.intakeStatorAMPS;
    inputs.pivotCurrentAMPS = inputs.pivotStatorAMPS;
    inputs.intakeTempC = 25.0;
    inputs.pivotTempC = 25.0;
  }
}
