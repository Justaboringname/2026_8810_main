package frc.robot.subsystems.FeederSubsystem;

/** Feeder 的 sim 实现 —— <b>仅用于 WPILib 仿真</b>(Constants.currentMode==SIM)。一阶滞后逼近,逻辑测试用,非标定物理。 */
public class FeederIOSim implements FeederIO {
  private static final double DT = 0.02;
  private static final double TAU = 0.1;
  private static final double KRAKEN_FREE_RPS = 100.0;

  private double beltTarget = 0.0;
  private double beltRps = 0.0;
  private double indexerTarget = 0.0;
  private double indexerRps = 0.0;

  @Override
  public void BeltSetRps(double RPS) {
    beltTarget = RPS;
  }

  @Override
  public void IndexerSetRps(double RPS) {
    indexerTarget = RPS * 12 / 36; // 与真实实现的减速比换算一致
  }

  @Override
  public void BeltSetV(double voltage) {
    beltTarget = voltage / 12.0 * KRAKEN_FREE_RPS;
  }

  @Override
  public void IndexerSetV(double voltage) {
    indexerTarget = voltage / 12.0 * KRAKEN_FREE_RPS;
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    beltRps += (beltTarget - beltRps) * DT / TAU;
    indexerRps += (indexerTarget - indexerRps) * DT / TAU;

    inputs.BeltConnected = true;
    inputs.IndexerConnected = true;
    inputs.BeltVelocityRPS = beltRps;
    inputs.IndexerVelocityRPS = indexerRps;
    inputs.BeltVoltageV = beltRps / KRAKEN_FREE_RPS * 12.0;
    inputs.IndexerVoltageV = indexerRps / KRAKEN_FREE_RPS * 12.0;
    inputs.BeltStatorAMPS = Math.abs(beltTarget - beltRps) * 2.0;
    inputs.IndexerStatorAMPS = Math.abs(indexerTarget - indexerRps) * 2.0;
    inputs.BeltCurrentAMPS = inputs.BeltStatorAMPS;
    inputs.IndexerCurrentAMPS = inputs.IndexerStatorAMPS;
    inputs.BeltTempC = 25.0;
    inputs.IndexerTempC = 25.0;
  }
}
