package frc.robot.subsystems.ShooterSubsystem;

/**
 * Shooter 的 sim 实现 —— <b>仅用于 WPILib 仿真,绝不上真车</b>(由 Constants.currentMode==SIM 选中)。
 *
 * <p>这是<b>逻辑测试用的一阶近似</b>,不是标定过的物理模型:飞轮/hood 用一阶滞后逼近设定值,目的是让 superstructure 的"飞轮到速 / hood 到角"门控在 sim
 * 里能真正触发(否则空 IO 永远返回 0,门控测不了)。 时间常数是拍的,只为 sim 手感;真实动态以上车为准。
 */
public class ShooterIOSim implements ShooterIO {
  private static final double DT = 0.02;
  private static final double FLY_TAU = 0.3; // 飞轮 spin-up 时间常数(s,sim 手感)
  private static final double HOOD_TAU = 0.2; // hood 到位时间常数(s,sim 手感)
  private static final double KRAKEN_FREE_RPS = 100.0; // voltage→rps 粗映射用

  private double targetRps = 0.0;
  private double flyRps = 0.0;
  private double targetHoodRot = 0.0;
  private double hoodRot = 0.0;
  private double hoodVolts = 0.0;

  @Override
  public void ShooterSetRps(double rps) {
    targetRps = rps;
  }

  @Override
  public void ShooterSetV(double voltage) {
    targetRps = voltage / 12.0 * KRAKEN_FREE_RPS;
  }

  @Override
  public void HoodSetAngle(double rotations) {
    targetHoodRot = rotations;
    hoodVolts = 0.0;
  }

  @Override
  public void HoodSetV(double voltage) {
    hoodVolts = voltage;
  }

  @Override
  public void HoodSetZero() {
    hoodRot = 0.0;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    // 飞轮:一阶逼近目标转速
    flyRps += (targetRps - flyRps) * DT / FLY_TAU;

    // hood:归位电压模式 → 按电压方向匀速移动;否则一阶逼近目标角
    if (hoodVolts != 0.0) {
      hoodRot += hoodVolts / 12.0 * 1.0 * DT;
    } else {
      hoodRot += (targetHoodRot - hoodRot) * DT / HOOD_TAU;
    }

    inputs.Shooter1Connected = true;
    inputs.Shooter2Connected = true;
    inputs.Shooter3Connected = true;
    inputs.Shooter1VelocityRPS = flyRps;
    inputs.Shooter2VelocityRPS = flyRps;
    inputs.Shooter3VelocityRPS = flyRps;
    inputs.Shooter1AppliedVolts = flyRps / KRAKEN_FREE_RPS * 12.0;
    inputs.Shooter2AppliedVolts = inputs.Shooter1AppliedVolts;
    inputs.Shooter3AppliedVolts = inputs.Shooter1AppliedVolts;
    inputs.Shooter1StatorAmps = Math.abs(targetRps - flyRps) * 2.0;
    inputs.Shooter2StatorAmps = inputs.Shooter1StatorAmps;
    inputs.Shooter3StatorAmps = inputs.Shooter1StatorAmps;
    inputs.Shooter1TempC = 25.0;
    inputs.Shooter2TempC = 25.0;
    inputs.Shooter3TempC = 25.0;
    inputs.ShooterRPS = flyRps;
    inputs.ShooterCurrentAMPS = inputs.Shooter1StatorAmps;

    inputs.HoodConnected = true;
    inputs.HoodAngle = hoodRot;
    inputs.HoodVelocityRPS = 0.0;
    inputs.HoodVoltageV = hoodVolts;
    // 归位逻辑靠电流阈值退出:sim 里施加归位电压时报一个超阈值电流,让 runHoodHoming 能在 sim 完成而不挂死
    inputs.HoodCurrentAMPS = (hoodVolts != 0.0) ? 5.0 : 0.0;
    inputs.HoodStatorAmps = inputs.HoodCurrentAMPS;
    inputs.HoodTempC = 25.0;
  }
}
