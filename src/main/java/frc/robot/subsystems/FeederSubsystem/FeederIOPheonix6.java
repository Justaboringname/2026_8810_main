package frc.robot.subsystems.FeederSubsystem;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Feeder Phoenix 6 实现。写法对齐 drive 层 {@code ModuleIOTalonFX} / shooter:实例字段(非 static)、 cached
 * StatusSignals + {@code BaseStatusSignal.refreshAll}、{@code Debouncer} 连接检测、 50 Hz 帧率 + {@code
 * optimizeBusUtilization}、配置经 {@code tryUntilOk} 重试。
 *
 * <p>保留旧版的全部硬件语义:indexer 逆时针为正、belt 顺时针为正、双 Coast、indexer 转速请求 ×12/36 减速比换算。
 */
public class FeederIOPheonix6 implements FeederIO {
  private final TalonFX indexerMotor;
  private final TalonFX beltMotor;

  private final VoltageOut indexerVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut beltVoltageRequest = new VoltageOut(0.0);
  private final VelocityTorqueCurrentFOC indexerVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final VelocityTorqueCurrentFOC beltVelocityRequest = new VelocityTorqueCurrentFOC(0.0);

  private final StatusSignal<AngularVelocity> indexerVel, beltVel;
  private final StatusSignal<Voltage> indexerVolts, beltVolts;
  private final StatusSignal<Current> indexerStator, beltStator;
  private final StatusSignal<Current> indexerSupply, beltSupply;
  private final StatusSignal<Temperature> indexerTemp, beltTemp;

  private final Debouncer indexerDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer beltDebounce = new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public FeederIOPheonix6() {
    indexerMotor =
        new TalonFX(Constants.MotorCANIds.indexerMotorCANId, Constants.MotorCANIds.CanBusName);
    beltMotor = new TalonFX(Constants.MotorCANIds.beltMotorCANId, Constants.MotorCANIds.CanBusName);

    TalonFXConfiguration indexerConfig = new TalonFXConfiguration();
    indexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    indexerConfig.Slot0 = new Slot0Configs();
    indexerConfig.Slot0.kP = Constants.FeederSubsystemPID.indexerKP;
    indexerConfig.Slot0.kI = Constants.FeederSubsystemPID.indexerKI;
    indexerConfig.Slot0.kD = Constants.FeederSubsystemPID.indexerKD;
    indexerConfig.Slot0.kS = Constants.FeederSubsystemPID.indexerKS;
    indexerConfig.Slot0.kV = Constants.FeederSubsystemPID.indexerKV;
    indexerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    tryUntilOk(5, () -> indexerMotor.getConfigurator().apply(indexerConfig, 0.25));

    TalonFXConfiguration beltConfig = new TalonFXConfiguration();
    beltConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    beltConfig.Slot0 = new Slot0Configs();
    beltConfig.Slot0.kP = Constants.FeederSubsystemPID.beltKP;
    beltConfig.Slot0.kI = Constants.FeederSubsystemPID.beltKI;
    beltConfig.Slot0.kD = Constants.FeederSubsystemPID.beltKD;
    beltConfig.Slot0.kS = Constants.FeederSubsystemPID.beltKS;
    beltConfig.Slot0.kV = Constants.FeederSubsystemPID.beltKV;
    beltConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> beltMotor.getConfigurator().apply(beltConfig, 0.25));

    indexerVel = indexerMotor.getVelocity();
    beltVel = beltMotor.getVelocity();
    indexerVolts = indexerMotor.getMotorVoltage();
    beltVolts = beltMotor.getMotorVoltage();
    indexerStator = indexerMotor.getStatorCurrent();
    beltStator = beltMotor.getStatorCurrent();
    indexerSupply = indexerMotor.getSupplyCurrent();
    beltSupply = beltMotor.getSupplyCurrent();
    indexerTemp = indexerMotor.getDeviceTemp();
    beltTemp = beltMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerVel,
        beltVel,
        indexerVolts,
        beltVolts,
        indexerStator,
        beltStator,
        indexerSupply,
        beltSupply,
        indexerTemp,
        beltTemp);
    ParentDevice.optimizeBusUtilizationForAll(indexerMotor, beltMotor);
  }

  @Override
  public void BeltSetRps(double RPS) {
    beltMotor.setControl(beltVelocityRequest.withVelocity(RPS));
  }

  @Override
  public void IndexerSetRps(double RPS) {
    indexerMotor.setControl(indexerVelocityRequest.withVelocity(RPS * 12 / 36));
  }

  @Override
  public void BeltSetV(double voltage) {
    beltMotor.setControl(beltVoltageRequest.withOutput(voltage));
  }

  @Override
  public void IndexerSetV(double voltage) {
    indexerMotor.setControl(indexerVoltageRequest.withOutput(voltage));
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    var idx =
        BaseStatusSignal.refreshAll(
            indexerVel, indexerVolts, indexerStator, indexerSupply, indexerTemp);
    var blt = BaseStatusSignal.refreshAll(beltVel, beltVolts, beltStator, beltSupply, beltTemp);

    inputs.IndexerConnected = indexerDebounce.calculate(idx.isOK());
    inputs.BeltConnected = beltDebounce.calculate(blt.isOK());

    inputs.IndexerVelocityRPS = indexerVel.getValueAsDouble();
    inputs.BeltVelocityRPS = beltVel.getValueAsDouble();
    inputs.IndexerVoltageV = indexerVolts.getValueAsDouble();
    inputs.BeltVoltageV = beltVolts.getValueAsDouble();
    inputs.IndexerStatorAMPS = indexerStator.getValueAsDouble();
    inputs.BeltStatorAMPS = beltStator.getValueAsDouble();
    inputs.IndexerCurrentAMPS = indexerSupply.getValueAsDouble();
    inputs.BeltCurrentAMPS = beltSupply.getValueAsDouble();
    inputs.IndexerTempC = indexerTemp.getValueAsDouble();
    inputs.BeltTempC = beltTemp.getValueAsDouble();
  }
}
