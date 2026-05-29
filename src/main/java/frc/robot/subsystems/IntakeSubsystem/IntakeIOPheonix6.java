package frc.robot.subsystems.IntakeSubsystem;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.Constants;

/**
 * Intake Phoenix 6 实现。写法对齐 drive 层 {@code ModuleIOTalonFX}:实例字段、cached signals +
 * refreshAll、Debouncer 连接检测、50 Hz + optimizeBusUtilization、配置经 tryUntilOk。
 *
 * <p>保留旧硬件语义:intake Coast/顺时针为正;pivot Brake + Slot0(含 kG)+ MotionMagic + SensorToMechanismRatio,
 * 构造末尾 setPosition(0)。pivot 位置以度对外(rotations→degrees)。
 */
public class IntakeIOPheonix6 implements IntakeIO {
  private final TalonFX intakeMotor;
  private final TalonFX pivotMotor;

  private final VelocityTorqueCurrentFOC intakeVelocityRequest = new VelocityTorqueCurrentFOC(0.0);
  private final MotionMagicVoltage pivotMotionMagicRequest = new MotionMagicVoltage(0.0);
  private final VoltageOut intakeVoltageRequest = new VoltageOut(0.0);
  private final VoltageOut pivotVoltageRequest = new VoltageOut(0.0);

  private final StatusSignal<AngularVelocity> intakeVel, pivotVel;
  private final StatusSignal<Angle> pivotPos;
  private final StatusSignal<Voltage> intakeVolts, pivotVolts;
  private final StatusSignal<Current> intakeStator, pivotStator;
  private final StatusSignal<Current> intakeSupply, pivotSupply;
  private final StatusSignal<Temperature> intakeTemp, pivotTemp;

  private final edu.wpi.first.math.filter.Debouncer intakeDebounce =
      new edu.wpi.first.math.filter.Debouncer(
          0.5, edu.wpi.first.math.filter.Debouncer.DebounceType.kFalling);
  private final edu.wpi.first.math.filter.Debouncer pivotDebounce =
      new edu.wpi.first.math.filter.Debouncer(
          0.5, edu.wpi.first.math.filter.Debouncer.DebounceType.kFalling);

  public IntakeIOPheonix6() {
    intakeMotor =
        new TalonFX(Constants.MotorCANIds.intakeMotorCANId, Constants.MotorCANIds.CanBusName);
    pivotMotor =
        new TalonFX(Constants.MotorCANIds.intakePivotMotorCANId, Constants.MotorCANIds.CanBusName);

    TalonFXConfiguration intakeConfig = new TalonFXConfiguration();
    intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    intakeConfig.Slot0 = new Slot0Configs();
    intakeConfig.Slot0.kP = Constants.IntakePID.intakeKP;
    intakeConfig.Slot0.kI = Constants.IntakePID.intakeKI;
    intakeConfig.Slot0.kD = Constants.IntakePID.intakeKD;
    intakeConfig.Slot0.kS = Constants.IntakePID.intakeKS;
    intakeConfig.Slot0.kV = Constants.IntakePID.intakeKV;
    intakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    tryUntilOk(5, () -> intakeMotor.getConfigurator().apply(intakeConfig, 0.25));

    TalonFXConfiguration pivotConfig = new TalonFXConfiguration();
    pivotConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    pivotConfig.Slot0 = new Slot0Configs();
    pivotConfig.Slot0.kP = Constants.IntakePID.pivotKP;
    pivotConfig.Slot0.kI = Constants.IntakePID.pivotKI;
    pivotConfig.Slot0.kD = Constants.IntakePID.pivotKD;
    pivotConfig.Slot0.kS = Constants.IntakePID.pivotKS;
    pivotConfig.Slot0.kV = Constants.IntakePID.pivotKV;
    pivotConfig.Slot0.kG = Constants.IntakePID.pivotKG;
    pivotConfig.MotionMagic = new MotionMagicConfigs();
    pivotConfig.MotionMagic.MotionMagicCruiseVelocity =
        Constants.IntakePID.pivotMotionMagicCruiseVelocity;
    pivotConfig.MotionMagic.MotionMagicAcceleration =
        Constants.IntakePID.pivotMotionMagicAcceleration;
    pivotConfig.Feedback.SensorToMechanismRatio = Constants.IntakePID.pivotGearRatio;
    pivotConfig.MotionMagic.MotionMagicJerk = Constants.IntakePID.pivotMotionMagicJerk;
    tryUntilOk(5, () -> pivotMotor.getConfigurator().apply(pivotConfig, 0.25));
    pivotMotor.setPosition(0);

    intakeVel = intakeMotor.getVelocity();
    pivotVel = pivotMotor.getVelocity();
    pivotPos = pivotMotor.getPosition();
    intakeVolts = intakeMotor.getMotorVoltage();
    pivotVolts = pivotMotor.getMotorVoltage();
    intakeStator = intakeMotor.getStatorCurrent();
    pivotStator = pivotMotor.getStatorCurrent();
    intakeSupply = intakeMotor.getSupplyCurrent();
    pivotSupply = pivotMotor.getSupplyCurrent();
    intakeTemp = intakeMotor.getDeviceTemp();
    pivotTemp = pivotMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeVel,
        pivotVel,
        pivotPos,
        intakeVolts,
        pivotVolts,
        intakeStator,
        pivotStator,
        intakeSupply,
        pivotSupply,
        intakeTemp,
        pivotTemp);
    ParentDevice.optimizeBusUtilizationForAll(intakeMotor, pivotMotor);
  }

  @Override
  public void intakesetrps(double RPS) {
    intakeMotor.setControl(intakeVelocityRequest.withVelocity(RPS));
  }

  @Override
  public void IntakesetV(double voltage) {
    intakeMotor.setControl(intakeVoltageRequest.withOutput(voltage));
  }

  @Override
  public void Pivotsetangle(double degrees) {
    pivotMotor.setControl(pivotMotionMagicRequest.withPosition(Units.degreesToRotations(degrees)));
  }

  @Override
  public void pivotsetV(double voltage) {
    pivotMotor.setControl(pivotVoltageRequest.withOutput(voltage));
  }

  @Override
  public void pivotSetZero() {
    pivotMotor.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    var ik =
        BaseStatusSignal.refreshAll(intakeVel, intakeVolts, intakeStator, intakeSupply, intakeTemp);
    var pv =
        BaseStatusSignal.refreshAll(
            pivotPos, pivotVel, pivotVolts, pivotStator, pivotSupply, pivotTemp);

    inputs.intakeConnected = intakeDebounce.calculate(ik.isOK());
    inputs.pivotConnected = pivotDebounce.calculate(pv.isOK());

    inputs.intakeVelocityRPS = intakeVel.getValueAsDouble();
    inputs.PivotAngledegrees = Units.rotationsToDegrees(pivotPos.getValueAsDouble());
    inputs.PivotVelocityRPS = pivotVel.getValueAsDouble();
    inputs.intakeVoltageV = intakeVolts.getValueAsDouble();
    inputs.pivotVoltageV = pivotVolts.getValueAsDouble();
    inputs.intakeStatorAMPS = intakeStator.getValueAsDouble();
    inputs.pivotStatorAMPS = pivotStator.getValueAsDouble();
    inputs.intakeCurrentAMPS = intakeSupply.getValueAsDouble();
    inputs.pivotCurrentAMPS = pivotSupply.getValueAsDouble();
    inputs.intakeTempC = intakeTemp.getValueAsDouble();
    inputs.pivotTempC = pivotTemp.getValueAsDouble();
  }
}
