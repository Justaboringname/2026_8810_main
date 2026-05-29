package frc.robot.subsystems.FeederSubsystem;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class FeederSubsystem extends SubsystemBase {
  private static FeederSubsystem m_Instance = null;

  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public static FeederSubsystem getInstance() {
    if (m_Instance == null) {
      throw new IllegalStateException("FeederSubsystem has not been constructed yet");
    }
    return m_Instance;
  }

  public FeederSubsystem() {
    if (m_Instance != null) {
      throw new IllegalStateException("FeederSubsystem already constructed");
    }
    m_Instance = this;
    switch (Constants.currentMode) {
      case REAL -> io = new FeederIOPheonix6();
      case SIM -> io = new FeederIOSim();
      default -> io = new FeederIO() {};
    }
  }

  public void setBeltRps(double rps) {
    io.BeltSetRps(rps);
  }

  public void setIndexerRps(double rps) {
    io.IndexerSetRps(rps);
  }

  public void setBeltVoltage(double voltage) {
    io.BeltSetV(voltage);
  }

  public void setIndexerVoltage(double voltage) {
    io.IndexerSetV(voltage);
  }

  public double getBeltVelocityRps() {
    return inputs.BeltVelocityRPS;
  }

  public double getIndexerVelocityRps() {
    return inputs.IndexerVelocityRPS;
  }

  public double getBeltCurrentAmps() {
    return inputs.BeltCurrentAMPS;
  }

  public double getIndexerCurrentAmps() {
    return inputs.IndexerCurrentAMPS;
  }

  public double getBeltVoltage() {
    return inputs.BeltVoltageV;
  }

  public double getIndexerVoltage() {
    return inputs.IndexerVoltageV;
  }

  public void processLog() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
  }

  public void processDashboard() {
    SmartDashboard.putNumber("Feeder/BeltVelocityRPS", inputs.BeltVelocityRPS);
    SmartDashboard.putNumber("Feeder/IndexerVelocityRPS", inputs.IndexerVelocityRPS);
    SmartDashboard.putNumber("Feeder/BeltCurrentAMPS", inputs.BeltCurrentAMPS);
    SmartDashboard.putNumber("Feeder/IndexerCurrentAMPS", inputs.IndexerCurrentAMPS);
    SmartDashboard.putNumber("Feeder/BeltVoltageV", inputs.BeltVoltageV);
    SmartDashboard.putNumber("Feeder/IndexerVoltageV", inputs.IndexerVoltageV);
  }

  @Override
  public void periodic() {
    processLog();
    processDashboard();
  }
}
