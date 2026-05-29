package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem.FeederSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShotCalculator.ShooterSetpoint;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.AimingParameters;
import frc.robot.subsystems.superstructure.ShotGate;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

/**
 * Auto 专用:<b>自动转底盘对准 HUB + spinup + hood + 就绪门控喂球</b>,喂够一段时间后自动结束。
 *
 * <p>转向用 {@link DriveCommands#joystickDriveAtAngle} 同款 {@link ProfiledPIDController}(带速度/加速度
 * 梯形约束),比旧 Aimbot 的裸 PID 更平滑(导师指定)。角度常量取 DriveCommands 里的同值 (那些常量在 DriveCommands 内是
 * private,无法跨类引用,这里复制并标注来源,改时两边同步)。
 *
 * <p>喂球门控复用已测纯逻辑 {@link ShotGate}(7 参版,<b>多一个"转向到位"前置门</b>)+ {@link
 * frc.robot.subsystems.ShooterSubsystem.ShotCalculator}(距离→setpoint + isValid)。 视觉(MegaTag2)已融进
 * drive pose estimator,所以"用 pose 算瞄准角"本身即视觉辅助瞄准。
 *
 * <p>去掉了旧 Aimbot 的 intake swing 3 状态机(导师说用不到)。requires drive+shooter+feeder。
 *
 * <h2>结束方式(导师选定)</h2>
 *
 * 门控全满足(对准+到速+hood到角+setpoint可信)并<b>持续喂球 {@value #FEED_DURATION_SEC}s</b> 后 {@link #isFinished} 返回
 * true 主动结束;NamedCommand 外层再包 {@code withTimeout} 兜底防卡死。 喂球中途若门控掉了(如被撞偏),累计计时清零,重新对准后再数。
 */
public class AutoAimAndShoot extends Command {
  private final Drive drive;
  private final ShooterSubsystem shooter;
  private final FeederSubsystem feeder;
  private final AimingParameters aiming;

  // --- 转向 ProfiledPID:取自 DriveCommands(private,故此处复制同值)---
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;
  private static final double AIM_TOLERANCE_RAD = Math.toRadians(2.0);
  private static final double AIM_VELOCITY_TOLERANCE = 0.1; // rad/s

  private final ProfiledPIDController aimController =
      new ProfiledPIDController(
          ANGLE_KP,
          0.0,
          ANGLE_KD,
          new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));

  // --- 喂球 ---
  private final LoggedTunableNumber feedIndexerVolts =
      new LoggedTunableNumber("AutoAimAndShoot/FeedIndexerVolts", 12.0);
  private final LoggedTunableNumber feedBeltVolts =
      new LoggedTunableNumber("AutoAimAndShoot/FeedBeltVolts", 12.0);

  /** hood 到角容差(度)。 */
  private static final double HOOD_TOLERANCE_DEG = 1.5;

  /** 持续喂球达到此时长(s)即认为打完一坨,主动结束(可调)。 */
  private static final double FEED_DURATION_SEC = 1.0;

  private final Timer feedTimer = new Timer();
  private boolean feedTimerRunning = false;

  public AutoAimAndShoot(
      Drive drive, ShooterSubsystem shooter, FeederSubsystem feeder, AimingParameters aiming) {
    this.drive = drive;
    this.shooter = shooter;
    this.feeder = feeder;
    this.aiming = aiming;
    aimController.enableContinuousInput(-Math.PI, Math.PI);
    aimController.setTolerance(AIM_TOLERANCE_RAD, AIM_VELOCITY_TOLERANCE);
    addRequirements(drive, shooter, feeder);
  }

  private boolean hoodAtAngle(double targetDeg) {
    double measuredDeg = shooter.getHoodAngle() * 360.0;
    return ShotGate.isNear(measuredDeg, targetDeg, HOOD_TOLERANCE_DEG);
  }

  @Override
  public void initialize() {
    aimController.reset(drive.getPose().getRotation().getRadians());
    feedTimer.stop();
    feedTimer.reset();
    feedTimerRunning = false;
  }

  @Override
  public void execute() {
    boolean enabled = !DriverStation.isDisabled();
    boolean alliancePresent = aiming.alliancePresent();
    ShooterSetpoint shot = aiming.currentShot();

    // ---- 转底盘对准 HUB(ProfiledPID,joystickDriveAtAngle 同款)----
    Pose2d pose = drive.getPose();
    double robotYaw = pose.getRotation().getRadians();
    double targetAngle =
        AimingParameters.targetHeadingRadians(pose.getX(), pose.getY(), aiming.isBlue());
    double omega = aimController.calculate(robotYaw, targetAngle);
    boolean aimed = aimController.atGoal();
    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(0, 0, omega), pose.getRotation()));

    // ---- spinup 飞轮 + hood(始终预热)----
    shooter.setShooterRps(shot.flywheelRps());
    shooter.setHoodAngle(shot.hoodAngleDeg());

    boolean shooterAtSpeed = shooter.isAtSetSpeed(shot.flywheelRps());
    boolean hoodAtAngle = hoodAtAngle(shot.hoodAngleDeg());

    // ---- 喂球门控:多一个"转向到位"前置门 ----
    boolean feeding =
        ShotGate.shouldFeed(
            true, enabled, alliancePresent, aimed, shooterAtSpeed, hoodAtAngle, shot);
    if (feeding) {
      feeder.setIndexerVoltage(feedIndexerVolts.get());
      feeder.setBeltVoltage(feedBeltVolts.get());
      if (!feedTimerRunning) {
        feedTimer.restart();
        feedTimerRunning = true;
      }
    } else {
      feeder.setIndexerVoltage(0.0);
      feeder.setBeltVoltage(0.0);
      // 门控掉了(被撞偏/掉速):停表清零,重新对准后再数
      feedTimer.stop();
      feedTimer.reset();
      feedTimerRunning = false;
    }

    Logger.recordOutput("AutoAimAndShoot/DistanceMeters", aiming.distanceMeters());
    Logger.recordOutput("AutoAimAndShoot/TargetAngleRad", targetAngle);
    Logger.recordOutput("AutoAimAndShoot/RobotYawRad", robotYaw);
    Logger.recordOutput("AutoAimAndShoot/Aimed", aimed);
    Logger.recordOutput("AutoAimAndShoot/TargetFlywheelRps", shot.flywheelRps());
    Logger.recordOutput("AutoAimAndShoot/TargetHoodDeg", shot.hoodAngleDeg());
    Logger.recordOutput("AutoAimAndShoot/ShotValid", shot.isValid());
    Logger.recordOutput("AutoAimAndShoot/AlliancePresent", alliancePresent);
    Logger.recordOutput("AutoAimAndShoot/ShooterAtSpeed", shooterAtSpeed);
    Logger.recordOutput("AutoAimAndShoot/HoodAtAngle", hoodAtAngle);
    Logger.recordOutput("AutoAimAndShoot/Feeding", feeding);
    Logger.recordOutput("AutoAimAndShoot/FeedElapsedSec", feedTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
    shooter.setShooterVoltage(0.0);
    feeder.setIndexerVoltage(0.0);
    feeder.setBeltVoltage(0.0);
    feedTimer.stop();
  }

  @Override
  public boolean isFinished() {
    // 持续喂球达 FEED_DURATION_SEC 即认为打完一坨,主动结束(外层 withTimeout 兜底)
    return feedTimerRunning && feedTimer.hasElapsed(FEED_DURATION_SEC);
  }
}
