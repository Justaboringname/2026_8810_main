package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

/**
 * Limelight MegaTag2 视觉的薄 IO seam。
 *
 * <p>设计约束(用户拍板):<b>不改动 {@link frc.robot.subsystems.drive.Drive} 里已上场验证的融合/接受门控逻辑</b>, 只把原本内联在
 * Drive.updateVision() 里直接调 {@code LimelightHelpers} 的部分收到这个接口后面。好处:
 *
 * <ul>
 *   <li>AdvantageKit replay 确定性:NT 读取变成 logged inputs,replay 时由 {@code Logger.processInputs} 回放;
 *   <li>可测/可替换:sim/replay 用空实现,不依赖真实 Limelight。
 * </ul>
 *
 * 融合数学仍在 Drive(读这里的 inputs,逐字保持原门控:|omega|≤2π、tagCount>0、avgTagDist<4、平移速度<2)。
 */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    /** MT2 估计是否可用(原逻辑里 mt2 != null)。false 时 Drive 报 disconnected 并跳过。 */
    public boolean connected = false;

    /** MT2 估计的机器人位姿(wpiBlue field frame)。 */
    public Pose2d estimatedPose = Pose2d.kZero;

    public double timestampSeconds = 0.0;
    public int tagCount = 0;
    public double avgTagDist = 0.0;
    public double avgTagArea = 0.0;
  }

  /**
   * 推当前机器人 yaw 给相机(MegaTag2 需要 gyro 朝向),然后读最新 MT2 估计填进 inputs。
   *
   * @param inputs 待填充的 logged inputs
   * @param robotYawDeg 机器人当前 yaw(度,field frame),来自 drive pose
   */
  public default void updateInputs(VisionIOInputs inputs, double robotYawDeg) {}
}
