package frc.robot.subsystems.vision;

import frc.robot.util.LimelightHelpers;

/**
 * VisionIO 的真实实现:把原 Drive.updateVision() 里那几行 {@code LimelightHelpers} 调用原样搬过来。
 *
 * <p>行为与重构前逐字一致:SetIMUMode(1) + SetRobotOrientation(gyro yaw) +
 * getBotPoseEstimate_wpiBlue_MegaTag2;mt2 为 null 视作未连接。<b>不</b>在这里做任何接受/拒绝判断—— 那是 Drive 融合门控的职责。
 */
public class VisionIOLimelight implements VisionIO {
  private final String name;

  public VisionIOLimelight(String name) {
    this.name = name;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs, double robotYawDeg) {
    LimelightHelpers.SetIMUMode(name, 1);
    LimelightHelpers.SetRobotOrientation(name, robotYawDeg, 0, 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);

    if (mt2 == null) {
      inputs.connected = false;
      return;
    }
    inputs.connected = true;
    inputs.estimatedPose = mt2.pose;
    inputs.timestampSeconds = mt2.timestampSeconds;
    inputs.tagCount = mt2.tagCount;
    inputs.avgTagDist = mt2.avgTagDist;
    inputs.avgTagArea = mt2.avgTagArea;
  }
}
