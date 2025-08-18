package com.slsh.IDeer9427.lib.Vision;

import edu.wpi.first.math.geometry.Translation2d;

/** 幾何與相機設定 */
public record VisionConfig(
    // 影像解析度與視場
    int imageWidthPx,
    int imageHeightPx,
    double hfovRad,
    double vfovRad,

    // 目標與相機幾何
    double gamepieceDiameterM, // Coral 直徑（公尺）
    double camHeightM,
    double gamepieceHeightM, // 地面物體填 0
    double camPitchRad, // 相機俯仰（+上仰）
    double camYawOffsetRad, // 相機相對機身 yaw 偏置
    Translation2d robotToCam, // 機身到相機（機器座標，X 向前，Y 向左）

    // 距離估計閾值
    int minShortPxForWidthMethod // 使用短邊法的最小像素
    ) {}
