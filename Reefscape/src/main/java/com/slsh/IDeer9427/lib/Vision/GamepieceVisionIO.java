package com.slsh.IDeer9427.lib.Vision;


public interface GamepieceVisionIO {
  public static record Measurement(
      boolean hasTarget,
      double shortSidePx, // 旋轉外接矩形的短邊像素（Limelight tshort）
      double txRad,       // 水平偏角（弧度）
      double tyRad,       // 垂直偏角（弧度）
      boolean cutOff      // 影像邊界截斷或寬度不可靠
  ) {}
  Measurement get();
}
