package com.slsh.IDeer9427.lib.Vision;

import static java.lang.Math.toRadians;

import java.util.Arrays;

/** Limelight3 實作：讀取 coral 偵測的關鍵欄位（Neural Detector 友善） */
public class LL3GamepieceVision implements GamepieceVisionIO {
  private final String tableName;
  private final VisionConfig cfg;

  public LL3GamepieceVision(String limelightName, VisionConfig cfg) {
    this.tableName = limelightName;
    this.cfg = cfg;
  }

  @Override
  public Measurement get() {
    // ---------- 路徑 1：t2d ----------
    double[] t2d = LimelightHelpers.getT2DArray(tableName);
    if (t2d.length >= 16 && t2d[0] == 1.0) { // valid
      double shortPx = t2d[13]; // targetShortSidePixels
      double horExtPx = t2d[14]; // targetHorizontalExtentPixels
      double verExtPx = t2d[15]; // targetVerticalExtentPixels
      double txRad = toRadians(t2d[4]); // tx[deg] → rad
      double tyRad = toRadians(t2d[5]); // ty[deg] → rad

      // 像素中心（正確比例如下：tx 相對「半視場角」線性映射）
      double cx =
          cfg.imageWidthPx() * 0.5 + (txRad / (cfg.hfovRad() * 0.5)) * (cfg.imageWidthPx() * 0.5);
      double cy =
          cfg.imageHeightPx() * 0.5 - (tyRad / (cfg.vfovRad() * 0.5)) * (cfg.imageHeightPx() * 0.5);

      double hx = 0.5 * horExtPx;
      double hy = 0.5 * verExtPx;

      boolean cut =
          (cx - hx <= 1.0)
              || (cx + hx >= cfg.imageWidthPx() - 1.0)
              || (cy - hy <= 1.0)
              || (cy + hy >= cfg.imageHeightPx() - 1.0)
              || (shortPx < 1.0);

      return new Measurement(true, shortPx, txRad, tyRad, cut);
    }

    // ---------- 路徑 2：rawdetections ----------
    LimelightHelpers.RawDetection[] dets = LimelightHelpers.getRawDetections(tableName);
    if (dets.length > 0) {
      var d = dets[0];

      // 角度：txnc/tync ∈ [-1,1] → 相對半視場角
      double txRad = d.txnc * (cfg.hfovRad() * 0.5);
      double tyRad = d.tync * (cfg.vfovRad() * 0.5);

      // 以四角換像素寬高（-1..1 → px）
      double[] xs = {d.corner0_X, d.corner1_X, d.corner2_X, d.corner3_X};
      double[] ys = {d.corner0_Y, d.corner1_Y, d.corner2_Y, d.corner3_Y};
      double minx = Arrays.stream(xs).min().orElse(0.0);
      double maxx = Arrays.stream(xs).max().orElse(0.0);
      double miny = Arrays.stream(ys).min().orElse(0.0);
      double maxy = Arrays.stream(ys).max().orElse(0.0);

      double wPx = (maxx - minx) * 0.5 * cfg.imageWidthPx();
      double hPx = (maxy - miny) * 0.5 * cfg.imageHeightPx();
      double shortPx = Math.min(wPx, hPx);

      // 邊界裁切（以標準化座標直接判斷）
      boolean cut =
          (minx <= -1.0) || (maxx >= 1.0) || (miny <= -1.0) || (maxy >= 1.0) || (shortPx < 1.0);

      if (shortPx > 0.0) return new Measurement(true, shortPx, txRad, tyRad, cut);
    }

    // 沒目標
    return new Measurement(false, 0, 0, 0, false);
  }
}
