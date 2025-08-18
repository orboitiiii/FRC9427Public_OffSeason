package frc.robot.subsystems.IntakeSubsystem.PivotSystem;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

public class IntakePivotConfig {
  public static SparkFlexConfig getLeftArmMotorConfig() {
    SparkFlexConfig cfg = new SparkFlexConfig();
    cfg.encoder
        .positionConversionFactor(IntakePivotConstants.System.POS_RAD_PER_ROT)
        .velocityConversionFactor(IntakePivotConstants.System.VEL_RAD_PER_RPM)
        .uvwMeasurementPeriod(10)
        .uvwAverageDepth(2);

    cfg.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(67).voltageCompensation(12.0);

    cfg.signals
        .primaryEncoderPositionAlwaysOn(true)
        .primaryEncoderVelocityAlwaysOn(true)
        .primaryEncoderVelocityPeriodMs(20)
        .appliedOutputPeriodMs(20)
        .busVoltagePeriodMs(20)
        .outputCurrentPeriodMs(20);

    cfg.closedLoop
        .pid(
            IntakePivotConstants.Physic.kP_V_PER_RAD,
            0.0,
            IntakePivotConstants.Physic.kD_V_PER_RAD_PER_S)
        .outputRange(-0.05, 0.25);
    // .maxMotion
    // .maxVelocity(IntakePivotConstants.Physic.CRUISE_VEL_RAD_PER_S)
    // .maxAcceleration(IntakePivotConstants.Physic.MAX_ACCEL_RAD_PER_S2)
    // .allowedClosedLoopError(0.0);

    return cfg;
  }

  public static SparkFlexConfig getRightArmMotorConfig() {
    SparkFlexConfig cfg = new SparkFlexConfig();
    // cfg.encoder
    //     .positionConversionFactor(IntakePivotConstants.System.POS_RAD_PER_ROT)
    //     .velocityConversionFactor(IntakePivotConstants.System.VEL_RAD_PER_RPM)
    //     .uvwMeasurementPeriod(10)
    //     .uvwAverageDepth(2);

    cfg.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(67).voltageCompensation(12.0);

    // cfg.signals
    //     .primaryEncoderPositionAlwaysOn(true)
    //     .primaryEncoderVelocityAlwaysOn(true)
    //     .primaryEncoderVelocityPeriodMs(20)
    //     .appliedOutputPeriodMs(20)
    //     .busVoltagePeriodMs(20)
    //     .outputCurrentPeriodMs(20);

    // cfg.closedLoop
    //     .pid(
    //         IntakePivotConstants.Physic.kP_V_PER_RAD,
    //         0.0,
    //         IntakePivotConstants.Physic.kD_V_PER_RAD_PER_S)
    //     .maxMotion
    //     .maxVelocity(IntakePivotConstants.Physic.CRUISE_VEL_RAD_PER_S)
    //     .maxAcceleration(IntakePivotConstants.Physic.MAX_ACCEL_RAD_PER_S2)
    //     .allowedClosedLoopError(0.0);

    return cfg;
  }
}
