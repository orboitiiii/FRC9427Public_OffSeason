package frc.robot.subsystems.IntakeSubsystem.PivotSystem;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.slsh.IDeer9427.lib.data.IDearLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeSubsystem.PivotSystem.IntakePivotConstants.Hardware;
import frc.robot.util.SparkFlexConfiguration;

public class IntakePivot extends SubsystemBase {
  final SparkFlex leftMotor;
  final SparkFlex rightMotor;

  private final SparkFlexConfiguration motorConfig =
      SparkFlexConfiguration.createIntakePivotMotor(Hardware.LeftMotorID, Hardware.rightMotorID);

  private final SparkClosedLoopController ctrl;

  public IntakePivot() {
    leftMotor = motorConfig.motors()[0];
    rightMotor = motorConfig.motors()[1];
    ctrl = leftMotor.getClosedLoopController();
    leftMotor.getEncoder().setPosition(0);
    IDearLog.getInstance()
        .addField(
            "PivotPosition",
            () -> Math.toDegrees(leftMotor.getEncoder().getPosition()),
            IDearLog.FieldType.CAN);

    IDearLog.getInstance()
        .addField("leftPower", () -> leftMotor.getAppliedOutput(), IDearLog.FieldType.CAN);

    IDearLog.getInstance()
        .addField("rightPower", () -> rightMotor.getAppliedOutput(), IDearLog.FieldType.CAN);
  }

  public void setVoltage(double volts) {
    leftMotor.setVoltage(volts);
    // rightMotor.setVoltage(-volts);
  }

  public void setPower(double power) {
    leftMotor.set(power);
    // rightMotor.set(-power);
  }

  public Command setPowerCommand(double power) {
    return Commands.runEnd(() -> setPower(power), () -> setVoltage(0), this);
  }

  public Command setVoltageCommand(double volts) {
    return Commands.runEnd(() -> setVoltage(volts), () -> setVoltage(0), this);
  }

  public Command setTarget(double target) {
    double setPoint = Math.toRadians(target);
    return Commands.runEnd(() -> setTargetAngleRad(setPoint), () -> setVoltage(0), this);
  }

  public void setTargetAngleRad(double thetaRad) {
    // double vff = Physic.kG_VOLTS * Math.cos(leftMotor.getEncoder().getPosition());
    ctrl.setReference(thetaRad, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
