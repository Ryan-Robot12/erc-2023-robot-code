// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ForkliftConstants;
import frc.robot.Constants.HardwareConstants;

public class ForkliftSubsystem extends SubsystemBase {
  private TalonFX wheelMotor;
  private TalonFX liftMotor;

  private StatusSignal<Double> liftPos;

  public ForkliftSubsystem() {
    wheelMotor = new TalonFX(ForkliftConstants.WHEEL_MOTOR_ID);
    liftMotor = new TalonFX(ForkliftConstants.LIFT_MOTOR_ID);

    TalonFXConfiguration liftConfig = new TalonFXConfiguration();
    liftConfig.Slot0.kP = ForkliftConstants.LIFT_P;
    liftConfig.Slot0.kI = ForkliftConstants.LIFT_I;
    liftConfig.Slot0.kD = ForkliftConstants.LIFT_D;
    liftConfig.Slot0.kS = ForkliftConstants.LIFT_S;
    liftConfig.Slot0.kV = ForkliftConstants.LIFT_V;
    liftConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    liftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    liftConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    liftConfig.MotorOutput.DutyCycleNeutralDeadband = HardwareConstants.MIN_FALCON_DEADBAND;
    liftMotor.getConfigurator().apply(liftConfig, HardwareConstants.TIMEOUT_S);
    liftPos = liftMotor.getPosition();
  }

  public void manualControl(double wheelSpeed, double liftSpeed) {
    wheelMotor.set(wheelSpeed);
    liftMotor.set(liftSpeed);
  }

  public void setWheelSpeed(double speed) {
    wheelMotor.set(speed);
  }

  /**
   * Sets the height of the system
   * @param height (meters)
   */
  public void setHeight(double height) {
    height *= ForkliftConstants.METERS_TO_LIFT_POS;
    // TODO: feedforward?
    MotionMagicVoltage req = new MotionMagicVoltage(height);
    liftMotor.setControl(req);
  }

  public double getHeight() {
    liftPos.refresh();
    return liftPos.getValue() / ForkliftConstants.METERS_TO_LIFT_POS;
  }

  @Override
  public void periodic() {
  }
}
