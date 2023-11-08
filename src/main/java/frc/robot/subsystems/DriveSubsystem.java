// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants.DriveConstants;
import frc.robot.sensors.RomiGyro;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {  
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private DifferentialDrive diffDrive;

  private RomiGyro gyro;

  /** Creates a new Drivetrain. */
  public DriveSubsystem() {
    leftMotor = new TalonFX(DriveConstants.LEFT_MOTOR_ID);
    rightMotor = new TalonFX(DriveConstants.RIGHT_MOTOR_ID);

    diffDrive = new DifferentialDrive(leftMotor, rightMotor);

    gyro = new RomiGyro();
  }

  public void drive(double ySpeed, double rotation) {
    diffDrive.curvatureDrive(ySpeed, rotation, true);
  }

  public Rotation3d getGyro() {
    return new Rotation3d(Math.toRadians(gyro.getAngleY()), Math.toRadians(gyro.getAngleX()), Math.toRadians(gyro.getAngleZ()));
  }

  @Override
  public void periodic() {
  }
}
