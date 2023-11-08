// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ForkliftConstants;
import frc.robot.Constants.HardwareConstants;
import frc.robot.subsystems.ForkliftSubsystem;

public class PickupLoadingStation extends CommandBase {
  private ForkliftSubsystem forkliftSubsystem;

  /** Creates a new PlaceHighGoal. */
  public PickupLoadingStation(ForkliftSubsystem forkliftSubsystem) {
    this.forkliftSubsystem = forkliftSubsystem;
    addRequirements(forkliftSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    forkliftSubsystem.setHeight(ForkliftConstants.PICKUP_HEIGHT);
    forkliftSubsystem.setWheelSpeed(ForkliftConstants.PICKUP_SPEED);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    forkliftSubsystem.setHeight(forkliftSubsystem.getHeight());
    forkliftSubsystem.setWheelSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(forkliftSubsystem.getHeight() - ForkliftConstants.PICKUP_SPEED) < HardwareConstants.MIN_FALCON_DEADBAND;
  }
}
