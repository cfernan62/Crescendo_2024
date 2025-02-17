// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;

public class AutoPivotShoot extends Command {
  private PivotSubsystem s_pivot;
  /** Creates a new AutoPivotShoot. */
  public AutoPivotShoot(PivotSubsystem pivot) {
    this.s_pivot = pivot;
    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto Pivot to Shoot");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_pivot.AutoToScore();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_pivot.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    double error;
    error = PivotConstants.kShootingPoint - s_pivot.pivotencoder();
    if(Math.abs( error ) <= 1.2)
      return true; 
    else 
      return false;
  }
}
