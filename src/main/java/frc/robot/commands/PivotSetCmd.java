// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;


public class PivotSetCmd extends Command {
  private PivotSubsystem s_pivot;
  private double speed;
  /** Creates a new PivotSetCmd. */
  public PivotSetCmd(PivotSubsystem pivotsub, double speed) {
    this.s_pivot = pivotsub;
    this.speed = speed;
    addRequirements(pivotsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    
    s_pivot.setmotor(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_pivot.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
