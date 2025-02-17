// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class MoveToReadySetCmd extends Command {
  private PivotSubsystem s_pivot;
  private IntakeSubsystem s_intake;
  private ShooterSubsystem s_shooter;
  
  /** Creates a new MoveToReadySetCmd. */
  public MoveToReadySetCmd(PivotSubsystem pivotsub, IntakeSubsystem intakesub, ShooterSubsystem shootersub) {
    this.s_pivot = pivotsub;
    this.s_intake = intakesub;
    this.s_shooter = shootersub;
    addRequirements(pivotsub, shootersub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_intake.setgreen();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_pivot.ToScore();
    s_shooter.setmotor(-1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.setledoff();
    s_shooter.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
