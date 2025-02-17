// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ReverseIntakeSetCmd extends Command {

  /** Creates a new ReverseIntakeSetCmd. */
  private IntakeSubsystem s_intake;
  private ShooterSubsystem s_shooter;
  private double speed;

  public ReverseIntakeSetCmd(IntakeSubsystem intakesub, ShooterSubsystem shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_intake = intakesub;
    this.s_shooter = shooter;
    this.speed = speed;
    addRequirements(intakesub, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intake.setmotor(speed);
   s_shooter.setmotor(0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.setmotor(0);
    s_shooter.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
