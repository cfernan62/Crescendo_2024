// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class AutoIntakeCmd extends Command {

  private IntakeSubsystem s_intake;
  private ShooterSubsystem s_shooter;
  private PivotSubsystem s_pivot;
  private double speed;
  /** Creates a new AutoIntakeCmd. */
  public AutoIntakeCmd(IntakeSubsystem intake, ShooterSubsystem shooter, PivotSubsystem pivot) {
    this.s_intake = intake;
    this.s_shooter = shooter;
    this.s_pivot = pivot;
    this.speed = 0.3;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Auto Picking up Ring");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intake.setmotor(speed);
    s_shooter.setmotor(-0.1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.setmotor(0);
    s_shooter.setmotor(0);
    s_pivot.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!s_intake.get_sensor());
  }
}
