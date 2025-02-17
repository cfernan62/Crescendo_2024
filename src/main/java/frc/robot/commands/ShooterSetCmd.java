// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class ShooterSetCmd extends Command {

  private ShooterSubsystem s_shooter;
  private IntakeSubsystem s_intake;
  private double velocity;

  private double speed;
  /** Creates a new ShooterSetCmd. */
  public ShooterSetCmd(ShooterSubsystem shootersub, IntakeSubsystem intakesub, double speed, double velocity) {
    this.s_shooter = shootersub;
    this.s_intake = intakesub;
    this.speed = speed;
    this.velocity = velocity;
    addRequirements(shootersub, intakesub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_shooter.setmotor(speed);

    // if shooter is at required speed enable the intake to feed
    // the NOTE to shooter
    //
    //
    if ( s_shooter.get_velocity() > Math.abs(velocity))
      s_intake.setmotor( 1 ); 
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_shooter.setmotor(0);
    s_intake.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (s_intake.get_sensor());
  }
}
