// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;


public class IntakeSetCmd extends Command {

  private IntakeSubsystem s_intake;
  private ShooterSubsystem s_shooter;
  private double speed;

  //private ShooterSubsystem s_shooterlowmotor;

  /** Creates a new IntakeSetCmd. */
  public IntakeSetCmd(IntakeSubsystem intakesub, ShooterSubsystem shooter, double speed) {
    this.s_intake = intakesub;
    this.s_shooter = shooter;
    this.speed = speed;
    addRequirements(intakesub, shooter);
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_intake.setred();  
    System.out.println("hi there woah 0-0");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_intake.setmotor(speed);
    s_shooter.setmotor(-0.25);
  

   /*if(s_pivot.pivotencoder()>290){
    s_intake.setmotor(0.35);
    s_shooter.setmotor(-0.1);
    SmartDashboard.putString("Intake State", "Intake Running");
    } */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_intake.setmotor(0);
    s_shooter.setmotor(0);
    s_intake.setledoff();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_intake.get_sensor() == false){
      s_intake.setgreen();
      return true;
    }
    else 
      return false;
    
    
  }
}
