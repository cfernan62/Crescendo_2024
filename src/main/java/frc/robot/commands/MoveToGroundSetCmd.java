// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
public class MoveToGroundSetCmd extends Command {
  private PivotSubsystem s_pivot;
  private IntakeSubsystem s_intake;
  
  /** Creates a new MoveToReadySetCmd. */
  public MoveToGroundSetCmd(PivotSubsystem pivotsub, IntakeSubsystem intakesub) {
    this.s_pivot = pivotsub;
    this.s_intake = intakesub;
    
    addRequirements(pivotsub);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
     SmartDashboard.putString("Ground Finished", "Init");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    s_pivot.ToGround();
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
    error = PivotConstants.kGroundPoint - s_pivot.pivotencoder();
    if(s_intake.get_sensor() == false){
      return true;
    }

    else if(Math.abs( error ) <= 2){
      SmartDashboard.putString("Ground Finished", "True");
       return true; //finish when within three degree of ground
       
    }

     
    else{
      SmartDashboard.putString("Ground Finished", "false");
      return false;
    }
  }
}
