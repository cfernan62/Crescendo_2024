// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoScoreAim extends Command {
  PivotSubsystem s_pivot;
  VisionSubsystem s_vision;
  /** Creates a new AutoScoreAim. */
  public AutoScoreAim(PivotSubsystem pivot, VisionSubsystem vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_pivot = pivot;
    s_vision = vision;
    addRequirements(pivot, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double shooterangle;
    double pivotangle;

    pivotangle = 180 - (s_pivot.pivotencoder() * 88.14); //ADD WHATEVER WE NEED TO CONVERT ENCODER TO ACCURATE ANGLE HERE

    shooterangle = Math.atan(83 / s_vision.GetSpeakerDistance());

    double p_error; //position error
    double output;
    final double PVALUE = 0.05; //lazy pid :)
    p_error = pivotangle - shooterangle;
    output = p_error * PVALUE;
    if(output > 0.5){ //positive moves pivot down
      output = 0.5; 
    }
    else if(output < -0.5){// negative moves pivot up
      output = -0.5;
    }
    
    s_pivot.setmotor(output);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_pivot.setmotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_pivot.pivotencoder() < Math.atan(83 / s_vision.GetSpeakerDistance()) + 2 && s_pivot.pivotencoder() > Math.atan(83 / s_vision.GetSpeakerDistance()) - 2){
      return true;
    }
    else{
      return false;
    }
  }
}
