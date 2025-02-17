// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimbSubsystem extends SubsystemBase {
  private CANSparkMax m_climber;
    
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    m_climber = new CANSparkMax(ClimberConstants.kClimberMotorCanId, MotorType.kBrushless);   
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setmotor(double speed){
    m_climber.set(-speed * 0.5);
  }
}
