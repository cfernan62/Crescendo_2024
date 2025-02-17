// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class ShooterSubsystem extends SubsystemBase {
  CANSparkMax m_shooter1 = new CANSparkMax(ShooterConstants.kShooterMotorCanId1, MotorType.kBrushless);
  CANSparkMax m_shooter2 = new CANSparkMax(ShooterConstants.kShooterMotorCanId2, MotorType.kBrushless);
  private final RelativeEncoder m_shooterEncoder = m_shooter1.getEncoder();
    
  
  /** Creates a new ShooterSubsystem. */
  public ShooterSubsystem() {
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Shooter Velocity", m_shooterEncoder.getVelocity());
  }

  public void setmotor(double speed){
    m_shooter1.set(-speed);
    m_shooter2.set(-speed);
  }
  public void setlowmotor(double speed){
    m_shooter1.set(speed);//lower motor for intake
  }

  public double get_velocity(){
    return Math.abs(m_shooterEncoder.getVelocity());
  }
}
