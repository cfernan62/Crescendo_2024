// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/* 
Notes
One motor has to be inverted for pivot
*/
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PivotSubsystem extends SubsystemBase {
  CANSparkMax m_pivot1 = new CANSparkMax(PivotConstants.kPivotMotorCanId1, MotorType.kBrushless);
  CANSparkMax m_pivot2 = new CANSparkMax(PivotConstants.kPivotMotorCanId2, MotorType.kBrushless);
  DutyCycleEncoder m_encoder = new DutyCycleEncoder(PivotConstants.kEncoderPort);

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem() {
    //set to 0.5 units per rotation
  m_encoder.setDistancePerRotation(1);
  m_encoder.reset();
  //m_encoder.setPositionOffset(0.47);
  }

  @Override
  public void periodic() {
    double adjustedangle;
    final double ZERO_OFFSET = 0.0;
    adjustedangle = m_encoder.getAbsolutePosition() - ZERO_OFFSET;
    if(adjustedangle<0){
      adjustedangle= 1 + adjustedangle;
    }
    adjustedangle = adjustedangle * 360;
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
    SmartDashboard.putNumber("absolute pos", m_encoder.getAbsolutePosition());
    SmartDashboard.putNumber("encoder pos", adjustedangle);

   
    // This method will be called once per scheduler run
  }

  public void setmotor(double speed){
    double error;
    double output;
    if(speed < 0){// moving up
        if(pivotencoder() < 165){ // not to far back
        speed = 0;
      }
    }

    else if(speed >= 0){
      if(pivotencoder() > PivotConstants.kGroundPoint){//not to low
        error = PivotConstants.kGroundPoint - pivotencoder();
        output = error * 0.1;
        if(output > 0.5){
          output = 0.5;
        }
        else if(output < -0.5){
          output = -0.5;
        }
        speed = output;
        SmartDashboard.putString("pivot mode", "is true");
      }
    }
    
    m_pivot1.set(speed);
    m_pivot2.set(-speed);
    SmartDashboard.putNumber("pivspeed", speed);
  }
  

  public void resetpivotencoder(){
    m_encoder.reset();
  }

  public double pivotencoder(){
    double adjustedangle;
    final double ZERO_OFFSET = 0.0;
    adjustedangle = m_encoder.getAbsolutePosition() - ZERO_OFFSET;
    if(adjustedangle<0){
      adjustedangle= 1 + adjustedangle;
    }
    adjustedangle = adjustedangle * 360;
    return adjustedangle;
  }

  public void ToDrive(){
    double p_error; //position error
    double output;
    final double PVALUE = 0.03; //lazy pid :)
    p_error = PivotConstants.kDrivePoint - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.5){ //positive moves pivot up
      output = 0.5; 
    }
    else if(output < -0.5){// negative moves pivot down
      output = -0.5;
    }
    
    setmotor(output);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Error", p_error);
  }

  public void ToGround(){

    double p_error; //position error
    double output;
    final double PVALUE = 0.01; //lazy pid :)
    p_error = PivotConstants.kGroundPoint - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.5){ //positive moves pivot up
      output = 0.5; 
    }
    else if(output < -0.5){// negative moves pivot down
      output = -0.5;
    }

    if(pivotencoder() > PivotConstants.kGroundPoint){
      SmartDashboard.putString("test", "im here");

      output = 0;
    }
    else SmartDashboard.putString("test", "im not here");
    
    setmotor(output);
    SmartDashboard.putNumber("Output", output);
    SmartDashboard.putNumber("Error", p_error);
   /*  if (pivotencoder() > PivotConstants.kGroundPoint + 5){
      setmotor(-.4);
    }
    else{
      if (pivotencoder() < PivotConstants.kGroundPoint - 1){
        setmotor(0.4);
      }
    }*/
  }

  public void AutoToGround(){

    double p_error; //position error
    double output;
    final double PVALUE = 0.06; //lazy pid :)
    p_error = PivotConstants.kGroundPoint - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.5){ //positive moves pivot up
      output = 0.5; 
    }
    else if(output < -0.5){// negative moves pivot down
      output = -0.5;
    }

    if(pivotencoder() > PivotConstants.kGroundPoint){
     

      output = 0;
    }
   
    
    setmotor(output);
  }

  public void ToIdleState(){
    if (pivotencoder() < PivotConstants.kStartPoint - 5){
      setmotor(0.3);
    }
    else{
      if (pivotencoder() > PivotConstants.kStartPoint + 5){
        setmotor(-0.3);
      }
    }
  }

  public void ToScore(){
    double p_error; //position error
    double output;
    final double PVALUE = 0.05; //lazy pid :)
    p_error = PivotConstants.kShootingPoint - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.6){ //positive moves pivot up
      output = 0.6; 
    }
    else if(output < -0.6){// negative moves pivot down
      output = -0.6;
    }
    SmartDashboard.putNumber("To Score Output", output);
    setmotor(output);
   /*  if (pivotencoder() > PivotConstants.kShootingPoint + 3){
      setmotor(-0.25);//moves arm up
    }
    else{
      if (pivotencoder() < PivotConstants.kShootingPoint - 1){
        setmotor(0.5);//moves arm down
      }
    }*/
  }

  public void AutoToScore(){
    double p_error; //position error
    double output;
    final double PVALUE = 0.04; //lazy pid :)
    p_error = PivotConstants.kShootingPoint - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.7){ //positive moves pivot up
      output = 0.7; 
    }
    else if(output < -0.7){// negative moves pivot down
      output = -0.7;
    }
   
    setmotor(output);
  }

  public void ToAmp(){
    double p_error; //position error
    double output;
    final double PVALUE = 0.03; //lazy pid :)
    p_error = PivotConstants.kAmpPoint - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.5){ //positive moves pivot up
      output = 0.5; 
    }
    else if(output < -0.5){// negative moves pivot down
      output = -0.5;
    }
    SmartDashboard.putNumber("To Score Output", output);
    setmotor(output);
  }
  public void ToIntakeHelper(){
    double p_error; //position error
    double output;
    final double PVALUE = 0.04; //lazy pid :)
    p_error = PivotConstants.kIntakeHelper - pivotencoder();
    output = p_error * PVALUE;
    if(output > 0.7){ //positive moves pivot up
      output = 0.7; 
    }
    else if(output < -0.7){// negative moves pivot down
      output = -0.7;
    }
    
    setmotor(output);
   }
  }

