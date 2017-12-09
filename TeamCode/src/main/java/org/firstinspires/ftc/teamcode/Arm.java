package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by Glenn on 12/08/2017.
 */

public class Arm
{
  
  ElapsedTime heightStepTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  
  private DcMotor armMotor = null;
  private AnalogInput armSensor = null;
  private double height_;
  private double desiredHeight_ = 0;
  
  private double topLimitVoltage = 0;
  private double bottomLimitVoltage = 0;
  private double heightVoltage_;
  
  
  private final double initHighTime = 2.5;  // run motor up for n seconds.
  private final double initLowTime = 1.5;  // run motor down for n seconds.
  
  private final double flipHeight_ = 0.1;
  private final double minFlipHeight_ = 0.25;
  
  private final double armTrimTop_ = 0.02;    // keeps the arm from moving to the absolute end of range
  private final double armTrimBottom_ = 0.03;
  
  private final double heightStep_ = 0.01;
  private final double heightTimerTick_ = 0.01;
  
  
  public Arm(DcMotor aMotor, AnalogInput aSensor)
  {
    armMotor = aMotor;
    armSensor = aSensor;
  }

  
  public  void setArmHeight (boolean up, boolean down)
  {
    // calculate the desired height based on the up down inputs
    if( up && ( desiredHeight_ < 1 ) && (heightStepTimer.time() > heightTimerTick_) )
    {
      desiredHeight_ += heightStep_ ;
      heightStepTimer.reset();
    }
    else if ( down && ( desiredHeight_ > 0) && (heightStepTimer.time() > heightTimerTick_) )
    {
      desiredHeight_ -= heightStep_ ;
      heightStepTimer.reset();
    }
  
    heightVoltage_ = armSensor.getVoltage();
    // checking and scaling of inputs
    height_ = Range.scale(heightVoltage_, bottomLimitVoltage, topLimitVoltage, 0, 1);
    desiredHeight_ = Range.clip( desiredHeight_, 0, 1);
    
    
    // crude control of motor power based on height and desired height
    if( height_ < desiredHeight_ )
    {
      armMotor.setPower(0.4);  // arm goes up
    }
    else if (height_ > desiredHeight_ )
    {
      armMotor.setPower(-0.1);  // arm goes down
    }
    else
    {
      armMotor.setPower(0);
    }
  }
  
  public boolean flipAllowed()
  {
    desiredHeight_ = flipHeight_;
    return height_ >= minFlipHeight_;
  }
  
  
  public  void initArmHeight ()
  {
    ElapsedTime initTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    desiredHeight_ = 0 ;
    initTimer.reset();
    
    
    // calibrate the upper limit
    armMotor.setPower(0.3);
    while (initTimer.time() < initHighTime )
    {
      topLimitVoltage = armSensor.getVoltage();
    }

    initTimer.reset();
    // calibrate the lower limit
    armMotor.setPower(-0.1);
    while (initTimer.time() < initLowTime )
    {
      bottomLimitVoltage = armSensor.getVoltage();
    }
    
    // apply the trim factors
    topLimitVoltage -= armTrimTop_;
    bottomLimitVoltage += armTrimBottom_;
  }
}