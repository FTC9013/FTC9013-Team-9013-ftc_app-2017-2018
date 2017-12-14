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
  ElapsedTime flipLockoutArmDownTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private final double flipHeightLockOut = 0.4;

  
  private DcMotor armMotor = null;
  private AnalogInput armSensor = null;
  ArmPID armPID = null;
  private double height_;
  private double desiredHeight_ = 0;
  
  private double topLimitVoltage = 0;
  private double bottomLimitVoltage = 0;
  private double heightVoltage_;
  
  
  private final double initHighTime = 1.5;  // run motor up for n seconds.
  private final double initLowTime = 0.75;  // run motor down for n seconds.
  
  private final double flipHeight_ = 0.3;
  private final double minFlipHeight_ = 0.25;
  
  private final double armTrimTop_ = 0.1;    // keeps the arm from moving to the absolute end of range
  private final double armTrimBottom_ = 0.1;
  
  private final double heightUpStep_ = 0.01;
  private final double heightDownStep_ = 0.001;
  private final double heightTimerTick_ = 0.01;
  
  
  public Arm(DcMotor aMotor, AnalogInput aSensor)
  {
    armMotor = aMotor;
    armSensor = aSensor;
  
    // create and initialize the PID for the arm motor

    final double propCoeff = 0.25;
    final double integCoeff = 0.01;
    final double diffCoeff = 0.04;
  
    armPID = new ArmPID(propCoeff, integCoeff, diffCoeff);
    
    // initially setup the PID parameters
    armPID.setOutputLimits( -1, 1 );
    armPID.setMaxIOutput(0.1);
    armPID.setOutputRampRate(0.02);
    armPID.setOutputFilter(0.2);
    armPID.setSetpointRange(1);
  
    armPID.setSetpoint(0);
  }
  
  
  public  void setArmHeight (boolean up, boolean down)
  {
    // calculate the desired height based on the up down inputs
    if( up && ( desiredHeight_ < 1 ) && (heightStepTimer.time() > heightTimerTick_) )
    {
      desiredHeight_ += heightUpStep_ ;
      heightStepTimer.reset();
    }
    else if ( down
             && ( desiredHeight_ > 0) && (heightStepTimer.time() > heightTimerTick_)
             && flipLockoutArmDownTimer.time() > flipHeightLockOut )
    {
      desiredHeight_ -= heightDownStep_ ;
      heightStepTimer.reset();
    }
  
    heightVoltage_ = armSensor.getVoltage();
    // checking and scaling of inputs
    height_ = Range.scale(heightVoltage_, bottomLimitVoltage, topLimitVoltage, 0, 1);
    
    armMotor.setPower(armPID.getOutput(height_,desiredHeight_));
  }
  
  public boolean flipAllowed()
  {
    if(height_ < minFlipHeight_)
    {
      desiredHeight_ = flipHeight_;
      flipLockoutArmDownTimer.reset();
    }
    
    return height_ >= minFlipHeight_;
  }
  
  
  public  void calibrateArmHeight ()
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