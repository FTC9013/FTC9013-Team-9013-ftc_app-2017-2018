package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import java.text.DecimalFormat;

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
  public double desiredHeight = 0;
  
  private double topLimitVoltage = 0;
  private double bottomLimitVoltage = 0;
  private double heightVoltage_;
  
  
  private final double initHighTime = 2.5;  // run motor up for n seconds.
  private final double initLowTime = 2.5;  // run motor down for n seconds.
  
  private final double flipHeight_ = 0.3;
  private final double minFlipHeight_ = 0.25;
  
  private final double armTrimTop_ = 0.025;    // keeps the arm from moving to the absolute end of range
  private final double armTrimBottom_ = 0.05;
  
  private final double heightUpStep_ = 0.015;
  private final double heightDownStep_ = 0.0125;
  private final double heightTimerTick_ = 0.009;
  
  
  public Arm(DcMotor aMotor, AnalogInput aSensor)
  {
    armMotor = aMotor;
    armSensor = aSensor;
  
    // create and initialize the PID for the arm motor

    final double propCoeff = 0.3;    // original value .25
    final double integCoeff = 0.01;  // original value .01
    final double diffCoeff = 0.05;   // original value .04
  
    armPID = new ArmPID(propCoeff, integCoeff, diffCoeff);
    
    // initially setup the PID parameters
    armPID.setOutputLimits( -0.01, 0.3 );
    armPID.setMaxIOutput(0.2);
    armPID.setOutputRampRate(0.1);
    armPID.setOutputFilter(0.1);
    armPID.setSetpointRange(1);
  
    armPID.setSetpoint(0);
  }
  
  public  void setArmHeight (boolean up, boolean down)
  {
    // calculate the desired height based on the up down inputs
    if( up && ( desiredHeight < 1 ) && (heightStepTimer.time() > heightTimerTick_) )
    {
      desiredHeight += heightUpStep_ ;
      heightStepTimer.reset();
    }
    else if ( down
             && ( desiredHeight > 0) && (heightStepTimer.time() > heightTimerTick_)
             && flipLockoutArmDownTimer.time() > flipHeightLockOut )
    {
      desiredHeight -= heightDownStep_ ;
      heightStepTimer.reset();
    }
  
    heightVoltage_ = armSensor.getVoltage();
    // checking and scaling of inputs
    height_ = Range.scale(heightVoltage_, bottomLimitVoltage, topLimitVoltage, 0, 1);
    
    armMotor.setPower(armPID.getOutput(height_,desiredHeight));
  }
  
  public boolean flipAllowed()
  {
    if(height_ < minFlipHeight_)
    {
      desiredHeight = flipHeight_;
      flipLockoutArmDownTimer.reset();
    }
    
    return height_ >= minFlipHeight_;
  }
  
  
  public  void calibrateArmHeight ()
  {
    ElapsedTime initTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
    desiredHeight = 0 ;
    initTimer.reset();
    
    
    // calibrate the upper limit
    armMotor.setPower(0.25);
    while (initTimer.time() < initHighTime )
    {
      topLimitVoltage = armSensor.getVoltage();
    }

    initTimer.reset();
    // calibrate the lower limit
    armMotor.setPower(-0.01);
    while (initTimer.time() < initLowTime )
    {
      bottomLimitVoltage = armSensor.getVoltage();
    }
    
    // apply the trim factors
    topLimitVoltage -= armTrimTop_;
    bottomLimitVoltage += armTrimBottom_;
  }
}