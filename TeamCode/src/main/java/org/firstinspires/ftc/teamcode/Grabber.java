package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Matrix on 12/07/2017.
 *
 * This grabber class hides the individual servos and provides a way to swap one
 * grabber ser with the other so that the control input can always address the Top or Bottom
 * grabber regardless of the orientation of the head.
 *
 */

public class Grabber
{
  private  Servo servo1 = null;
  private  Servo servo2 = null;
  private  boolean isOpen_ = true;// initialize to open
  double openA = 1;
  double openB = 0;
  
  
  

  Grabber( Servo aServo, Servo bServo )
  {
    this.servo1 = aServo;
    this.servo2 = bServo;
    Open();
  }


  public void Open()
  {
    // set the servos to the open jaw state
    servo1.setPosition( openA );
    servo2.setPosition( openB );
    isOpen_= true;
  }
  
  public void PartOpen( boolean mode )
  {
    if(mode)
    {
      openA = 0.5;
      openB = 0.5;
    }
    else
    {
      openA = 1;
      openB = 0;
    }
  }
  
  public void close()
  {
    // set the servos to the closed jaw state
    servo1.setPosition( 0 );
    servo2.setPosition( 1 );
    isOpen_= false;
  }
  
  public boolean isOpen_()
  {
    return this.isOpen_;
  }


  public void swap( Grabber grabber )
  {
    Servo tempServo1 = servo1;
    Servo tempServo2 = servo2;
    boolean tempState = isOpen_();

    // swap the servos with the grabber passed in
    servo1 = grabber.servo1;
    servo2 = grabber.servo2;
    
    if ( !grabber.isOpen_() )
    {
      close();
    }
    else
    {
      Open();
    }
    
    grabber.servo1 = tempServo1;
    grabber.servo2 = tempServo2;
    if ( !tempState )
    {
      grabber.close();
    }
    else
    {
      grabber.Open();
    }
  }
}
