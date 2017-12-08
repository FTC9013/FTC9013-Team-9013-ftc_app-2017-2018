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
  private  boolean isOpen_ = true;    // initialize to open

  Grabber( Servo aServo, Servo bServo )
  {
    this.servo1 = aServo;
    this.servo2 = bServo;
    open();
  }


  public void open()
  {
    // set the servos to the open jaw state
    servo1.setPosition( 1 );
    servo2.setPosition( 1 );
    isOpen_= true;
  }


  public void close()
  {
    // set the servos to the closed jaw state
    servo1.setPosition( 0 );
    servo2.setPosition( 0 );
    isOpen_= false;
  }


  public boolean isOpen()
  {
    return this.isOpen_;
  }


  public void swap( Grabber grabber )
  {
    Servo tempServo1 = servo1;
    Servo tempServo2 = servo2;

    // swap the servos with the grabber passed in
    servo1 = grabber.servo1;
    servo2 = grabber.servo2;

    grabber.servo1 = tempServo1;
    grabber.servo2 = tempServo2;
  }
}
