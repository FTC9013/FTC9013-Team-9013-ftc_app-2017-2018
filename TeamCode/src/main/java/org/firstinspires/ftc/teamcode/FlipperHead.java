package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by Matrix on 12/08/2017.
 */

public class FlipperHead
{
  
  private Grabber topGrabber = null;
  private Grabber bottomGrabber = null;
  private Servo flipServo = null;
  private boolean flipState_ = false;

  public FlipperHead ( Grabber aGrabber, Grabber bGrabberl, Servo fServo )
  {
    topGrabber = aGrabber;
    bottomGrabber = bGrabberl;
    flipServo = fServo;
    flipState_ = false;
  }
  
  public void flipInit()
  {
    flipServo.setPosition( 0 );
    topGrabber.open();
    bottomGrabber.open();
  }
  
  
  public void flip()
  {
    if (flipState_)
    {
      flipServo.setPosition( 0 );
      flipState_ = false;
      topGrabber.swap( bottomGrabber );
    }
    else
    {
      flipServo.setPosition( 1 );
      flipState_ = true;
      topGrabber.swap( bottomGrabber );
    }
  }
  
  
  public void closeTop()
  {
    topGrabber.close();
  }
  
  public void closeBottom()
  {
    bottomGrabber.close();
  }
  
  
  public void openTop()
  {
    topGrabber.open();
  }
  
  public void openBottom()
  {
    bottomGrabber.open();
  }
  
    public void openBoth()
  {
    topGrabber.open();
    bottomGrabber.open();
  }
 
}
