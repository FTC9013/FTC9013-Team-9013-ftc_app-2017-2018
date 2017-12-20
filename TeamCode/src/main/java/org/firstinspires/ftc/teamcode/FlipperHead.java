package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Matrix on 12/08/2017.
 */

public class FlipperHead
{
  private Grabber topGrabber = null;
  private Grabber bottomGrabber = null;
  private Servo flipServo = null;
  private Arm arm = null;
  private boolean flipState_ = false;
  private boolean partialGrabState_ = false;
  
  ElapsedTime partialGrabTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private final double partialGrabHoldOffTime = 0.5;
  
  ElapsedTime flipTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private final double flipHoldOffTime = 2.0;
  
  ElapsedTime topTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private final double topHoldOffTime = 0.5;
  
  ElapsedTime bottomTimer = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
  private final double bottomHoldOffTime = 0.5;
  
  
  public FlipperHead ( Grabber aGrabber, Grabber bGrabberl, Servo fServo, Arm armIn )
  {
    topGrabber = aGrabber;
    bottomGrabber = bGrabberl;
    flipServo = fServo;
    arm = armIn;
    flipState_ = false;
  }
  
  public void flipInit()
  {
    flipServo.setPosition( 0 );
    topGrabber.Open();
    bottomGrabber.Open();
  }
  
  public void partOpen ()
  {
    
    if ( partialGrabState_ && partialGrabAllowed() )
    {
      partialGrabState_ = false;
      topGrabber.PartOpen(false);
      bottomGrabber.PartOpen(false);
      partialGrabTimer.reset();
    }
    else if ( partialGrabAllowed() )
    {
      partialGrabState_ = true;
      topGrabber.PartOpen(true);
      bottomGrabber.PartOpen(true);
      partialGrabTimer.reset();
    }
    
  }
  
  public void flip()
  {
    flipTimer.reset();
    
    if (flipState_ && arm.flipAllowed() )
    {
      flipServo.setPosition( 0 );
      flipState_ = false;
      topGrabber.swap( bottomGrabber );
    }
    else if ( arm.flipAllowed() )
    {
      flipServo.setPosition( 1 );
      flipState_ = true;
      topGrabber.swap( bottomGrabber );
    }
  }
  
  public boolean partialGrabAllowed()
{
  return ( partialGrabTimer.time() > partialGrabHoldOffTime );
}
  
  // prevents flipping too soon after the previous flip
  public boolean flipAllowed()
  {
    return ( flipTimer.time() > flipHoldOffTime );
  }
  
  // prevents grabber usage too soon after the previous use
  public boolean topAllowed()
  {
    return ( topTimer.time() > topHoldOffTime );
  }
  
  public boolean bottomAllowed()
  {
    return bottomTimer.time() > bottomHoldOffTime;
  }
  
  public void closeTop()
  {
    topTimer.reset();
    topGrabber.close();
  }
  
  public void closeBottom()
  {
    bottomTimer.reset();
    bottomGrabber.close();
  }
  
  
  public void OpenTop()
  {
    topTimer.reset();
    topGrabber.Open();
  }
  
  public void OpenBottom()
  {
    bottomTimer.reset();
    bottomGrabber.Open();
  }
  
    public void openBoth()
  {
    topTimer.reset();
    bottomTimer.reset();
    
    topGrabber.Open();
    bottomGrabber.Open();
  }
 
}
