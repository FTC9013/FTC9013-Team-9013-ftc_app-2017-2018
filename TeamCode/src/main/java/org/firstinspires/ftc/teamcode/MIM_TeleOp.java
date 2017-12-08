/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.AnalogSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@TeleOp(name="TeleOp", group="Linear Opmode")

public class MIM_TeleOp extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // this sensor looks at the jewel and reports the color as 3 RGB integer values
        ColorSensor jewelColor;
        
        // this it the sensor that measures the height of the main arm.
        AnalogInput armPosition;
        
        

        DcMotor frontLeftDrive;
        DcMotor frontRightDrive;
        DcMotor rearLeftDrive;
        DcMotor rearRightDrive;
        DcMotor armDrive;

        Servo jewelServo;
        Servo leftGrabA;
        Servo rightGrabA;
        Servo leftGrabB;
        Servo rightGrabB;
        Servo flipServo;
        Servo tempServo1;
        Servo tempServo2;

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0;
        double rightPower = 0;

        double leftStick1 = 0;
        double rightStick1 = 0;
        
        double grabAButtonTime = 0;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeftDrive = hardwareMap.get (DcMotor.class, "rearLeft");
        rearRightDrive = hardwareMap.get (DcMotor.class, "rearRight");
        armDrive = hardwareMap.get (DcMotor.class, "armDrive");

        jewelServo = hardwareMap.servo.get("jewelServo");
        leftGrabA = hardwareMap.servo.get("leftGrabA");
        rightGrabA = hardwareMap.servo.get("rightGrabA");
        leftGrabB = hardwareMap.servo.get("leftGrabB");
        rightGrabB = hardwareMap.servo.get("rightGrabB");
        flipServo = hardwareMap.servo.get("flipServo");

        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        jewelColor.enableLed(true);
        
        armPosition = hardwareMap.analogInput.get("armPos");


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        // Start position of the jewel bumping arm (up)
        jewelServo.setPosition(1);

        leftGrabA.setPosition(1);
        rightGrabA.setPosition(0);
        leftGrabB.setPosition(0);
        rightGrabB.setPosition(1);
        flipServo.setPosition(0);

        armDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
  
      boolean flipReleaseFlag = false; //True when the flipper button is being pressed
      boolean flipFlag = false; //True when the flipper is flipped
      boolean aGrabFlag = false; //True when the A servos are closed
      boolean aGrabReleaseFlag = false; //True when the A Grab button is being pressed
      boolean bGrabFlag = false; //True when the B servos are closed
      boolean bGrabReleaseFlag = false; //True when the B Grab button is being pressed
      boolean flipLock = false; //True when the flipper is locked
      boolean aGrabLock = false; //True when the A Grabbers are locked
      boolean bGrabLock = false; //True when the B Grabbers are locked
      boolean aUpFlag = false; //True when A grabber is on top
  
      // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            leftStick1 =  gamepad1.left_stick_y;
            rightStick1 = gamepad1.right_stick_y;
    
            double leftRamp = Math.pow(Math.abs(leftStick1), 2.5);
            double rightRamp = Math.pow(Math.abs(rightStick1), 2.5);
    
            if( gamepad1.left_stick_y < 0 )
            {
              leftRamp = -leftRamp;
            }
    
            if( gamepad1.right_stick_y < 0 )
            {
              rightRamp = -rightRamp;
            }
    
            //When the dpad is pressed up or down and the flag is false then the arm moves up or down as arm runtime is tracked,
            // otherwise it is stationary and and arm runtime is set to zero and arm off time begins tracking
            if(gamepad2.dpad_up)
            {
              armDrive.setPower(0.4);
              //armTime = time;
            }
            else if (gamepad2.dpad_down)
            {
              armDrive.setPower(-0.1);
      
            }
            else
            {
              armDrive.setPower(0);
            }
    
    
            // When Y is pressed, the flipper servo is below position one, Y has been released from the last press, and the flipper is unlocked,
            // set the position to 1 and wait trigger the flipper button release flag (wait for the button to be released)
            if (gamepad2.y && flipFlag == false && flipReleaseFlag == false && flipLock == false)
            {
              flipFlag = true;
              flipReleaseFlag = true;
            }
            // When Y is pressed, the flipper servo is in position one, Y has been released from the last press, and the flipper is unlocked,
            // set the position to 0 and wait trigger the flipper button release flag (wait for the button to be released)
            else if (gamepad2.y && flipFlag == true && flipReleaseFlag == false && flipLock == false)
            {
              flipFlag = false;
              flipReleaseFlag = true;
            }
            // When Y is released, set the release flag to false, allowing the other flipper commands to take place
            if (!gamepad2.y)
            {
              flipReleaseFlag = false;
            }
    
    
            // When the left bumper is pressed, the aGrabFlag is false, the bumper has been released from the last press, and the grabber is unlocked,
            //set the aGrabFlag to true (which closes bottom the grabbers) and trigger the release flag (wait for the button to be released)
            if (gamepad2.left_bumper && aGrabFlag == false && aGrabReleaseFlag == false && aGrabLock == false)
            {
              aGrabFlag = true;
              aGrabReleaseFlag = true;
            }
            // When the left bumper is pressed, the aGrabFlag is true, the bumper has been released from the last press, and the grabber is unlocked,
            //set the aGrabFlag to false (which opens bottom the grabbers) and trigger the release flag (wait for the button to be released)
            else if (gamepad2.left_bumper && aGrabFlag == true && aGrabReleaseFlag == false && aGrabLock == false)
            {
              aGrabFlag = false;
              aGrabReleaseFlag = true;
            }
            // When the left bumper is released, set the release flag to false, allowing the other A Grabber commands to take place
            if (!gamepad2.left_bumper && aGrabReleaseFlag == true)
            {
              aGrabReleaseFlag = false;
            }
    
            // When the right bumper is pressed, the bGrabFlag is false, the bumper has been released from the last press, and the grabber is unlocked,
            //set the bGrabFlag to true (which closes the top grabbers) and trigger the release flag (wait for the button to be released)
            if (gamepad2.right_bumper && bGrabFlag == false && bGrabReleaseFlag == false && bGrabLock == false)
            {
              bGrabFlag = true;
              bGrabReleaseFlag = true;
            }
            // When the left bumper is pressed, the bGrabFlag is true, the bumper has been released from the last press, and the grabber is unlocked,
            //set the bGrabFlag to false (which opens bottom the grabbers) and trigger the release flag (wait for the button to be released)
            else if (gamepad2.right_bumper && bGrabFlag == true && bGrabReleaseFlag == false && bGrabLock == false)
            {
              bGrabFlag = false;
              bGrabReleaseFlag = true;
            }
            // When the left bumper is released, set the release flag to false, allowing the other B Grabber commands to take place
            if (!gamepad2.right_bumper && bGrabReleaseFlag == true)
            {
              bGrabReleaseFlag = false;
            }
    
    
            //Clips the left or right drive powers to 1 if it is > 1 and to -1 if it is < -1
            // (sets the values to between 1 and -1)
            leftPower = Range.clip( leftRamp, -1.0, 1.0 );
            rightPower = Range.clip( rightRamp, -1.0, 1.0 );
    
    
            // Send calculated power to wheels
            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);
    
            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);
    
            //Close the bottom grabbers when the aGrabFlag is set true by the left bumper
            if (aGrabFlag == true)
            {
              leftGrabA.setPosition(0);
              rightGrabA.setPosition(1);
            }
            //Open the bottom grabbers when the aGrabFlag is set false by the left bumper
            else if (aGrabFlag == false)
            {
              leftGrabA.setPosition(1);
              rightGrabA.setPosition(0);
            }
    
            //Close the top grabbers when the bGrabFlag is set true by the right bumper
            if (bGrabFlag == true)
            {
              leftGrabB.setPosition(1);
              rightGrabB.setPosition(0);
            }
            //Open the top grabbers when the bGrabFlag is set true by the right bumper
            else if (bGrabFlag == false)
            {
              leftGrabB.setPosition(0);
              rightGrabB.setPosition(1);
            }
  
          //Close the top grabbers when the flipFlag is set true by Y
          if (flipFlag == true)
          {
            flipServo.setPosition(1);
          }
          //Open the top grabbers when the flipFlag is set true by the Y
          else if (flipFlag == false)
          {
            flipServo.setPosition(0);
          }
  
          // Show the elapsed game time and wheel power.
            telemetry.addData("Arm Position","(%.2f)",armPosition.getVoltage() );
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}

