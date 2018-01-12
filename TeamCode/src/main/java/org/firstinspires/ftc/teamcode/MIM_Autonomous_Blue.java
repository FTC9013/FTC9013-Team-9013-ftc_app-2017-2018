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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;



@Autonomous(name="AutonomousBlue", group="Linear Opmode")

public class MIM_Autonomous_Blue extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode()
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        ColorSensor jewelColor;



        DcMotor frontLeftDrive;
        DcMotor frontRightDrive;
        DcMotor rearLeftDrive;
        DcMotor rearRightDrive;
        DcMotor armDrive;

        Servo jewelServo;
        Servo leftGrab;
        Servo rightGrab;

        double armUp = 0;
        double armDown = 0.65;
        
        boolean driveBlue = false;
        boolean driveRed = false;

        // Setup a variable for each drive wheel to save power level for telemetry
        double leftPower = 0;
        double rightPower = 0;

        double leftStick1 = 0;
        double rightStick1 = 0;

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        frontLeftDrive  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRightDrive = hardwareMap.get(DcMotor.class, "frontRight");
        rearLeftDrive = hardwareMap.get (DcMotor.class, "rearLeft");
        rearRightDrive = hardwareMap.get (DcMotor.class, "rearRight");
        armDrive = hardwareMap.get (DcMotor.class, "armDrive");

        jewelServo = hardwareMap.servo.get("jewelServo");


        jewelColor = hardwareMap.colorSensor.get("jewelColor");
        jewelColor.enableLed(true);


        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        rearLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        rearRightDrive.setDirection(DcMotor.Direction.REVERSE);
        armDrive.setDirection(DcMotor.Direction.REVERSE);

        // Start position of the jewel bumping arm (up)
        jewelServo.setPosition(armUp);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        
        runtime.reset();
        
        jewelServo.setPosition(armDown);
        // wait a couple seconds for the arm to lower and steady.
        while(runtime.seconds() < 2.0){}
  
        runtime.reset();
        
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {

            // Send calculated power to wheels
            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);

            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);

            // analyze color for 1 second
            if(runtime.seconds() < 1.0 && !(driveBlue || driveRed))
            {
              //See Color,
              if (jewelColor.blue() > 20)
              {
                driveBlue = true;
              }
              //See Color,
              if (jewelColor.red() > 20)
              {
                driveRed = true;
              }
            }
            else if (runtime.seconds() > 1.0 && runtime.seconds() < 2.0 )
            {
              if ( driveBlue )
              {
                rightPower=0.15;
                leftPower=0.15;
              }
              else if ( driveRed )
              {
                rightPower=-0.15;
                leftPower=-0.15;
              }
            }
            else
            {
              rightPower = 0;
              leftPower = 0;
              jewelServo.setPosition(armUp);
            }
          
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Color", "Blue: (%d), Red: (%d),Green: (%d)",
                    jewelColor.blue(),jewelColor.red(),jewelColor.green());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}




