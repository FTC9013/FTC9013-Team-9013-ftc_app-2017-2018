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

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;


@Autonomous(name="AutonomousRed", group="Linear Opmode")

public class MIM_Autonomous_Red extends LinearOpMode
{

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    VuforiaLocalizer vuforia;

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
        leftGrab = hardwareMap.servo.get("leftGrab");
        rightGrab = hardwareMap.servo.get("rightGrab");

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
        jewelServo.setPosition(1);

        leftGrab.setPosition(0);
        rightGrab.setPosition(1);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "ASPJ+XX/////AAAAGVoUw33gakWRlcLvnWu+vK4/0BincNFIHCnW6Vxig4SR4t7P9G9oRc/LemAnahIp4wXtyWHVoSaLI/7EIpxN/3Tq08Pgs6zsO9pxNukXh8cm7bWcVJ3/RCqAyZReB8kD2duQoOqUlfG9vjVzcLcdu+SFkQP0MkHR8jviuZX30Rp2TWCAtmR/ecjzb6uHPRvkw4IfpWhppDv0TVkCaCvkeMNXI+cR50ythnyVKtZRRs9f0tb3abDhQ1RaFqF9ljyTyzUIgd6M+LnF3fEPJXYvgJSYk4jYyzFf5ATDK93sv8Iem9jNN6zcmKydWFET/vBWhyRLmaU7EkFdseNeWgW5Hgc+G33OeYdPbHy7DKVUB/vg";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive())
        {
            double leftRamp = Math.pow(Math.abs(leftStick1), 2.5);
            double rightRamp = Math.pow(Math.abs(rightStick1), 2.5);

            //Clips the left or right drive powers to 1 if it is > 1 and to -1 if it is < -1 (sets the values to between 1 and -1)
            leftPower = Range.clip( leftRamp, -1.0, 1.0 );
            rightPower = Range.clip( rightRamp, -1.0, 1.0 );



            // Send calculated power to wheels
            frontLeftDrive.setPower(leftPower);
            rearLeftDrive.setPower(leftPower);

            frontRightDrive.setPower(rightPower);
            rearRightDrive.setPower(rightPower);

            runtime.reset();


            if (jewelColor.blue() > 20 && (runtime.seconds() < 1.0))
            {
                rightPower=-0.5;
                leftPower=-0.5;
            }

            if (jewelColor.red() > 20 && (runtime.seconds() < 1.0))
            {
                rightPower=0.5;
                leftPower=0.5;
            }

            if ( runtime.seconds() > 1.5 && jewelColor.red() < 20 && jewelColor.blue() < 20)
            {
                jewelServo.setPosition(1);
            }
            else
            {
                jewelServo.setPosition(0.5);
            }

            if ( runtime.seconds() > 2.0 );
            {
                RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
                if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                    telemetry.addData("VuMark", "%s visible", vuMark);

                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)relicTemplate.getListener()).getPose();
                    telemetry.addData("Pose", format(pose));
                    //if (pose = )
                    if (pose != null) {
                        VectorF trans = pose.getTranslation();
                        Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                        // Extract the X, Y, and Z components of the offset of the target relative to the robot
                        double tX = trans.get(0);
                        double tY = trans.get(1);
                        double tZ = trans.get(2);

                        // Extract the rotational components of the target relative to the robot
                        double rX = rot.firstAngle;
                        double rY = rot.secondAngle;
                        double rZ = rot.thirdAngle;
                    }
                }
                else {
                    telemetry.addData("VuMark", "not visible");
                }

                telemetry.update();
            }




            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Color", "Blue: (%d), Red: (%d),Green: (%d)",
                    jewelColor.blue(),jewelColor.red(),jewelColor.green());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
