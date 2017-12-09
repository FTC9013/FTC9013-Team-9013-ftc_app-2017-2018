package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Glenn on 12/08/2017.
 */

public class Arm
{

    private DcMotor armMotor = null;
    private AnalogInput armSensor = null;


    public Arm(DcMotor aMotor, AnalogInput aSensor)
    {
        armMotor = aMotor;
        armSensor = aSensor;
    }

}