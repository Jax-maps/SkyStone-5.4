package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.WayPoint;

import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.MyPosition;

import java.util.ArrayList;
import java.util.List;

/*
For the enumerations
 */

public class HardwareOmnibot extends HardwareOmnibotDrive{
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        super.init(ahwMap);
    }

    public void resetEncoders()
    {
        int sleepTime = 0;
        int encoderCount = frontLeft.getCurrentPosition();

        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        while((encoderCount != 0) && (sleepTime < 1000)) {
            try {
                Thread.sleep(10);
            } catch (InterruptedException e) { break; }
            sleepTime += 10;
            resetReads();
            encoderCount = frontLeft.getCurrentPosition();
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
