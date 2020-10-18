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

}
