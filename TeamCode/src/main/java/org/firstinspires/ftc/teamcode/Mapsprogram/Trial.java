package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.WayPoint;

@Autonomous(name="Trial")
public class Trial extends AutoTrialXY
{
    @Override
    public void setAutoWayPoints() {
        // Robot starting location
        //startLocation = new WayPoint(22.86, 203.7131, Math.toRadians(0.0), 0.0);
        startLocation = new WayPoint(0, 0, Math.toRadians(0.0), 0.0);

        // Locations to test
        //sampleLocation1 = new WayPoint(27.86, 32.86, Math.toRadians(0.0), 1.0);
        sampleLocation1 = new WayPoint(10, 0, Math.toRadians(0.0), 0.5);
    }
}

