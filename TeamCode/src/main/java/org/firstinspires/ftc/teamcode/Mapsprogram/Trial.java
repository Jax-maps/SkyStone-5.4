package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.WayPoint;

@Autonomous(name="Trial")
public class Trial extends AutoTrialXY
{
    @Override
    public void setAutoWayPoints() {
        // Robot starting location
        startLocation = new WayPoint(22.86, 203.7131, Math.toRadians(0.0), 0.0);

        // Location to test
        sampleLocation1 = new WayPoint(27.86, 32.86, Math.toRadians(0.0), 1.0);
        sampleLocation1 = new WayPoint(27.86, 235.9901, Math.toRadians(0.0), 1.0);
        sampleLocation1 = new WayPoint(27.86, 32.86, Math.toRadians(90.0), 1.0);
    }
}
