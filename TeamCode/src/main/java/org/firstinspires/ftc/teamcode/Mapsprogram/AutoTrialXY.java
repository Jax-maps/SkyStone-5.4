package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.WayPoint;
import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.MyPosition;

public abstract class AutoTrialXY extends AutoTrialBase{
    // Coordinates for the vision pipeline to be overriden in the alliance classes.
    public static int position = 0;

    public abstract void setAutoWayPoints();

    @Override
    public void runOpMode() {
        telemetry.addLine("Calling robot.init");
        updateTelemetry(telemetry);
        setupRobotParameters(4.0, 19.2);

        telemetry.addLine("Ready");
        updateTelemetry(telemetry);
        robot.encodersReset = true;

        setAutoWayPoints();

        /*
         * Wait for the user to press start on the Driver Station
         */
        waitForStart();
        // This is overall auto timer so we can keep track of events with other bot.
        autoTimer.reset();

        //give MyPosition our current positions so that it saves the last positions of the wheels
        //this means we won't teleport when we start the match. Just in case, run this twice
        for (int i = 0; i < 2; i++) {
            robot.resetReads();
            MyPosition.initialize(robot.getLeftEncoderWheelPosition(),
                    robot.getRightEncoderWheelPosition(),
                    robot.getStrafeEncoderWheelPosition());
        }

        // Set our robot starting coordinates on the field.
        robot.resetReads();
        MyPosition.setPosition(startLocation.x, startLocation.y, startLocation.angle);

        driveToWayPoint(sampleLocation1, false, false);

        telemetry.addData("Moved to Position 1:", "happy");
        telemetry.update();


    }
}
