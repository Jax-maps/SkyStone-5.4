/*
 * Copyright (c) 2019 OpenFTC Team
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.WayPoint;
import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.MyPosition;
import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.MovementVars;

/**
 * Created by MAPS
 */
public abstract class AutoTrialBase extends LinearOpMode
{
    protected ElapsedTime timer;

    public static float mmPerInch = AutoTrialBase.MM_PER_INCH;
    public static float mmBotWidth = 18 * mmPerInch;            // ... or whatever is right for your robot
    public static float mmFTCFieldWidth = (12 * 12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
    protected boolean skipThis = false;
    protected boolean integrated = false;

    //Starting Location for test
    protected WayPoint startLocation;

    //List of Locations for test
    protected WayPoint sampleLocation1;
    protected WayPoint sampleLocation2;
    protected WayPoint sampleLocation3;

    protected ElapsedTime autoTimer = new ElapsedTime();
    protected ElapsedTime autoTaskTimer = new ElapsedTime();

    HardwareOmnibot robot = new HardwareOmnibot();

    // Default to 4" wheels
    private static double myWheelSize = 4.0;
    // Default to 40:1 motors
    private static double myMotorRatio = 19.2;

    private static final double encoderClicksPerRev = 28;
    private static double clicksPerCm = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize * 2.54);


    public static final float MM_PER_INCH = 25.4f;

    public void setupRobotParameters(double newWheelSize, double newMotorRatio) {
        robot.init(hardwareMap);
        timer = new ElapsedTime();

        //robot.resetEncoders();
        robot.setInputShaping(false);
        myWheelSize = newWheelSize;
        myMotorRatio = newMotorRatio;

        clicksPerCm = (myMotorRatio * encoderClicksPerRev) / (Math.PI * myWheelSize * 2.54);
    }
    protected void updatePosition() {
        // Allow the robot to read sensors again
        robot.resetReads();
        MyPosition.giveMePositions(robot.getLeftEncoderWheelPosition(),
                robot.getRightEncoderWheelPosition(),
                robot.getStrafeEncoderWheelPosition());

        // Progress the robot actions.
        //performRobotActions();
    }

    protected void driveToWayPoint(WayPoint destination, boolean passThrough, boolean pullingFoundation) {
        // Loop until we get to destination.
        updatePosition();
        while (!robot.driveToXY(destination.x, destination.y, destination.angle,
                destination.speed, passThrough, pullingFoundation)
                && opModeIsActive()) {
            updatePosition();
        }
    }
    protected void rotateToWayPointAngle(WayPoint destination, boolean pullingFoundation) {
            // Move the robot away from the wall.
            updatePosition();
            robot.rotateToAngle(destination.angle, pullingFoundation, true);
            // Loop until we get to destination.
            updatePosition();
            while(!robot.rotateToAngle(destination.angle, pullingFoundation, false) && opModeIsActive()) {
                updatePosition();
            }
    }


}