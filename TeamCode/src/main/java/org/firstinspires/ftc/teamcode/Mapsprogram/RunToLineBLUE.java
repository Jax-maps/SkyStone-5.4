package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auto-BLUE-LZ")
public class RunToLineBLUE extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
    HardwareSensors onbot = new HardwareSensors();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        onbot.init(hardwareMap);
        robot.initIMU();

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        telemetry.addData("Status", "Running");
        telemetry.update();

        drive(1, -1);
        sleep(275);
        drive(0, 0);

        turn(0.50, 1, -80);

        onbot.startDenesting();
        while( !(onbot.denestState.equals(HardwareSensors.DENEST_ACTIVITY.IDLE) )){
            onbot.performDenesting();
        }
        sleep(5200);

        drive(0.5, -1);
        sleep(850);

        drive(0,0);



    }

    public void shuffle(double power, int direction ) {
        robot.frontLeft.setDirection(DcMotor.Direction.REVERSE);
        robot.rearLeft.setDirection(DcMotor.Direction.REVERSE);

        robot.frontRight.setPower(power*direction);
        robot.rearRight.setPower(power);
        robot.frontLeft.setPower(power*direction);
        robot.rearLeft.setPower(power);
    }
    public void drive( double power,int direction) {
        robot.frontLeft.setDirection(DcMotor.Direction.FORWARD);
        robot.rearLeft.setDirection(DcMotor.Direction.FORWARD);

        robot.frontRight.setPower(power);
        robot.rearRight.setPower(power);
        robot.frontLeft.setPower(power*direction);
        robot.rearLeft.setPower(power*direction);
    }
    public void turn(double power, int direction, double angle) {
        double currentAngle = -robot.imu.getAngularOrientation().firstAngle;
        telemetry.addData("Angle", + currentAngle);
        telemetry.update();
        while(angle < ((Math.abs(currentAngle) - 1)*-1))
        {
            currentAngle = -robot.imu.getAngularOrientation().firstAngle;
            drive(power, direction);
        }
        drive(0, 0);
    }

}


