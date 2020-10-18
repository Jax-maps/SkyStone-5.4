//package org.firstinspires.ftc.teamcode.Mapsprogram;
//
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import java.util.List;
//
//@Autonomous
//public class AutoRedBuild extends LinearOpMode {
//    private ElapsedTime runtime = new ElapsedTime();
//    HardwareOmnibotDrive robot = new HardwareOmnibotDrive();
//    HardwareSensors onbot = new HardwareSensors();
//    final double COUNTS_PER_INCH = 307.699557;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        onbot.init(hardwareMap);
//        robot.init(hardwareMap);
//        robot.initIMU();
//        robot.enableDriveEncoders();
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        telemetry.addData("Status", "Initialized");
//        telemetry.update();
//        // Wait for the game to start (driver presses PLAY)
//        waitForStart();
//
//        // run until the end of the match (driver presses STOP)
//        telemetry.addData("Status", "Running");
//        telemetry.update();
//
//        //Unhook the foundation
//        onbot.hook1.setPosition(0);
//        onbot.hook2.setPosition(0.5);//0.5
//        sleep(1000);
//
//        //Drive forward
//        yMovement(0.5, 15, 1.5);
//
//        //Shuffle
//
//
//        telemetry.addData("Status", + robot.rearLeft.getCurrentPosition());
//        telemetry.addData("Status", + robot.frontRight.getCurrentPosition());
//        telemetry.update();
//        sleep(8000);
//    }
//    public void yMovement(double power, double distance, double allowableDistance){
//        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        telemetry.addData("Counts", "Check");
//        telemetry.update();
//
//        double countsToGo = distance*COUNTS_PER_INCH;
//        double error = allowableDistance*COUNTS_PER_INCH;
//        double averageEncoders = (robot.rearLeft.getCurrentPosition() + robot.frontRight.getCurrentPosition())/2;
//
//        while( (averageEncoders <= (countsToGo - error)) || (robot.readBackLeftTo() > error) || (robot.readBackRightTo() > error)){
//            averageEncoders = (robot.frontLeft.getCurrentPosition() + robot.rearRight.getCurrentPosition())/2;
//            robot.readBackRightTo();
//            robot.readBackLeftTo();
//
//            robot.setFrontLeftMotorPower(power);
//            robot.setRearLeftMotorPower(power);
//            robot.setFrontRightMotorPower(power*(-1));
//            robot.setRearRightMotorPower(power*(-1));
//
//            telemetry.addData("Counts", + averageEncoders);
//            telemetry.update();
//        }
//        robot.setFrontLeftMotorPower(0);
//        robot.setRearLeftMotorPower(0);
//        robot.setFrontRightMotorPower(0);
//        robot.setRearRightMotorPower(0);
//    }
//    public void xMovement(double power, double distance, double allowableDistance){
//        robot.rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        double countsToGo = distance*COUNTS_PER_INCH;
//        double error = allowableDistance*COUNTS_PER_INCH;
//        double horizontalEncoder = robot.frontLeft.getCurrentPosition();
//        while( (horizontalEncoder <= Math.abs(countsToGo - error)) || (robot.readBackLeftTo() > error) || (robot.readBackRightTo() > error)){
//            horizontalEncoder = robot.frontLeft.getCurrentPosition();
//            robot.readBackRightTo();
//            robot.readBackLeftTo();
//
//            robot.setFrontLeftMotorPower(power);
//            robot.setRearLeftMotorPower(power);
//            robot.setFrontRightMotorPower(power);
//            robot.setRearRightMotorPower(power);
//        }
//        robot.setFrontLeftMotorPower(0);
//        robot.setRearLeftMotorPower(0);
//        robot.setFrontRightMotorPower(0);
//        robot.setRearRightMotorPower(0);
//    }
//    public int distanceSensors() {
//        return 10;
//    }
//    public void spin(int degrees, double speed){
//
//    }
//
//}
