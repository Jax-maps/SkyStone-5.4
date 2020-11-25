package org.firstinspires.ftc.teamcode.Mapsprogram;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import android.os.SystemClock;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.max;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.toRadians;

import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.WayPoint;
import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.MyPosition;
import org.firstinspires.ftc.teamcode.Mapsprogram.RobotUtilities.MovementVars;

import java.util.List;

/**
 *Created by MAPS
 */
public class HardwareOmnibotDrive
{
    /* Public OpMode members. */
    public final static double MIN_SPIN_RATE = 0.05;
    public final static double MIN_DRIVE_RATE = 0.05;
    public final static double MIN_STRAFE_RATE = 0.19;
    public final static double MIN_FOUNDATION_SPIN_RATE = 0.19;
    public final static double STRAFE_MULTIPLIER = 1.5;
    public final static double SLOW_STRAFE_MULTIPLIER = 1.5;
    public final static double MIN_FOUNDATION_DRIVE_RATE = 0.18;
    public final static double MIN_FOUNDATION_STRAFE_RATE = 0.19;
    public final static double MIN_DRIVE_MAGNITUDE = Math.sqrt(MIN_DRIVE_RATE*MIN_DRIVE_RATE+MIN_DRIVE_RATE*MIN_DRIVE_RATE);
    public final static double MIN_FOUNDATION_DRIVE_MAGNITUDE = Math.sqrt(MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE+MIN_FOUNDATION_DRIVE_RATE*MIN_FOUNDATION_DRIVE_RATE);

    // Robot Controller Config Strings
    public final static String IMU = "imu";
    public final static String FRONT_LEFT_MOTOR = "l1";
    public final static String FRONT_RIGHT_MOTOR = "r1";
    public final static String REAR_LEFT_MOTOR = "l2";
    public final static String REAR_RIGHT_MOTOR = "r2";
    public final static String SENSOR_RANGE_1 = "range1";
    public final static String SENSOR_RANGE_2 = "range2";

    // Hardware objects
    protected DcMotor frontLeft = null;
    protected DcMotor frontRight = null;
    protected DcMotor rearLeft = null;
    protected DcMotor rearRight = null;
    protected BNO055IMU imu = null;
    public DistanceSensor sensorRange; // Sense distance from stone
    public DistanceSensor sensorRange2; // Confirm distance from stone

    List<LynxModule> allHubs;

    public static boolean encodersReset = false;
    public boolean forceReset = false;

    // Tracking variables
    private static final int encoderClicksPerSecond = 2800;
    protected double frontLeftMotorPower = 0.0;
    protected double rearLeftMotorPower = 0.0;
    protected double frontRightMotorPower = 0.0;
    protected double rearRightMotorPower = 0.0;
    private boolean inputShaping = true;
    protected boolean imuRead = false;
    protected double imuValue = 0.0;


    protected double strafeMultiplier = STRAFE_MULTIPLIER;

    /* local OpMode members. */
    protected HardwareMap hwMap  =  null;

    /* Constructor */
    public HardwareOmnibotDrive(){
    }

    public void setInputShaping(boolean inputShapingEnabled) {
        inputShaping = inputShapingEnabled;
    }

    public void initIMU()
    {
        // Init IMU code
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hwMap.get(BNO055IMU.class, IMU);
        imu.initialize(parameters);
    }

    public void resetReads() {
        // This tells the expansion hub to bulk read all the data next time you
        // read from the hub.
        for (LynxModule module : allHubs) {
            module.clearBulkCache();
        }
        imuRead = false;
    }

    public double readIMU()
    {
        if(!imuRead) {
            // Read IMU Code
            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            imuValue = (double)angles.firstAngle;
            imuRead = true;
        }

        return imuValue;
    }

    public double readBackLeftTo() {
        return sensorRange.getDistance(DistanceUnit.INCH);
    }

    public double readBackRightTo(){

        return sensorRange2.getDistance(DistanceUnit.INCH);
    }

    public void setFrontLeftMotorPower(double power)
    {
        if(power != frontLeftMotorPower)
        {
            frontLeftMotorPower = power;
            frontLeft.setPower(power);
        }
    }

    public void setRearLeftMotorPower(double power)
    {
        if(power != rearLeftMotorPower)
        {
            rearLeftMotorPower = power;
            rearLeft.setPower(power);
        }
    }

    public void setFrontRightMotorPower(double power)
    {
        if(power != frontRightMotorPower)
        {
            frontRightMotorPower = power;
            frontRight.setPower(power);
        }
    }

    public void setRearRightMotorPower(double power)
    {
        if(power != rearRightMotorPower)
        {
            rearRightMotorPower = power;
            rearRight.setPower(power);
        }
    }

    public void setPowerforAll( double rf, double rb, double lf, double lb) {
        setFrontRightMotorPower(rf);
        setRearRightMotorPower(rb);
        setFrontLeftMotorPower(lf);
        setRearLeftMotorPower(lb);
    }

    public void setAllDrive(double power) {
        setFrontLeftMotorPower(power);
        setFrontRightMotorPower(power);
        setRearRightMotorPower(power);
        setRearLeftMotorPower(power);
    }

    public void setAllDriveZero()
    {
        setAllDrive(0.0);
    }

    /**
     *
     * @param xPower - -1.0 to 1.0 power in the X axis
     * @param yPower - -1.0 to 1.0 power in the Y axis
     * @param spin - -1.0 to 1.0 power to rotate the robot
     * @param angleOffset - The offset from the gyro to run at, such as drive compensation
     */
    public void drive(double xPower, double yPower, double spin, double angleOffset) {
        double gyroAngle = readIMU() + angleOffset;
        double leftFrontAngle = toRadians(135.0 + gyroAngle); // previous value 45
        double rightFrontAngle = toRadians(45.0 + gyroAngle); // previous value -45
        double leftRearAngle = toRadians(225.0 + gyroAngle); // previous value 135
        double rightRearAngle = toRadians(-45.0 + gyroAngle); // previous value -135
        double joystickMagnitude = sqrt(xPower*xPower + yPower*yPower);
        double joystickAngle = atan2(yPower, xPower);
        double newPower = driverInputShaping(joystickMagnitude);
        double newSpin = driverInputSpinShaping(spin);
        double newXPower = newPower * cos(joystickAngle) ;
        double newYPower = newPower * sin(joystickAngle) ;

        double LFpower = newXPower * cos(leftFrontAngle) + newYPower * sin(leftFrontAngle) + newSpin;
        double LRpower = newXPower * cos(leftRearAngle) + newYPower * sin(leftRearAngle) + newSpin;
        double RFpower = newXPower * cos(rightFrontAngle) + newYPower * sin(rightFrontAngle) + newSpin;
        double RRpower = newXPower * cos(rightRearAngle) + newYPower * sin(rightRearAngle) + newSpin;

        double maxPower = max(1.0, max(max(abs(LFpower), abs(LRpower)),
                max(abs(RFpower), abs(RRpower))));

        if(maxPower > 1.0) {
            LFpower /= maxPower;
            RFpower /= maxPower;
            RFpower /= maxPower;
            RRpower /= maxPower;
        }

        setFrontLeftMotorPower(LFpower);
        setFrontRightMotorPower(RFpower);
        setRearRightMotorPower(RRpower);
        setRearLeftMotorPower(LRpower);
    }

    /**
     * @param targetAngle  - The angle the robot should try to face when reaching destination.
     * @param pullingFoundation - If we are pulling the foundation.
     * @param resetDriveAngle - When we start a new drive, need to reset the starting drive angle.
     * @return - Boolean true we have reached destination, false we have not
     */
    public double lastDriveAngle;
    public boolean rotateToAngle(double targetAngle, boolean pullingFoundation, boolean resetDriveAngle) {
        boolean reachedDestination = false;
        double errorMultiplier = pullingFoundation ? 0.04 : 0.016;
        double minSpinRate = pullingFoundation ? MIN_FOUNDATION_SPIN_RATE : MIN_SPIN_RATE;
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;

        // This should be set on the first call to start us on a new path.
        if(resetDriveAngle) {
            lastDriveAngle = deltaAngle;
        }

        // We are done if we are within 2 degrees
        if(Math.abs(Math.toDegrees(deltaAngle)) < 2) {
            // We have reached our destination if the angle is close enough
            setAllDriveZero();
            reachedDestination = true;
            // We are done when we flip signs.
        } else if(lastDriveAngle < 0) {
            // We have reached our destination if the delta angle sign flips from last reading
            if(deltaAngle >= 0) {
                setAllDriveZero();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                MovementVars.movement_x = 0;
                MovementVars.movement_y = 0;
                if(turnSpeed > -minSpinRate) {
                    turnSpeed = -minSpinRate;
                }
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            // We have reached our destination if the delta angle sign flips
            if(deltaAngle <= 0) {
                setAllDriveZero();
                reachedDestination = true;
            } else {
                // We still have some turning to do.
                MovementVars.movement_x = 0;
                MovementVars.movement_y = 0;
                if(turnSpeed < minSpinRate) {
                    turnSpeed = minSpinRate;
                }
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        }
        lastDriveAngle = deltaAngle;

        return reachedDestination;
    }

    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle - The angle the robot should try to face when reaching destination in radians.
     * @param minSpeed    - The minimum speed that allows movement.
     * @param maxSpeed    - Sets the maximum speed to drive.
     * @param errorMultiplier - Sets the proportional speed to slow down.
     * @param allowedError - Sets the allowable error to claim target reached.
     * @param passThrough - Allows waypoint to be a drive through where the robot won't slow down.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double minSpeed,
                             double maxSpeed, double errorMultiplier, double allowedError,
                             boolean passThrough) {
        boolean reachedDestination = false;
        double deltaX = x - MyPosition.worldXPosition;
        double deltaY = y - MyPosition.worldYPosition;
        double driveAngle = Math.atan2(deltaY, deltaX);
        double deltaAngle = MyPosition.AngleWrap(targetAngle - MyPosition.worldAngle_rad);
        double magnitude = Math.sqrt(deltaX * deltaX + deltaY * deltaY);
        double driveSpeed;
        double turnSpeed = Math.toDegrees(deltaAngle) * errorMultiplier;
        // Have to convert from world angles to robot centric angles.
        double robotDriveAngle = driveAngle - MyPosition.worldAngle_rad + Math.toRadians(90);

        // This will allow us to do multi-point routes without huge slowdowns.
        // Such use cases will be changing angles, or triggering activities at
        // certain points.
        if(!passThrough) {
            driveSpeed = magnitude * errorMultiplier;
        } else {
            driveSpeed = maxSpeed;
        }

        if(driveSpeed < minSpeed) {
            driveSpeed = minSpeed;
        } else if (driveSpeed > maxSpeed) {
            driveSpeed = maxSpeed;
        }

        // Check if we passed through our point
        if(magnitude <= allowedError) {
            reachedDestination = true;
            if(!passThrough) {
                setAllDriveZero();
            } else {
                // This can happen if the robot is already at error distance for drive through
                MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
                MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
                MovementVars.movement_turn = turnSpeed;
                ApplyMovement();
            }
        } else {
            MovementVars.movement_x = driveSpeed * Math.cos(robotDriveAngle);
            MovementVars.movement_y = driveSpeed * Math.sin(robotDriveAngle);
            MovementVars.movement_turn = turnSpeed;
            ApplyMovement();
        }

        return reachedDestination;
    }


    protected double driverInputShaping( double valueIn) {
        double valueOut = 0.0;

        if(Math.abs(valueIn) < MIN_DRIVE_RATE) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                if (valueIn > 0) {
                    valueOut = MIN_DRIVE_RATE + (1.0 - MIN_DRIVE_RATE) * valueIn;
                } else {
                    valueOut = -MIN_DRIVE_RATE + (1.0 - MIN_DRIVE_RATE) * valueIn;
                }
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    protected double driverInputSpinShaping( double valueIn) {
        double valueOut = 0.0;

        if(Math.abs(valueIn) < MIN_SPIN_RATE) {
            valueOut = 0.0;
        } else {
            if (inputShaping) {
                if (valueIn > 0) {
                    valueOut = (1.0 + MIN_SPIN_RATE) * valueIn - MIN_SPIN_RATE;
                } else {
                    valueOut = (1.0 + MIN_SPIN_RATE) * valueIn + MIN_SPIN_RATE;
                }
            } else {
                valueOut = valueIn;
            }
        }

        return valueOut;
    }

    public void disableDriveEncoders()
    {
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void enableDriveEncoders(){
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetDriveEncoders()
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
            encoderCount = frontLeft.getCurrentPosition();
        }

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Set the stop mode
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public int getLeftEncoderWheelPosition() {
        // This is to compensate for GF having a negative left.
        return -rearLeft.getCurrentPosition();
    }

    public int getRightEncoderWheelPosition() {
        return frontRight.getCurrentPosition();
    }

    public int getStrafeEncoderWheelPosition() {
        return frontLeft.getCurrentPosition();
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // This tells the hubs to go into bulk read mode. This is faster to read
        // the encoders. Typically it takes a few mSec to read each encoder, so
        // if you are reading all three encoders every loop, it causes quite a
        // delay. This reads all the encoders on a hub all at once when you
        // call the resetReads function.
        allHubs = hwMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        // Define and Initialize Motors
        frontLeft = hwMap.dcMotor.get(FRONT_LEFT_MOTOR);
        frontRight  = hwMap.dcMotor.get(FRONT_RIGHT_MOTOR);
        rearLeft = hwMap.dcMotor.get(REAR_LEFT_MOTOR);
        rearRight = hwMap.dcMotor.get(REAR_RIGHT_MOTOR);
        sensorRange = hwMap.get(DistanceSensor.class, SENSOR_RANGE_1);
        sensorRange2 = hwMap.get(DistanceSensor.class, SENSOR_RANGE_2);

        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        rearLeft.setDirection(DcMotor.Direction.FORWARD);
        rearRight.setDirection(DcMotor.Direction.FORWARD);

        // Set all motors to zero power
        setAllDriveZero();

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //resetDriveEncoders(); //Had to exclude for drive to run properly

        initIMU();
    }
    public boolean gotoRearTarget(double driveSpeed, double spinSpeed){
        //These are the values we see when it is against the platform
        double backLeftTargetDistance = 2.0;
        double backRightTargetDistance = 4.5;

        double leftToDistance = readBackLeftTo();
        double rightToDistance = readBackRightTo();

        double drivePower = 0.0;
        double spinPower = 0.0;

        double leftError = backLeftTargetDistance- leftToDistance;
        double rightError = backRightTargetDistance - rightToDistance;
        boolean touching = false;

        //Slow down for 10cm
        double slowDownStart = 20.0;
        double slowDownRange = 10.0;
        if((leftError < 0) || (rightError < 0)){
            //Have to drive backwards towards the foundation
            drivePower = driveSpeed;
            //if one of them is within range, drive speed should be minimum, all
            // rotation
            if(!((rightError < 0) && (leftError < 0))){
                drivePower = MIN_DRIVE_RATE;
                spinPower = Math.copySign(spinPower, MIN_SPIN_RATE);

            } else {
                // We want the one closer to the foundation minError should be negative.
                double minError = Math.max(rightError, leftError);

                // Need to set drive power based on range.
                // Go at slowest speed.
                double scaleFactor = 0.95 * (slowDownRange - (slowDownStart + minError))/ slowDownRange + MIN_DRIVE_RATE;
                scaleFactor = Math.min(1.0, scaleFactor);
                drivePower = driveSpeed * scaleFactor;
            }
            // make sure we don't go below minimum spin power;
            if(spinPower < MIN_SPIN_RATE) {
                spinPower = Math.copySign(spinPower, MIN_SPIN_RATE);
            }
            // make sure we don't go below minimum drive power.
            if(drivePower < MIN_DRIVE_RATE){
                drivePower = MIN_DRIVE_RATE;
            }
            // Scale the power based on how far
            drive( 0, -drivePower, -spinPower, -readIMU());
        }else {
            drive(0, -MIN_DRIVE_RATE, 0, -readIMU());
            touching = true;
        }
        return touching;
    }

    /**
     * @param x           - The X field coordinate to go to.
     * @param y           - The Y field coordinate to go to.
     * @param targetAngle  - The angle the robot should try to face when reaching destination in radians.
     * @param maxSpeed    - Sets the speed when we are driving through the point.
     * @param passThrough - Slows the robot down to stop at destination coordinate.
     * @param pullingFoundation - If we are pulling the foundation.
     * @return - Boolean true we have reached destination, false we have not
     */
    public boolean driveToXY(double x, double y, double targetAngle, double maxSpeed,
                             boolean passThrough, boolean pullingFoundation) {
        double errorMultiplier = pullingFoundation ? 0.020 : 0.014;
        double minDriveMagnitude = pullingFoundation ? MIN_FOUNDATION_DRIVE_MAGNITUDE : MIN_DRIVE_MAGNITUDE;
        double allowedError = 2;

        if(passThrough) {
            allowedError = 7;
        }
        return (driveToXY(x, y, targetAngle, minDriveMagnitude, maxSpeed, errorMultiplier,
                allowedError, passThrough));
    }

    // Odometry updates
    private long lastUpdateTime = 0;

    /**converts movement_y, movement_x, movement_turn into motor powers */
    public void ApplyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        // 2.1 is the ratio between the minimum power to strafe, 0.19, and driving, 0.09.
        double tl_power_raw = MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;
        double bl_power_raw = MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double br_power_raw = -MovementVars.movement_y-MovementVars.movement_turn-MovementVars.movement_x*strafeMultiplier;
        double tr_power_raw = -MovementVars.movement_y-MovementVars.movement_turn+MovementVars.movement_x*strafeMultiplier;

        //find the maximum of the powers
        double maxRawPower = Math.abs(tl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(tr_power_raw) > maxRawPower){ maxRawPower = Math.abs(tr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        tl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        tr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        setFrontLeftMotorPower(tl_power_raw);
        setFrontRightMotorPower(tr_power_raw);
        setRearRightMotorPower(br_power_raw);
        setRearLeftMotorPower(bl_power_raw);
    }
}
