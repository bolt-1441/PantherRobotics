package org.firstinspires.ftc.teamcode.drive.Aton;

import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class atonTest extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor turret = null;
    private Servo claw = null;
    NormalizedColorSensor colorSensor;

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    View relativeLayout;
    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 12 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 4 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);

    private DcMotor led = null;
    private boolean isFlashing = false;
    private static double wheelCircumference = 4.0;
    private Orientation angles;
    private Acceleration gravity;
    private double currentHeading, desiredHeading, currentX, currentY, desiredX, desiredY;
    private double Kp = 0.1;
    private double Ki = 0.01;
    private double Kd = 0.1;

    private  double turnPower = .3;

    double integralHeading = 0;
    double previousHeadingError = 0;
    double integralDistance = 0;
    double previousDistanceError = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        initBot();
        setMotorDirection();
        waitForStart();
        while (opModeIsActive()){
            loop2();
            desiredX = 0;
            desiredY = 0;
            telemetry.update();
        }


    }

    private void setMotorDirection() {
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
    }

    private void initBot(){
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "FrontLeft");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "BackLeft");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "FrontRight");
        rightBackDrive = hardwareMap.get(DcMotor.class, "BackRight");
        turret = hardwareMap.get(DcMotor.class, "turret");
        claw = hardwareMap.get(Servo.class,"wrist");
        led = hardwareMap.get(DcMotor.class,"LED");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();
    }

    public void loop2() {
        //Get the current heading, x, and y position from the IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        gravity = imu.getGravity();
        currentX = currentX +(gravity.xAccel * 0.01);
        currentY = currentY +(gravity.zAccel * 0.01);
        telemetry.addData("x",currentX);telemetry.addData("y",currentY);

        //Set the desired heading and position
        desiredHeading = Math.toDegrees(Math.atan2(desiredY - currentY, desiredX - currentX));

        //Calculate the error for heading
        double headingError = desiredHeading - currentHeading;
        if (headingError > 180) {
            headingError -= 360;
        } else if (headingError < -180) {
            headingError += 360;
        }

        //Calculate the error for distance
        double distanceError = Math.sqrt(Math.pow((desiredX - currentX), 2) + Math.pow((desiredY - currentY), 2));

        //Use the PID controller to adjust the heading and distance
        double turnPower = Kp*headingError + Ki*integralHeading + Kd*(headingError - previousHeadingError);
        integralHeading += headingError;
        previousHeadingError = headingError;
        double drivePower = Kp*distanceError + Ki*integralDistance + Kd*(distanceError - previousDistanceError);
        integralDistance += distanceError;
        previousDistanceError = distanceError;

        //Move the robot based on the calculated powers
        leftFrontDrive.setPower((drivePower - turnPower)/1000);
        leftBackDrive.setPower((drivePower - turnPower)/1000);
        rightFrontDrive.setPower((drivePower + turnPower)/1000);
        rightBackDrive.setPower((drivePower + turnPower)/1000);
    }

}
