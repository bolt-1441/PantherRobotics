package org.firstinspires.ftc.teamcode.drive.Aton;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(group = "drive")
public class AtonRightNonRoadRunner extends LinearOpMode {

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



    @Override
    public void runOpMode() throws InterruptedException {
        initBot();
        setMotorDirection();
        initialized();


        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        waitForStart();
        runtime.reset();
        claw.setPosition(1);

        sleep(1000);
        moveDistance(12,.3);
        moveDistanceR(12,.3);
        sleep(5000);



    }

    public void moveDistance(double distance, double speed) {
        // Calculate the number of rotations needed for each wheel
        double rotations = distance / wheelCircumference;

        // Set the target position for each wheel
        leftFrontDrive.setTargetPosition((int)(rotations * leftFrontDrive.getMotorType().getTicksPerRev()));
        leftBackDrive.setTargetPosition((int)(rotations * leftBackDrive.getMotorType().getTicksPerRev()));
        rightFrontDrive.setTargetPosition((int)(rotations * rightFrontDrive.getMotorType().getTicksPerRev()));
        rightBackDrive.setTargetPosition((int)(rotations * rightBackDrive.getMotorType().getTicksPerRev()));
        telemetry.addData("ticks ", leftBackDrive.getTargetPosition());

        // Set the mode for each wheel to run to position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("mode ", leftBackDrive.getMode());

        // Set the power for each wheel
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        telemetry.addData("speed ", speed);

        // Wait for the motors to finish moving
        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            correction = checkDirection();
            telemetry.addData("speed ", speed);
            telemetry.addData("mode ", leftBackDrive.getMode());
            telemetry.addData("ticks ", leftBackDrive.getTargetPosition());
            telemetry.addData("correction ", correction);
            leftFrontDrive.setPower(speed - correction);
            leftBackDrive.setPower(speed - correction);
            rightFrontDrive.setPower(speed + correction);
            rightBackDrive.setPower(speed + correction);
            telemetry.update();
        }

        // Stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void moveDistanceR(double distance, double speed) {
        // Calculate the number of rotations needed for each wheel
        double rotations = -distance / wheelCircumference;

        // Set the target position for each wheel
        leftFrontDrive.setTargetPosition((int)(rotations * leftFrontDrive.getMotorType().getTicksPerRev()));
        leftBackDrive.setTargetPosition((int)(rotations * leftBackDrive.getMotorType().getTicksPerRev()));
        rightFrontDrive.setTargetPosition((int)(rotations * rightFrontDrive.getMotorType().getTicksPerRev()));
        rightBackDrive.setTargetPosition((int)(rotations * rightBackDrive.getMotorType().getTicksPerRev()));
        telemetry.addData("ticks ", leftBackDrive.getTargetPosition());

        // Set the mode for each wheel to run to position
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        telemetry.addData("mode ", leftBackDrive.getMode());

        // Set the power for each wheel
        leftFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        telemetry.addData("speed ", speed);

        // Wait for the motors to finish moving
        while (leftFrontDrive.isBusy() && leftBackDrive.isBusy() && rightFrontDrive.isBusy() && rightBackDrive.isBusy()) {
            correction = checkDirection();
            telemetry.addData("speed ", speed);
            telemetry.addData("mode ", leftBackDrive.getMode());
            telemetry.addData("ticks ", leftBackDrive.getTargetPosition());
            telemetry.addData("correction ", correction);
            leftFrontDrive.setPower(speed + correction);
            leftBackDrive.setPower(speed + correction);
            rightFrontDrive.setPower(speed - correction);
            rightBackDrive.setPower(speed - correction);
            telemetry.update();
        }

        // Stop the motors
        leftFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightFrontDrive.setPower(0);
        rightBackDrive.setPower(0);

        leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    private void fix(){
        correction = (checkDirection())*5;telemetry.addData("correction", correction);
        leftFrontDrive.setPower(-correction);
        leftBackDrive.setPower(-correction);
        rightFrontDrive.setPower(correction);
        rightBackDrive.setPower(correction);
        telemetry.update();
    }

    private void initialized() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(0);
        telemetry.addData("Status", "Initialized");
        telemetry.update();

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

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    /**
     * Get current cumulative angle rotation from last reset.
     * @return Angle in degrees. + = left, - = right.
     */
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    private double getAccelX(){
        Acceleration acceleration = imu.getLinearAcceleration();
        double speed = -acceleration.yAccel;
        return speed;
    }
    private double getAccelY(){
        Acceleration acceleration = imu.getLinearAcceleration();
        double speed = acceleration.zAccel;
        return speed;
    }
    private double getAccelZ(){
        Acceleration acceleration = imu.getLinearAcceleration();
        double speed = acceleration.xAccel;
        return speed;
    }
    private double getAunglerSpeed(){
        AngularVelocity angularVelocity = imu.getAngularVelocity();
        double speed = -angularVelocity.zRotationRate;
        return speed;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .04;

        angle = getAngle();

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     * @param degrees Degrees to turn, + is left - is right
     */
    private void rotate(int degrees, double power)
    {
        leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double  leftPower, rightPower;

        // restart imu movement tracking.
        resetAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        for (int i = 0; i < 3; i++) {


            if (degrees < 0) {   // turn right.
                leftPower = power;
                rightPower = -power;
            } else if (degrees > 0) {   // turn left.
                leftPower = -power;
                rightPower = power;
            } else return;
            telemetry.addData("left power: ", leftPower);telemetry.addData("right power: ", rightPower);
            // set power to rotate.
            leftBackDrive.setPower(leftPower);
            leftFrontDrive.setPower(leftPower);
            rightBackDrive.setPower(rightPower);
            rightFrontDrive.setPower(rightPower);

            // rotate until turn is completed.
            if (degrees < 0) {
                // On right turn we have to get off zero first.
                while (opModeIsActive() && getAngle() == 0) {
                    telemetry.addData("left power: ", leftPower);telemetry.addData("right power: ", rightPower);
                    telemetry.addData("angle: ", getAngle());
                    telemetry.update();
                }

                while (opModeIsActive() && getAngle() > degrees) {
                    telemetry.addData("left power: ", leftPower);telemetry.addData("right power: ", rightPower);
                    telemetry.addData("angle: ", getAngle());
                    telemetry.update();
                }
            } else    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {
                    telemetry.addData("left power: ", leftPower);telemetry.addData("right power: ", rightPower);
                    telemetry.addData("angle: ", getAngle());
                    telemetry.update();
                }
        }
        // turn the motors off.
        leftBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
        rightBackDrive.setPower(0);
        rightFrontDrive.setPower(0);


        // reset angle tracking on new heading.
        resetAngle();
    }
    public void setBrightnessFlash(double brightness) {
        if (!isFlashing) {
            if (getRuntime() >= 80) {
                isFlashing = true;
            } else {
                led.setPower(0);
            }
        } else {
            int time = (int)(getRuntime() * 5);
            if (time%2 == 0 ) {
                led.setPower(brightness);
            } else {
                led.setPower(0);
            }
        }
    }
    protected int runSample() {
        // You can give the sensor a gain value, will be multiplied by the sensor's raw value before the
        // normalized color values are calculated. Color sensors (especially the REV Color Sensor V3)
        // can give very low values (depending on the lighting conditions), which only use a small part
        // of the 0-1 range that is available for the red, green, and blue values. In brighter conditions,
        // you should use a smaller gain than in dark conditions. If your gain is too high, all of the
        // colors will report at or near 1, and you won't be able to determine what color you are
        // actually looking at. For this reason, it's better to err on the side of a lower gain
        // (but always greater than  or equal to 1).
        float gain = 5;
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Once per loop, we will update this hsvValues array. The first element (0) will contain the
        // hue, the second element (1) will contain the saturation, and the third element (2) will
        // contain the value. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
        // for an explanation of HSV color.
        final float[] hsvValues = new float[3];

        // xButtonPreviouslyPressed and xButtonCurrentlyPressed keep track of the previous and current
        // state of the X button on the gamepad
        boolean xButtonPreviouslyPressed = false;
        boolean xButtonCurrentlyPressed = false;

        // Get a reference to our sensor object. It's recommended to use NormalizedColorSensor over
        // ColorSensor, because NormalizedColorSensor consistently gives values between 0 and 1, while
        // the values you get from ColorSensor are dependent on the specific sensor you're using.

        // If possible, turn the light on in the beginning (it might already be on anyway,
        // we just make sure it is if we can).
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight)colorSensor).enableLight(true);
        }

        // Wait for the start button to be pressed.
        waitForStart();

        // Loop until we are asked to stop
        // while (opModeIsActive()) {
        // Explain basic gain information via telemetry
        //telemetry.addLine("Hold the A button on gamepad 1 to increase gain, or B to decrease it.\n");
        telemetry.addLine("Higher gain values mean that the sensor will report larger numbers for Red, Green, and Blue, and Value\n");

        // Show the gain value via telemetry
        telemetry.addData("Gain", gain);

        // Tell the sensor our desired gain value (normally you would do this during initialization,
        // not during the loop)
        colorSensor.setGain(gain);

        // Get the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        /* Use telemetry to display feedback on the driver station. We show the red, green, and blue
         * normalized values from the sensor (in the range of 0 to 1), as well as the equivalent
         * HSV (hue, saturation and value) values. See http://web.archive.org/web/20190311170843/https://infohost.nmt.edu/tcc/help/pubs/colortheory/web/hsv.html
         * for an explanation of HSV color. */

        // Update the hsvValues array by passing it to Color.colorToHSV()
        Color.colorToHSV(colors.toColor(), hsvValues);

        telemetry.addLine()
                .addData("Red", "%.3f", colors.red)
                .addData("Green", "%.3f", colors.green)
                .addData("Blue", "%.3f", colors.blue);
        telemetry.addLine()
                .addData("Hue", "%.3f", hsvValues[0])
                .addData("Saturation", "%.3f", hsvValues[1])
                .addData("Value", "%.3f", hsvValues[2]);
        telemetry.addData("Alpha", "%.3f", colors.alpha);

        /* If this color sensor also has a distance sensor, display the measured distance.
         * Note that the reported distance is only useful at very close range, and is impacted by
         * ambient light and surface reflectivity. */
        if (colorSensor instanceof DistanceSensor) {
            telemetry.addData("Distance (cm)", "%.3f", ((DistanceSensor) colorSensor).getDistance(CM));
        }

        telemetry.update();

        // Change the Robot Controller's background color to match the color detected by the color sensor.
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(hsvValues));
            }
        });
        double red = colors.red;
        double green = colors.green;
        double blue = colors.blue;
        if(red>green)
            if(red>blue)
                return 1;
        if(green>red)
            if(green>blue)
                return 2;
        if(blue>red)
            if(blue>green)
                return 3;
        return 4;
        //}
    }


}

