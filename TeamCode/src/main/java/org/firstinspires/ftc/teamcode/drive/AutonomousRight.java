package org.firstinspires.ftc.teamcode.drive;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous(group = "drive")
public class AutonomousRight extends LinearOpMode {
    private Servo wrist = null;
    private DcMotor turret = null;
    NormalizedColorSensor colorSensor;
    View relativeLayout;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        wrist = hardwareMap.get(Servo.class,"wrist");
        turret = hardwareMap.get(DcMotor.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        runSample(); // actually execute the sample
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        wrist.setPosition(1);
        /////////////////////////THIS IS WHERE IT STARTS/////EVERYTHING BEFORE HERE IS INITIALIZATION/////////////
        waitForStart();
        sleep(1000);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        samePostitionarm();
        turret.setTargetPosition(700);
        turret.setPower(.3);
        samePostitionarm();
        sleep(1000);
        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .splineTo(new Vector2d(24, 0), 0)
                .build();
        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .splineTo(new Vector2d(60, 0), 0)
                .build();
        //Trajectory trajCen = drive.trajectoryBuilder(traj2.end(),true)
        //        .splineTo(new Vector2d(55, 0), Math.toRadians(180))
        //        .build();
        //Trajectory traj35 = drive.trajectoryBuilder(trajCen.end())
        //        .splineTo(new Vector2d(59, 5), Math.toRadians(45))
        //        .build();
        Trajectory traj4 = drive.trajectoryBuilder(traj2.end(),true)
                .splineTo(new Vector2d(48, 0), Math.toRadians(180))
                .build();
        Trajectory trajC = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(54, 0), Math.toRadians(-90))
                .build();
        Trajectory trajL = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(53, 23), Math.toRadians(-90))
                .build();
        Trajectory trajR = drive.trajectoryBuilder(traj4.end())
                .splineTo(new Vector2d(55, -22), Math.toRadians(-90))
                .build();
        drive.followTrajectory(traj1);
        sleep(100);
        int post = runSample();
        sleep(1000);
        drive.followTrajectory(traj2);
        //drive.followTrajectory(trajCen);
//        sleep(2000);
//        turret.setPower(.5);
//        turret.setTargetPosition(2950);
//        samePostitionarm();
//        sleep(4000);
        //drive.followTrajectory(traj35);
        //sleep(100);
        //sleep(1000);
        //turret.setTargetPosition(2750);
        //samePostitionarm();
        sleep(1000);
        wrist.setPosition(.5);
        sleep(100);
        drive.followTrajectory(traj4);
        turret.setPower(1);
        turret.setTargetPosition(5);
        samePostitionarm();
        sleep(1000);
        if(post == 3){
            drive.followTrajectory(trajR);
        }
        else if(post == 1){
            drive.followTrajectory(trajL);
        }
        else{
        }
        sleep(2000);
    }
    private void samePostitionarm(){
        if(turret.getTargetPosition() != turret.getCurrentPosition()){
            turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            telemetry.addData("Status", "Run Time: " + turret.getCurrentPosition());
            telemetry.update();
        }
    }
    //HOW TO MAKE THE ROBOT MOE TO A SPECIFIC SPOT//
//    drive.followTrajectory(
//            drive.trajectoryBuilder(traj.end(), true)
//            .splineTo(new Vector2d(0, 0), Math.toRadians(180))
//            .build()
//        );
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