package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


public class atonTest extends LinearOpMode {

    private DcMotor leftDrive, rightDrive;
    private BNO055IMU imu;
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

    }

    public void init2() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");
        leftDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        rightDrive.setDirection(DcMotorSimple.Direction.FORWARD);
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

    }

    public void loop2() {
        //Get the current heading, x, and y position from the IMU
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        currentHeading = angles.firstAngle;
        gravity = imu.getGravity();
        currentX = currentX + Math.cos(Math.toRadians(currentHeading)) * (gravity.xAccel * 0.01);
        currentY = currentY + Math.sin(Math.toRadians(currentHeading)) * (gravity.yAccel * 0.01);

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
        leftDrive.setPower(drivePower + turnPower);
        rightDrive.setPower(drivePower - turnPower);
    }

}
