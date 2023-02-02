package org.firstinspires.ftc.teamcode.drive.Aton;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerImpl;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

class DriveTrain extends LinearOpMode {
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final double speedAvrage = 0.5;
    private BNO055IMU imu;
    private final double minSpeed = 0.05;
    private final double maxSpeed = 0.95;
    private final int wheelInches = 4;
    private final int tickRate = 28;

    private Orientation             lastAngles = new Orientation();
    double                  globalAngle, correction;

    public DriveTrain(DcMotor leftFrontMotor, DcMotor leftBackMotor, DcMotor rightFrontMotor, DcMotor rightBackMotor,BNO055IMU imu) {
        this.leftFrontMotor = leftFrontMotor;
        this.leftBackMotor = leftBackMotor;
        this.rightFrontMotor = rightFrontMotor;
        this.rightBackMotor = rightBackMotor;
        this.imu = imu;
    }

    public double rotationVeloity() {
        AngularVelocity angularVelocity = imu.getAngularVelocity();
        return (angularVelocity.zRotationRate + angularVelocity.xRotationRate + angularVelocity.yRotationRate)/3;

    }

    public void moveForward(double distance,double pow) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetPosition = (int)(distance / (wheelInches * Math.PI) * tickRate);
        leftFrontMotor.setTargetPosition(targetPosition);
        rightFrontMotor.setTargetPosition(targetPosition);
        leftBackMotor.setTargetPosition(targetPosition);
        rightBackMotor.setTargetPosition(targetPosition);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double leftFrontPower = pow + rotationVeloity()*0.01;
        double rightFrontPower = pow - rotationVeloity()*0.01;
        double leftBackPower = pow + rotationVeloity()*0.01;
        double rightBackPower = pow - rotationVeloity()*0.01;
        leftFrontPower = Range.clip(leftFrontPower, minSpeed, maxSpeed);
        rightFrontPower = Range.clip(rightFrontPower, minSpeed, maxSpeed);
        leftBackPower = Range.clip(leftBackPower, minSpeed, maxSpeed);
        rightBackPower = Range.clip(rightBackPower, minSpeed, maxSpeed);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy()) {
            //Wait for the motors to reach the target position
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    public void moveBackward(double distance,double pow) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetPosition = -(int) (distance / (wheelInches * Math.PI) * tickRate);
        leftFrontMotor.setTargetPosition(targetPosition);
        rightFrontMotor.setTargetPosition(targetPosition);
        leftBackMotor.setTargetPosition(targetPosition);
        rightBackMotor.setTargetPosition(targetPosition);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double leftFrontPower = pow + rotationVeloity()*0.01;
        double rightFrontPower = pow - rotationVeloity()*0.01;
        double leftBackPower = pow + rotationVeloity()*0.01;
        double rightBackPower = pow - rotationVeloity()*0.01;
        leftFrontPower = Range.clip(leftFrontPower, minSpeed, maxSpeed);
        rightFrontPower = Range.clip(rightFrontPower, minSpeed, maxSpeed);
        leftBackPower = Range.clip(leftBackPower, minSpeed, maxSpeed);
        rightBackPower = Range.clip(rightBackPower, minSpeed, maxSpeed);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        while (leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy()) {
            //Wait for the motors to reach the target position
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    public void strafeLeft(double distance,double pow) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetPosition = (int)(distance / (wheelInches * Math.PI) * tickRate);
        leftFrontMotor.setTargetPosition(-targetPosition);
        rightFrontMotor.setTargetPosition(targetPosition);
        leftBackMotor.setTargetPosition(targetPosition);
        rightBackMotor.setTargetPosition(-targetPosition);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double leftFrontPower = pow + rotationVeloity()*0.01;
        double rightFrontPower = pow - rotationVeloity()*0.01;
        double leftBackPower = pow + rotationVeloity()*0.01;
        double rightBackPower = pow - rotationVeloity()*0.01;
        leftFrontPower = Range.clip(leftFrontPower, minSpeed, maxSpeed);
        rightFrontPower = Range.clip(rightFrontPower, minSpeed, maxSpeed);
        leftBackPower = Range.clip(leftBackPower, minSpeed, maxSpeed);
        rightBackPower = Range.clip(rightBackPower, minSpeed, maxSpeed);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy()) {
            //Wait for the motors to reach the target position
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
    }
    public void strafeRight(double distance,double pow) {
        leftFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int targetPosition = (int)(distance / (wheelInches * Math.PI) * tickRate);
        leftFrontMotor.setTargetPosition(targetPosition);
        rightFrontMotor.setTargetPosition(-targetPosition);
        leftBackMotor.setTargetPosition(-targetPosition);
        rightBackMotor.setTargetPosition(targetPosition);
        leftFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        double leftFrontPower = pow + rotationVeloity()*0.01;
        double rightFrontPower = pow - rotationVeloity()*0.01;
        double leftBackPower = pow + rotationVeloity()*0.01;
        double rightBackPower = pow - rotationVeloity()*0.01;
        leftFrontPower = Range.clip(leftFrontPower, minSpeed, maxSpeed);
        rightFrontPower = Range.clip(rightFrontPower, minSpeed, maxSpeed);
        leftBackPower = Range.clip(leftBackPower, minSpeed, maxSpeed);
        rightBackPower = Range.clip(rightBackPower, minSpeed, maxSpeed);
        leftFrontMotor.setPower(leftFrontPower);
        rightFrontMotor.setPower(rightFrontPower);
        leftBackMotor.setPower(leftBackPower);
        rightBackMotor.setPower(rightBackPower);
        while(leftFrontMotor.isBusy() && rightFrontMotor.isBusy() && leftBackMotor.isBusy() && rightBackMotor.isBusy()) {
            //Wait for the motors to reach the target position
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
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
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightBackMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightFrontMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double  leftPower, rightPower;

        // restart imu movement tracking.
        double startDes = getAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        for (int i = 0; i < 3; i++) {


            if (degrees < startDes) {   // turn right.
                leftPower = power;
                rightPower = -power;
            } else if (degrees > startDes) {   // turn left.
                leftPower = -power;
                rightPower = power;
            } else return;
            telemetry.addData("left power: ", leftPower);telemetry.addData("right power: ", rightPower);
            // set power to rotate.
            leftBackMotor.setPower(leftPower);
            leftFrontMotor.setPower(leftPower);
            rightFrontMotor.setPower(rightPower);
            rightBackMotor.setPower(rightPower);

            // rotate until turn is completed.
            if (degrees < startDes) {
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
                    leftPower*=.99;
                    rightPower*=.99;
                    leftBackMotor.setPower(leftPower);
                    leftFrontMotor.setPower(leftPower);
                    rightBackMotor.setPower(rightPower);
                    rightFrontMotor.setPower(rightPower);
                }
            } else    // left turn.
                while (opModeIsActive() && getAngle() < degrees) {
                    telemetry.addData("left power: ", leftPower);telemetry.addData("right power: ", rightPower);
                    telemetry.addData("angle: ", getAngle());
                    telemetry.update();
                    leftPower*=.99;
                    rightPower*=.99;
                    leftBackMotor.setPower(leftPower);
                    leftFrontMotor.setPower(leftPower);
                    rightFrontMotor.setPower(rightPower);
                    rightBackMotor.setPower(rightPower);
                }

        }
        // turn the motors off.
        leftFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        rightFrontMotor.setPower(0);


        // reset angle tracking on new heading.
        resetAngle();
    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}

