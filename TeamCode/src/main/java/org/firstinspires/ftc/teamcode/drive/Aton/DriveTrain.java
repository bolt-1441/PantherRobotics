package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;

class DriveTrain {
    private DcMotor leftFrontMotor, leftBackMotor, rightFrontMotor, rightBackMotor;

    private final double speedAvrage = 0.5;
    private BNO055IMU imu;
    private final double minSpeed = 0.05;
    private final double maxSpeed = 0.95;
    private final int wheelInches = 4;
    private final int tickRate = 28;

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


}

