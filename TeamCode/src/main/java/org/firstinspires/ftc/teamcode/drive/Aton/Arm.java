package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

public class Arm {
    private DcMotor armMotor;
    private Servo griper;
    private static int spoolInch = 2;
    private final int tickRate = 28;

    public Arm(DcMotor armMotor, Servo griper) {
        this.griper = griper;
        this.armMotor = armMotor;
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToHeight(double height) {
        int targetPosition = (int) (height / (spoolInch * Math.PI) * tickRate);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }

    public void lowG() {
        int targetPosition = 1500;
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }

    public void medG() {
        int targetPosition = 2250;
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }

    public void highG() {
        int targetPosition = 3000;
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }

    public void coneHigh(int coneNum) {
        double height = 5.5 * coneNum;
        int targetPosition = (int) (height / (spoolInch * Math.PI) * tickRate);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while (armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
}
