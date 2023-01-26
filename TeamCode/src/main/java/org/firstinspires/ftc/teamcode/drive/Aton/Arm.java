package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Arm {
    private DcMotor armMotor;
    private Servo griper;
    private int armMinHeight;
    private int armMaxHeight;
    private static int spoolInch = 2;
    private final int tickRate = 28;

    public Arm(DcMotor armMotor,Servo griper, int armMinHeight, int armMaxHeight) {
        this.griper = griper;
        this.armMotor = armMotor;
        this.armMinHeight = (int)(armMinHeight / (spoolInch * Math.PI) * tickRate);
        this.armMaxHeight = (int)(armMaxHeight / (spoolInch * Math.PI) * tickRate);
        this.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveToHeight(double height) {
        int targetPosition = (int)(height / (spoolInch * Math.PI) * tickRate);
        targetPosition = Range.clip(targetPosition, armMinHeight, armMaxHeight);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void lowG() {
        int targetPosition = 1500;
        targetPosition = Range.clip(targetPosition, armMinHeight, armMaxHeight);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void medG() {
        int targetPosition = 2250;
        targetPosition = Range.clip(targetPosition, armMinHeight, armMaxHeight);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void highG() {
        int targetPosition = 3000;
        targetPosition = Range.clip(targetPosition, armMinHeight, armMaxHeight);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void coneHigh(int coneNum){
        double height = 5.5 * coneNum;
        int targetPosition = (int)(height / (spoolInch * Math.PI) * tickRate);
        targetPosition = Range.clip(targetPosition, armMinHeight, armMaxHeight);
        armMotor.setTargetPosition(targetPosition);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotor.setPower(1);
        while(armMotor.isBusy()) {
            //Wait for the arm to reach the target height
        }
        armMotor.setPower(0);
    }
    public void openGripper() {
        griper.setPosition(1);
    }

    public void closeGripper() {
        griper.setPosition(.5);
    }

}
