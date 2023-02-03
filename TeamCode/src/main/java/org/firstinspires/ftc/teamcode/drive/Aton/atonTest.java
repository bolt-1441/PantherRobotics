package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class atonTest extends LinearOpMode {
    DriveTrain driveTrain = new DriveTrain("FrontLeft","BackLeft",
            "FrontRight","BackRight", hardwareMap);
    Arm arm = new Arm(hardwareMap.get(DcMotor.class, "turret"),hardwareMap.get(Servo.class,"wrist"));
    LED led = new LED(hardwareMap.get(DcMotor.class,"LED"));
    LimitSwitch limitSwitch = new LimitSwitch(hardwareMap,"limitSwitch");
    @Override
    public void runOpMode() throws InterruptedException {

    }
}
