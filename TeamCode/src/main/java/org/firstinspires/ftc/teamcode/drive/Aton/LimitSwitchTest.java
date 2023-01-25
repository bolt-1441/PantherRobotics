package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class LimitSwitchTest extends LinearOpMode {
    LimitSwitch limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException {
        limitSwitch = new LimitSwitch(hardwareMap,"limitSwitch");
        waitForStart();
        while (opModeIsActive()){
            if(limitSwitch.isPressed())
                telemetry.addData("Test:",limitSwitch.isPressed());
            telemetry.addData("limit Switch:",limitSwitch.isPressed());
            telemetry.update();
        }
    }
    
}

