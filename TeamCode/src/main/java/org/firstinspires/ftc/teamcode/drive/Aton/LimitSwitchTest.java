package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public class LimitSwitchTest extends LinearOpMode {
    LimitSwitch limitSwitch;

    @Override
    public void runOpMode() throws InterruptedException { // TODO: 1/21/2023 test this i dont think it will work might need one class 
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

