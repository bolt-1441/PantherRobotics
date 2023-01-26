package org.firstinspires.ftc.teamcode.drive.Aton;

import android.view.View;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp
public class atonTest extends LinearOpMode {
    DriveTrain driveTrain = new DriveTrain(hardwareMap.get(DcMotor.class, "FrontLeft"),hardwareMap.get(DcMotor.class, "FrontLeft"),
            hardwareMap.get(DcMotor.class, "FrontLeft"),hardwareMap.get(DcMotor.class, "FrontLeft"));

    @Override
    public void runOpMode() throws InterruptedException {

    }
}
