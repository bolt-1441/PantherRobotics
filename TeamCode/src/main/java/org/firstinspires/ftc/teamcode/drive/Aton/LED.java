package org.firstinspires.ftc.teamcode.drive.Aton;

import static java.lang.Runtime.getRuntime;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class LED {
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor led;
    private boolean isFlashing = false;
    public LED(DcMotor led){
        this.led = led;
    }
    public void setBrightnessFlash(double brightness,int rate) {
        if (!isFlashing) {
            if ((1000*runtime.milliseconds()) >= 10) {
                isFlashing = true;
            } else {
                led.setPower(0);
            }
        } else {
            int time = (int)((1000*runtime.milliseconds()) * rate);
            if (time%2 == 0 ) {
                led.setPower(brightness);
            } else {
                led.setPower(0);
            }
        }
    }
    public void setBrightness(double brightness) {
        // Set the power of the motor to control the duty cycle of the PWM signal
        // A power of 0.0 corresponds to a duty cycle of 0% (LEDs off)
        // A power of 1.0 corresponds to a duty cycle of 100% (LEDs on)
        led.setPower(brightness);
    }
}
