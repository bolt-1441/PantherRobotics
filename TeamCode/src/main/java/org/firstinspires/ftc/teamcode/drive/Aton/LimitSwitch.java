package org.firstinspires.ftc.teamcode.drive.Aton;

import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LimitSwitch {
    private DigitalChannel switchPin;

    public LimitSwitch(HardwareMap hardwareMap, String switchName) {
        switchPin = hardwareMap.get(DigitalChannel.class, switchName);
        switchPin.setMode(DigitalChannel.Mode.INPUT);
    }

    public boolean isPressed() {
        return switchPin.getState();
    }
}
