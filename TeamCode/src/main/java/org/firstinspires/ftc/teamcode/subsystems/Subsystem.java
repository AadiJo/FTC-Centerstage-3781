package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

public interface Subsystem {
    void initialize(HardwareMap hardwareMap);
    void update();
    void stop();
}