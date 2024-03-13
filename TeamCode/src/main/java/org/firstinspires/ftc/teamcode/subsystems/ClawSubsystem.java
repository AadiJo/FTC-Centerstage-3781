package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ClawSubsystem implements Subsystem {
    Servo claw;
    enum ClawState {
        OPEN,
        CLOSED
    }
    ClawState state = ClawState.OPEN;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        claw = hardwareMap.servo.get("claw");
    }

    @Override
    public void update() {
        switch (state) {
            case OPEN:
                claw.setPosition(0.0);
                break;
            case CLOSED:
                claw.setPosition(0.6);
                break;
        }
    }

    @Override
    public void stop() {

    }

    public void open() {
        state = ClawState.OPEN;
    }

    public void close() {
        state = ClawState.CLOSED;
    }
}
