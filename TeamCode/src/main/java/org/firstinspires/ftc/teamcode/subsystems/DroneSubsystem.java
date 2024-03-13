package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DroneSubsystem implements Subsystem{
    DcMotorEx drone;
    CRServo flicker;
    double ticksPerRevDrone = 103.8;
    public double maxTicksPerSec = (double) 1620 /60 * ticksPerRevDrone;
    enum DroneState {
        ON,
        OFF
    }

    DroneState state = DroneState.OFF;
    @Override
    public void initialize(HardwareMap hardwareMap) {
        drone.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        flicker = hardwareMap.crservo.get("drone");
    }

    @Override
    public void update() {
        switch (state) {
            case ON:
                drone.setVelocity(-0.85 * maxTicksPerSec);
                new WaitCommand(1000);
                flicker.setPower(-1);
                new WaitCommand(200);
                break;
            case OFF:
                drone.setPower(0);
                flicker.setPower(0);
                break;
        }
    }

    @Override
    public void stop() {state = DroneState.OFF;}

    public void launch() {state = DroneState.ON;}

}
