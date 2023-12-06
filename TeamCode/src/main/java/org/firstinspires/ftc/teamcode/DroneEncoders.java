package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class DroneEncoders {
    private Servo DroneServo = hardwareMap.get(Servo.class, "drone_servo");
    //use setVelocity() to make the encoders work the same speed even when the battery is lower
    final int test_cycles = 500;   // Number of control cycles to run to determine cycle times.
    private DcMotorEx m1, m2, m3, m4; // Motor Objects
    private long e1, e2, e3, e4; // Encoder Values
    private double v1, v2, v3, v4; // Velocities




}
