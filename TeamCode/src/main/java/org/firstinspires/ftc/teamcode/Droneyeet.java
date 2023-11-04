package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.lang.Math;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Droneyeet {
    yeet.motor = hardwareMap.get(DcMotor.class, "yeetmotor");
    boosh = hardwareMap.get(DcMotor.class, "boosh");
    while( gamepad1.a){
        yeetmotor.setPower(1);
        Servo boosh = boosh;
        boosh.setPosition(0);
        boosh.setPosition(1);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
}


//initialize yeet_motor()√
//Start_launch_motor(rpm)√
//initialize servo()√
// set_servo_limits (min degree, max degree)√
//push_drone(){
//    move_servo(max)
//}√
//Stop_servo(){
    //move_servo(min) 
// }√
////stop_launch_motor()√
    