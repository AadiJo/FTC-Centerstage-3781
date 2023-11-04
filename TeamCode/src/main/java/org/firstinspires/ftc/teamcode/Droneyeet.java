package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    