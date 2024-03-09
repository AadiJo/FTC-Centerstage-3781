package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@Disabled
@TeleOp(name = "Cassette Drop Testing")
public class CassetteDropTest extends LinearOpMode {

    DcMotorEx armMotor;
    Servo door;


    @Override
    public void runOpMode() throws InterruptedException {
        door = hardwareMap.get(Servo.class, "door");
        boolean isOpen = false;
        ElapsedTime time = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        waitForStart();
        while (opModeIsActive()){
            while (gamepad1.a){ //&& !isOpen){
                door.setPosition(0);
                //isOpen = true;
                sleep(12);
                door.setPosition(0.6);
                sleep(20);
            }

        }


    }
}
