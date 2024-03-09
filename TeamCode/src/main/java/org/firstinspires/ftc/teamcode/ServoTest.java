package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
@Disabled
@TeleOp(name = "Servo Testing")
public class ServoTest extends LinearOpMode {

    Servo claw;
    DcMotorEx pullMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        claw = hardwareMap.get(Servo.class, "claw");
        pullMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        pullMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double armMotorPower = 1;
        double pullMotorPower = 0.75;


        waitForStart();
        while (opModeIsActive()){

            if (gamepad1.y){
                claw.setPosition(1);
            }
            if (gamepad1.b){
                claw.setPosition(0);
            }
        }






    }
}
