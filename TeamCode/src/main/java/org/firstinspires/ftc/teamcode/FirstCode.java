package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;//YESS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.Servo.Direction;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FirstController", group = "Pirhos")

public class FirstCode extends LinearOpMode  {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo clawL;
    private Servo clawR;

    private CRServo arm;
    public void strafe(double speed){
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }
//    public void strafeRight(double speed){
//        frontLeft.setPower(speed);
//        frontRight.setPower(speed);
//        backLeft.setPower(-speed);
//        backRight.setPower(speed);
//    }
//    public void strafeRight(double speed){
//        frontLeft.setPower(speed);
//        frontRight.setPower(-speed);
//        backLeft.setPower(speed);
//        backRight.setPower(speed);
//
//    }

    public void forward(double speed){
        backRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(-speed);
        frontLeft.setPower(speed);


    }
    public void setLeft(double speed){
        frontLeft.setPower(speed);
        backLeft.setPower(speed);
    }
    public void setRight(double speed){
        frontRight.setPower(speed);
        backRight.setPower(speed);

    }
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");
        arm = hardwareMap.crservo.get("arm");
        clawR = hardwareMap.servo.get("clawR");
        clawL = hardwareMap.servo.get("clawL");
        //BACK LEFT SHOULD BE NEGATIVE to go fwd
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {

            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(gamepad1.left_stick_y);
            frontLeft.setPower(gamepad1.left_stick_y);
            frontRight.setPower(gamepad1.left_stick_y);
        while(gamepad1.dpad_right){
            strafe(.5);
        }
        while(gamepad1.dpad_left){
            strafe(-.5);
        }

while (gamepad1.a)
{

    arm.setPower(-1);
}
while(gamepad1.b){
    arm.setPower(1);
}
arm.setPower(0);

if (gamepad1.left_bumper){
    clawR.setPosition(0);
    clawL.setPosition(1);


}
if (gamepad1.right_bumper){
                clawR.setPosition(1);
                clawL.setPosition(0);
            }
        }



    }}
