package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "FirstController", group = "Pirhos")

public class FirstCode extends LinearOpMode {

private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;




    public void forward(double speed){
        backRight.setPower(-speed);
        backLeft.setPower(-speed);
        frontRight.setPower(speed);
        frontLeft.setPower(-speed);


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

        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        Hardware2 robot = new Hardware2(false);
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            time.reset();
            forward(gamepad1.left_stick_y);
            setRight(-gamepad1.right_stick_x);
            setLeft(gamepad1.right_stick_x);

}

    }}
