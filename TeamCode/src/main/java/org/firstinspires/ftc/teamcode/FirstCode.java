package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;//YESS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@TeleOp(name = "Bug Squasher", group = "Pirhos")

public class FirstCode extends LinearOpMode  {
private BHI260IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private Servo claw;
    private CRServo arm;
    public void strafe(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
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
        frontRight.setPower(-speed);
        backRight.setPower(speed);

    }
    public void runOpMode() throws InterruptedException {
        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");
        arm = hardwareMap.crservo.get("arm");
        claw = hardwareMap.servo.get("claw");


        imu = hardwareMap.get(BHI260IMU.class,"imu");
        //BACK LEFT SHOULD BE NEGATIVE to go fwd
        PermissionValidatorWrapper.Hardware2 robot = new PermissionValidatorWrapper.Hardware2(false);
        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
//            telemetry.addData("Current IMU", imu.getAngularOrientation(AxesReference.EXTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle );
            backLeft.setPower(gamepad1.left_stick_y);
            backRight.setPower(-gamepad1.left_stick_y);
            frontLeft.setPower(-gamepad1.left_stick_y);
            frontRight.setPower(-gamepad1.left_stick_y);


            //strafing commands
            while(gamepad1.dpad_right){
            strafe(.5);
        }
        while(gamepad1.dpad_left){
            strafe(-.5);
        }


        // controls the arm
        while (gamepad1.a)
{

    arm.setPower(-1);
}
while(gamepad1.b){
    arm.setPower(1);
}
arm.setPower(0);

//sets claw positions
if (gamepad1.left_bumper){
    claw.setPosition(1);


}

if (gamepad1.right_bumper){
                claw.setPosition(0);
            }




            //turns the robot left and right
if((gamepad1.right_stick_x)>0){
    backLeft.setPower(gamepad1.left_stick_y);
    backRight.setPower(gamepad1.left_stick_y);
    frontLeft.setPower(-gamepad1.left_stick_y);
    frontRight.setPower(gamepad1.left_stick_y);
}
            if((gamepad1.right_stick_x)<0){
                backLeft.setPower(-gamepad1.left_stick_y);
                backRight.setPower(-gamepad1.left_stick_y);
                frontLeft.setPower(gamepad1.left_stick_y);
                frontRight.setPower(-gamepad1.left_stick_y);
            }

        }




    }}
