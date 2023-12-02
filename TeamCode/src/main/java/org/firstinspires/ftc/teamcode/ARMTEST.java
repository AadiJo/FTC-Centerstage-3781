package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "ARM TEST", group = "Current")
public class ARMTEST extends LinearOpMode {

    final double ticksPerDeg = 7.77777778;

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme
    public double getTickFromDeg(int deg){

        return ticksPerDeg*deg;
    }
    //    public void setArmPos(double ticks){
//        if(botArm.getCurrentPosition() < ticks){
//            while (botArm.getCurrentPosition() < ticks){
//            botArm.setPower(.7);
//        }}
//            else{
//                while (botArm.getCurrentPosition()>ticks){
//                    botArm.setPower(-.7);
//                }
//            }
//
//    }
    @Override
    public void runOpMode() throws InterruptedException {
        // constructor takes in frontLeft, frontRight, backLeft, backRight motors
        // IN THAT ORDER

        float droneRPM = 1620;

        Motor leftFront = new Motor(hardwareMap, "frntLF");
        Motor leftBack = new Motor(hardwareMap, "bckLF");;
        Motor rightBack = new Motor(hardwareMap, "bckRT");
        Motor rightFront = new Motor(hardwareMap, "frntRT");
        DcMotorEx droneMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        DcMotorEx botArm = hardwareMap.get(DcMotorEx.class, "droneMotor");
        botArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        botArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        {
            leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            droneMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        {
            leftBack.setInverted(true);
            rightBack.setInverted(true);
            rightFront.setInverted(true);
            droneMotor.setDirection(DcMotorEx.Direction.REVERSE);
        }

        MecanumDrive drive = new MecanumDrive(
                leftFront, rightFront, leftBack, rightBack
        );

        // initialize the IMU

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        IMU imu = (IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
        imu.resetYaw();


        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {
            telemetry.addLine(botArm.getCurrentPosition()+"");
            telemetry.update();

            // Driving the mecanum base takes 3 joystick parameters: leftX, leftY, rightX.
            // These are related to the left stick x value, left stick y value, and
            // right stick x value respectively. These values are passed in to represent the
            // strafing speed, the forward speed, and the turning speed of the robot frame
            // respectively from [-1, 1].

            // Below is a model for how field centric will drive when given the inputs
            // for (leftX, leftY, rightX). As you can see, for (0,1,0), it will travel forward
            // regardless of the heading. For (1,0,0) it will strafe right (ref to the 0 heading)
            // regardless of the heading.
            //
            //                   heading
            //                     /
            //            (0,1,0) /
            //               |   /
            //               |  /
            //            ___|_/_____
            //          /           /
            //         /           / ---------- (1,0,0)
            //        /__________ /

            // optional fifth parameter for squared inputs

            if (gamepad1.back){
                imu.resetYaw();
                gamepad1.rumble(1, 1, 200);
            }

            if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){
                droneMotor.setPower(droneRPM / 1620);
                gamepad2.rumble(1, 1, 200);
            }else{
                droneMotor.setPower(0);
            }

            if(gamepad1.a) {
                botArm.setPower(.6);

            }
            if(gamepad1.b){
                botArm.setPower(-.6);}

            botArm.setPower(.1);

            drive.driveFieldCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX() * 0.5,
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                    false
            );


        }
    }

}









