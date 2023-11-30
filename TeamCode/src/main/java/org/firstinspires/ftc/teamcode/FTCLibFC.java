package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOP", group = "Current")
public class FTCLibFC extends LinearOpMode {

    // This variable determines whether the following program
    // uses field-centric or robot-centric driving styles. The
    // differences between them can be read here in the docs:
    // https://docs.ftclib.org/ftclib/features/drivebases#control-scheme

    public static class VARS{
        public float droneRPM = 1053;
        public double dPadX;
        public double dPadY;
        public boolean dpPressed;
        public double dpStartSpeed = 0.2;
        public double dpSpeed = 0.2;
        public double dpMaxSpeed = 1;

        public double ARM_TICKS_PER_REV = 384.5;
    }

    public static VARS VARS= new VARS();

    @Override
    public void runOpMode() throws InterruptedException {
        Motor leftFront = new Motor(hardwareMap, "frntLF");
        Motor leftBack = new Motor(hardwareMap, "bckLF");;
        Motor rightBack = new Motor(hardwareMap, "bckRT");
        Motor rightFront = new Motor(hardwareMap, "frntRT");

        Servo clawL = hardwareMap.servo.get("clawL");
        Servo clawR = hardwareMap.servo.get("clawR");
        Servo cassette = hardwareMap.servo.get("cassette");

        DcMotorEx droneMotor = hardwareMap.get(DcMotorEx.class,"droneMotor");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        double cassetteMoveIncrement = 0;

        {
            leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            droneMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        {
            leftBack.setInverted(true);
            rightBack.setInverted(true);
            rightFront.setInverted(true);
            armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        MecanumDrive drive = new MecanumDrive(
                leftFront, rightFront, leftBack, rightBack
        );

        // initialize the IMU

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        IMU imu = (IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
        imu.resetYaw();


        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        waitForStart();

        while (!isStopRequested()) {

            telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            telemetry.update();

            {

                if (gamepad1.back) {
                    imu.initialize(parameters);
                    imu.resetYaw();
                    gamepad1.rumble(1, 1, 200);
                }

                if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){
                    droneMotor.setPower(-VARS.droneRPM / 1620);
                    gamepad2.rumble(1, 1, 200);
                }else{
                    droneMotor.setPower(0);
                }

                if (gamepad2.left_bumper) {
                    clawR.setPosition(0.3);
                    clawL.setPosition(0.85);//1


                }
                if (gamepad2.right_bumper) {
                    clawR.setPosition(0.5);
                    clawL.setPosition(0.4);//0
                }

                if (gamepad2.a) {
                    // DOWN
                    // adjust servo
                    cassette.setPosition(cassette.getPosition() + cassetteMoveIncrement);
                    armMotor.setPower(-1);
                }else if (gamepad2.y) {
                    // UP
                    // adjust servo
                    cassette.setPosition(cassette.getPosition() - cassetteMoveIncrement);
                    armMotor.setPower(1);
                }else{
                    armMotor.setPower(0);
                }

                if (gamepad2.x){
                    // switch to flat position
                }else if (gamepad2.b){
                    // switch to parallel to backdrop position
                }

            }

            {
                if (gamepad1.dpad_left) {
                    VARS.dpPressed = true;
                    VARS.dPadX = -VARS.dpSpeed;
                } else if (gamepad1.dpad_right) {
                    VARS.dpPressed = true;
                    VARS.dPadX = VARS.dpSpeed;

                } else {
                    VARS.dpPressed = false;
                    VARS.dPadX = 0;
                }

                if (gamepad1.dpad_down) {
                    VARS.dpPressed = true;
                    VARS.dPadY = -VARS.dpSpeed;
                } else if (gamepad1.dpad_up) {
                    VARS.dpPressed = true;
                    VARS.dPadY = VARS.dpSpeed;

                } else {
                    VARS.dpPressed = false;
                    VARS.dPadY = 0;
                }

                if (VARS.dpPressed) {
                    if (VARS.dpSpeed < VARS.dpMaxSpeed) {
                        VARS.dpSpeed += 0.005;
                    }
                } else {
                    VARS.dpSpeed = VARS.dpStartSpeed;
                }
            }

            // constructor takes in frontLeft, frontRight, backLeft, backRight motors
            // IN THAT ORDER

            if (VARS.dpPressed){
                drive.driveFieldCentric(
                        VARS.dPadX,
                        VARS.dPadY,
                        driverOp.getRightX() * 0.5,
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                        false
                );
            }
            else{
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

}
