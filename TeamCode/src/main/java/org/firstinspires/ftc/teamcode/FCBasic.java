package org.firstinspires.ftc.teamcode;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "TeleOP Basic", group = "Testing")
public class FCBasic extends LinearOpMode {

    private void log(String caption, Object message){
        if (Global.LOG){
            telemetry.addData(caption, message);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Motor leftFront = new Motor(hardwareMap, "frntLF");
        Motor leftBack = new Motor(hardwareMap, "bckLF");
        Motor rightBack = new Motor(hardwareMap, "bckRT");
        Motor rightFront = new Motor(hardwareMap, "frntRT");


        {
            leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        }

        {
            leftBack.setInverted(true);
            leftBack.setInverted(true);
        }

        MecanumDrive drive = new MecanumDrive(
                leftFront, rightFront, leftBack, rightBack
        );

        // initialize the IMU
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        IMU imu = (IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
        imu.resetYaw();

        // extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        log("Status", "Initialized");

        waitForStart();
        while (!isStopRequested()) {

            drive.driveFieldCentric(
                    driverOp.getLeftX(),
                    driverOp.getLeftY(),
                    driverOp.getRightX() * 0.5,
                    imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                    false
            );

            telemetry.addLine("FIELD CENTRIC DRIVE");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("Controls:");
            telemetry.addLine("");
            telemetry.addLine("BACK to reset gyro");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("");

            log("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            log("Pitch", imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES));
            log("Roll", imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES));
            telemetry.update();


            if (gamepad1.back || imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) == 0.0) {
                imu.initialize(parameters);
                imu.resetYaw();
                gamepad1.rumble(1, 1, 200);
            }

            }
}
}


