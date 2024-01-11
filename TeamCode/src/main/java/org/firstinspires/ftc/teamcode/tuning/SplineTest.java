package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.roadrunner.TankDrive;

public final class SplineTest extends LinearOpMode {
    private IMU imu;
    DcMotorEx leftFront, leftBack, rightFront, rightBack, armMotor;
    private void turn(double angle){
        // angle going from 0 90 180 - 90 0
        imu.resetYaw();
        if (angle < 180 && angle > 0){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < angle){
                leftFront.setPower(-0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(-0.5);
                rightBack.setPower(0.5);
            }
        }else if (angle > -180 && angle < 0){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > angle){
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(-0.5);
            }
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-71.86, 23.29, 0));

            leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
            leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
            rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
            rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");
            armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                        //.splineTo(new Vector2d(60, 0), Math.PI)
//                        .splineTo(new Vector2d(-36.50, -36.50), Math.toRadians(0))
                        .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y + 24))
                        .strafeToConstantHeading(new Vector2d(drive.pose.position.x + 30, drive.pose.position.y))
                        .build());

            turn(90);
            // check if left or right
            // LEFT
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 24))
                            .build()
            );

            //RIGHT
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y - 45))
                            .build()
            );
            // AFTER
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // Right in front of backdrop
                            .strafeToConstantHeading(new Vector2d(-36.36, 44.59))
                            .build()
            );
        }
    }
}
