package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Disabled
public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-0.14, -48.00, Math.toRadians(0.00));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .splineTo(new Vector2d(23.29, -19.31), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-0.57, -2.27), Math.toRadians(180.00))
                        .splineTo(new Vector2d(-31.95, 21.73), Math.toRadians(90.00))
                        .splineTo(new Vector2d(0.14, 48.71), Math.toRadians(0.00))
                        .splineTo(new Vector2d(19.31, -5.96), Math.toRadians(270.00))
                        .splineTo(new Vector2d(-1.28, -47.86), Math.toRadians(177.71))
                        .waitSeconds(0.1)
                        .turn(Math.toRadians(180), new TurnConstraints(2, -2, 2))
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
