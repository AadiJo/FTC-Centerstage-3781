package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class AutoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(-69.22, 4.9, Math.toRadians(2.49)));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(-64.99, 5.41), Math.toRadians(6.95))
                        .splineTo(new Vector2d(-63.14, -10.77), Math.toRadians(270.00))
                        .splineTo(new Vector2d(-61.18, -22.32), Math.toRadians(-74.05))
                        .splineTo(new Vector2d(-59.42, -34.07), Math.toRadians(-80.54))
                        .splineTo(new Vector2d(-47.26, -49.32), Math.toRadians(-51.43))
                        .splineTo(new Vector2d(-38.71, -54.27), Math.toRadians(-87.73))
                        .splineTo(new Vector2d(-38.81, -61.90), Math.toRadians(269.23))
                        .splineTo(new Vector2d(-30.67, -62.41), Math.toRadians(45.00))
                        .splineTo(new Vector2d(-21.08, -53.03), Math.toRadians(83.75))
                        .splineTo(new Vector2d(-21.08, -42.11), Math.toRadians(126.87))
                        .splineTo(new Vector2d(-31.70, -33.24), Math.toRadians(144.46))
                        .splineTo(new Vector2d(-37.47, -16.23), Math.toRadians(80.13))
                        .splineTo(new Vector2d(-37.06, -13.86), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-37.06, -3.04), Math.toRadians(84.91))
                        .splineTo(new Vector2d(-35.92, 12.01), Math.toRadians(92.94))
                        .splineTo(new Vector2d(-36.85, 32.83), Math.toRadians(87.77))
                        .splineTo(new Vector2d(-37.16, 48.50), Math.toRadians(88.67))
                        .build());



    }
}
