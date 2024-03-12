package org.firstinspires.ftc.teamcode.tuning;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;
@Autonomous(name = "Spline Test")
public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(-40, -60, Math.toRadians(90.00));
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
//                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
//                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .strafeTo(new Vector2d(-60, -22))
                        .strafeTo(new Vector2d(-40, -22))
                        .strafeTo(new Vector2d(-40, -31.5))
                        .strafeTo(new Vector2d(-40, -40), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-40, 20))
                        .setTangent(Math.toRadians(180))
                        .splineToLinearHeading(new Pose2d(-47, -11.5, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 50))
                        .splineToConstantHeading(new Vector2d(28, -20), Math.toRadians(0))
                        .splineToConstantHeading(new Vector2d(40, -45), Math.toRadians(0))
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
