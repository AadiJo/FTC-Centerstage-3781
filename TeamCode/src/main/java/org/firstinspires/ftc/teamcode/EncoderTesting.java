package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "encoder test", group = "Current")

public class EncoderTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Encoder leftEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).par0;
        waitForStart();
        int prePose = leftEncoder.getPositionAndVelocity().velocity;
        while (opModeIsActive()){
            telemetry.addLine("left encoder:  "+ Math.abs(prePose - leftEncoder.getPositionAndVelocity().position));
            telemetry.update();
        }
    }
}
