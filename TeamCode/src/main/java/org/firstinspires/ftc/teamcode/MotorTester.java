package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Math;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "Motor Testing", group = "Testing")
public class MotorTester extends LinearOpMode {

    private void log(String caption, Object message){
        if (Global.LOG){
            telemetry.addData(caption, message);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        DcMotorEx motor2 = hardwareMap.get(DcMotorEx.class, "armMotor");
        // CHANGE THIS TO THE NAME OF THE MOTOR YOU WANT TO TEST

        float power = 1f;

        log("Status", "Initialized");

        waitForStart();
        while (!isStopRequested()) {


            telemetry.addLine("MOTOR TESTER");
            telemetry.addLine("");
            telemetry.addLine("EDIT LINE 27");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("Currently testing motor: " + motor.getDeviceName());
            telemetry.addLine("");
            telemetry.addLine("Current power level: " + motor.getPower());
            telemetry.addLine("");
            telemetry.addLine("");

            log("Motor Ticks", motor.getCurrentPosition());
            log("Velocity (Ticks/Second)", motor.getVelocity());
            log("Motor Ticks", motor.getPower());
            telemetry.update();


            if (gamepad1.a) {
                motor.setPower(power);
                motor2.setPower(1);
            }else{
                motor.setPower(0);
                motor2.setPower(0);
            }

            if (gamepad1.y) {
                motor.setPower(-power);
                motor2.setPower(-1);
            }else{
                motor.setPower(0);
                motor2.setPower(0);
            }

            if (gamepad1.b) {
                motor.setPower(power);
            }else{
                motor.setPower(0);
            }

            if (gamepad1.dpad_up) {
                motor.setPower(Math.clamp(power += 0.05, 0, 1));
            }

            if (gamepad1.dpad_down) {
                motor.setPower(Math.clamp(power -= 0.05, 0, 1));
            }

            if (gamepad1.right_bumper){
                motor2.setPower(0.2);
            }

            else if (gamepad1.left_bumper){
                motor2.setPower(-0.2);
            }

            else{
                motor2.setPower(0);
            }

            }
}
}


