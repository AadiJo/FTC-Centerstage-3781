package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name = "Cassette Sync Test")
public class CassetteSyncTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        Servo cassette = hardwareMap.get(Servo.class, "cassette");

        double SERVO_MIN_POSITION = 0.0;
        double SERVO_MAX_POSITION = 1.0;
        double SERVO_RANGE = SERVO_MAX_POSITION - SERVO_MIN_POSITION;
        double TICKS_PER_REV = 384.5 * 13.7 * 28;
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (!isStopRequested()){
            int armPos = arm.getCurrentPosition();

            double cassettePos = (double) armPos / TICKS_PER_REV;
            //cassettePos = Math.min(Math.max(cassettePos, SERVO_MAX_POSITION), SERVO_MAX_POSITION);

            cassette.setPosition(cassettePos);

            if (gamepad1.y){
                arm.setPower(-1);
            }else if (gamepad1.a){
                arm.setPower(1);
            }else{
                arm.setPower(0);
            }
        }

    }
}