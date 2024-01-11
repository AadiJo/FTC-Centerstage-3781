package org.firstinspires.ftc.teamcode;
import com.acmerobotics.roadrunner.Math;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Servo Testing", group = "Testing")
public class ServoTester extends LinearOpMode {

    private void log(String caption, Object message){
        if (Global.LOG){
            telemetry.addData(caption, message);
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo = hardwareMap.get(Servo.class, "cassette");
        // CHANGE THIS TO THE NAME OF THE SERVO YOU WANT TO TEST

        float position = 0f;

        log("Status", "Initialized");
        log("WARNING", "Servo will jump to 0 position on start");

        waitForStart();
        servo.setPosition(0);
        while (!isStopRequested()) {


            telemetry.addLine("MOTOR TESTER");
            telemetry.addLine("");
            telemetry.addLine("EDIT LINE 21");
            telemetry.addLine("");
            telemetry.addLine("");
            telemetry.addLine("Currently testing motor: " + servo.getDeviceName());
            telemetry.addLine("");
            telemetry.addLine("Current servo position: " + servo.getPosition());
            telemetry.addLine("");
            telemetry.addLine("");

            telemetry.update();

            if (gamepad1.dpad_up) {
                servo.setPosition(Math.clamp(position += 0.05, 0, 1));
            }

            if (gamepad1.dpad_down) {
                servo.setPosition(Math.clamp(position -= 0.05, 0, 1));
            }

            }
}
}


