package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "Hang Controls")
public class PullTesting extends LinearOpMode {

    DcMotorEx armMotor;
    DcMotorEx pullMotor;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        pullMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        pullMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        double armMotorPower = 1;
        double pullMotorPower = 0.75;


        waitForStart();
        while (opModeIsActive()){

            if (isStopRequested()){
                return;
            }

            telemetry.addLine("Xbox Buttons");
            telemetry.addLine("");
            telemetry.addLine(" X           - Wind Hang Motor");
            telemetry.addLine(" B           - Unwind Hang Motor");
            telemetry.addLine(" Y           - Arm Forward");
            telemetry.addLine(" A           - Arm Backward");

            if (gamepad1.dpad_up){
                if (armMotorPower < 1){
                    armMotorPower = armMotorPower + 0.1;
                }

            }

            if (gamepad1.dpad_down){
                if (armMotorPower > 0.1){
                    armMotorPower = armMotorPower - 0.1;
                }

            }

            if (gamepad1.dpad_right){
                if (pullMotorPower < 1){
                    pullMotorPower = pullMotorPower + 0.1;
                }

            }

            if (gamepad1.dpad_left){
                if (pullMotorPower > 0.1){
                    pullMotorPower = pullMotorPower - 0.1;
                }

            }

            if (gamepad1.right_trigger > 0.2 && gamepad1.left_trigger > 0.2){
                armMotor.setPower(armMotorPower);
                pullMotor.setPower(pullMotorPower);
            }else if(gamepad1.right_bumper && gamepad1.left_bumper){
                armMotor.setPower(-armMotorPower);
                pullMotor.setPower(-pullMotorPower);
            }else{
                if (gamepad1.y){
                    armMotor.setPower(-armMotorPower);
                }else if (gamepad1.a){
                    armMotor.setPower(armMotorPower);
                }else{
                    armMotor.setPower(0);
                }

                if (gamepad1.b){
                    pullMotor.setPower(pullMotorPower);
                }else if (gamepad1.x){
                    pullMotor.setPower(-pullMotorPower);
                }else{
                    pullMotor.setPower(0);
                }
            }

            telemetry.addData("Arm Power", armMotorPower);
            telemetry.addData("Pull Power", pullMotorPower);
            telemetry.addData("Arm Current", armMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.addData("Pull Current", pullMotor.getCurrent(CurrentUnit.AMPS));
            telemetry.update();
        }






    }
}
