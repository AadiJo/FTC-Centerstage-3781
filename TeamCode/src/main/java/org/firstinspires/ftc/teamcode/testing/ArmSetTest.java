package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;

import java.util.Base64;

@Autonomous(name = "Arm Set Testing")
public class ArmSetTest extends LinearOpMode {

    DcMotorEx armMotor;
    Servo cassette;

    Encoder par0, par1, perp;

    void moveCassetteDown(){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() - 0.055 > 0){
                cassette.setPosition(cassette.getPosition() - 0.055);
                sleep(100);
            }else{
                cassette.setPosition(0);
                sleep(100);
            }
        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }

    }

    void moveCassetteUp(){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() + 0.055 < 1){
                cassette.setPosition(cassette.getPosition() + 0.055);
                sleep(100);
            }else{
                cassette.setPosition(1);
                sleep(100);
            }

        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }
    }

    void setArm(int position){
        double tolerance = 50;
        int par0Pos = par0.getPositionAndVelocity().position;
        int par1Pos = par1.getPositionAndVelocity().position;
        int perpPos = perp.getPositionAndVelocity().position;
        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();
        while ((Math.abs(position - armMotor.getCurrentPosition()) > tolerance) && time1.time() < 2.5) {

            // obtain the encoder position
            double encoderPosition = armMotor.getCurrentPosition();
            // calculate the error
            double error = position - encoderPosition;
            // set motor power proportional to the error
            armMotor.setPower(error);
            if (position > encoderPosition){
                moveCassetteUp();
            }else{
                moveCassetteDown();
            }

            if (((Math.abs(par0Pos - par0.getPositionAndVelocity().position) < 50 && Math.abs(par1Pos - par1.getPositionAndVelocity().position) < 50))){
                break;
            }

        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        par0 = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).par0;
        //par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"))); // leftEncoder
        par1 = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).par1;
        // par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntLF"))); // frntLF
        perp = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).perp;
        //perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));
        cassette = hardwareMap.servo.get("cassette");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        cassette.setPosition(1);
        int startPos = armMotor.getCurrentPosition();
        waitForStart();
        setArm(armMotor.getCurrentPosition() - 3000);
        setArm(startPos);

    }
}
