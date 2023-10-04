package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;//YESS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptExploringIMUOrientation;
import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name = "Encoder Test", group = "Pirhos")
public class EncoderDrive extends LinearOpMode{
public final double encoderTicks = 537.7;
public double targetTicks;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;

    public void setPowerToAllMotorsTo(double speed){
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);

    }

    public void runOpMode() throws InterruptedException {


         frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
         frontRight = hardwareMap.get(DcMotor.class, "frntRT");
         backLeft = hardwareMap.get(DcMotor.class, "bckLF");
         backRight = hardwareMap.get(DcMotor.class, "bckRT");


        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

waitForStart();
        while(opModeIsActive()){

encoder(1);

        }
    }
public void encoder(int fraction){
    backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        targetTicks = encoderTicks/fraction;
    backLeft.setTargetPosition((int)targetTicks);
    frontRight.setTargetPosition((int)targetTicks);
    frontLeft.setTargetPosition((int)targetTicks);
    backRight.setTargetPosition((int)targetTicks);
    setPowerToAllMotorsTo(.3);
    backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



}

    }
