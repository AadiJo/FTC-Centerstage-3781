package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;//YESS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptExploringIMUOrientation;
import org.firstinspires.ftc.robotcontroller.internal.PermissionValidatorWrapper;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Encoder Test", group = "Pirhos")
public class EncoderDrive extends LinearOpMode{
public final double encoderTicks = 537.7;
public double targetTicks;
public final double cirucumferenceOfWheel = 3.78 * 2* Math.PI;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    BHI260IMU imu;
    Orientation angles = new Orientation();
int getPosition;

    public void setPowerToAllMotorsTo(double speed){
        backLeft.setPower(speed);
        frontRight.setPower(speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);

    }
    public void encoderDrive(double inches, double speed){

        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.getCurrentPosition();
        backRight.getCurrentPosition();
        frontRight.getCurrentPosition();
        backLeft.getCurrentPosition();
        targetTicks = encoderTicks*(inches/cirucumferenceOfWheel);
        backLeft.setTargetPosition((int)targetTicks);
        frontRight.setTargetPosition((int)targetTicks);
        frontLeft.setTargetPosition((int)targetTicks);
        backRight.setTargetPosition((int)targetTicks);

        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);



        setPowerToAllMotorsTo(speed);
        while(backLeft.isBusy()){

        }
        setPowerToAllMotorsTo(0);

    }

    public void strafeWithEncoder(double angle, double inches, double speed){
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        targetTicks = encoderTicks*(inches/cirucumferenceOfWheel);
        if (angle == 45){
            frontRight.setTargetPosition((int)targetTicks);
            backLeft.setTargetPosition((int)(targetTicks));
            backLeft.setPower(speed);
            frontRight.setPower(speed);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (angle == -45){
            frontLeft.setTargetPosition((int)targetTicks);
            backRight.setTargetPosition((int)(targetTicks));
            frontLeft.setPower(speed);
            backRight.setPower(speed);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (angle == -135){
            frontRight.setTargetPosition(-(int)targetTicks);
            backLeft.setTargetPosition(-(int)(targetTicks));
            backLeft.setPower(-speed);
            frontRight.setPower(-speed);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }
        if(angle == 135){
            frontLeft.setTargetPosition(-(int)targetTicks);
            backRight.setTargetPosition(-(int)(targetTicks));
            frontLeft.setPower(-speed);
            backRight.setPower(-speed);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        if (angle == 180){
            backLeft.setTargetPosition((int)targetTicks);
            frontRight.setTargetPosition((int)targetTicks);
            frontLeft.setTargetPosition((int)targetTicks);
            backRight.setTargetPosition((int)targetTicks);
            setPowerToAllMotorsTo(speed);
            backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        }


    }
    public void turn(int angle, double speed, char direction){
        if (direction == 'r'){
while (angles.firstAngle != angle){
    backLeft.setPower(speed);
    frontLeft.setPower(speed);
    backRight.setPower(-speed);
    frontRight.setPower(-speed);
}
            if (direction == 'l'){
                while (angles.firstAngle != angle){
                    backLeft.setPower(-speed);
                    frontLeft.setPower(-speed);
                    backRight.setPower(speed);
                    frontRight.setPower(speed);
                }
        }


    }}

    public void runOpMode() throws InterruptedException {


        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");
        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));


        //initialize imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        imu.initialize(parameters);
        imu.resetYaw();


        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

waitForStart();


while (opModeIsActive()){
    if(gamepad1.a){
    encoderDrive(12,.5);}
    if (gamepad1.b){
        strafeWithEncoder(45,12,.5);
    }
    if (gamepad1.y){
        encoderDrive(12,.7);
        encoderDrive(-12,.7);
    }
    if(gamepad1.x) {
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }


    telemetry.addData("backLeft", backLeft.getCurrentPosition());
    telemetry.addData("backRight", backRight.getCurrentPosition());

    telemetry.addData("frontLeft", frontLeft.getCurrentPosition());

    telemetry.addData("frontRight", frontRight.getCurrentPosition());
    telemetry.update();

}





    }}
