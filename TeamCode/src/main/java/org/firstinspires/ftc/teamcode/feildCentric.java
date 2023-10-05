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

public class feildCentric extends LinearOpMode {

     BHI260IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
double initYaw;
double adjustedYaw;

Orientation angles = new Orientation();

    public void runOpMode() throws InterruptedException {

       BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
parameters.mode = BNO055IMU.SensorMode.IMU;
parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BHI260IMU.class,"imu");
        imu.initialize();
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);

        initYaw = angles.firstAngle;


        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");


        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        //BACK LEFT SHOULD BE NEGATIVE to go fwd


        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {
            angles=imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            adjustedYaw = angles.firstAngle-initYaw;
            double zerodYaw = -initYaw+ angles.firstAngle;

            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;

            double turn = gamepad1.right_stick_x;


            double theta = Math.atan2(x,y)*180/Math.PI;//angles of gamepad
            double realTheta;
            realTheta = (360-zerodYaw)+theta;
            double power = Math.hypot(x,y);


            double sin = Math.sin((realTheta * (Math.PI/180)) - (Math.PI/4));
            double cos = Math.cos((realTheta * (Math.PI/180))-(Math.PI/4));

            double maxSinCos = Math.max(Math.abs(sin), Math.abs(cos));

            double leftFront = (power * cos /maxSinCos + turn);
            double rightFront = (power * sin / maxSinCos - turn);
            double leftBack = (power * sin / maxSinCos + turn);
            double rightBack = (power * cos /maxSinCos - turn);

            if((power + Math.abs(turn))>1){
                leftFront /= power+turn;
                rightFront /= power - turn;
                leftBack /= power+turn;
                rightBack /= power - turn;

            }
            frontLeft.setPower(leftFront);
            frontRight.setPower(rightFront);
            backLeft.setPower(-leftBack);
            backRight.setPower(rightBack);

        }
    }}