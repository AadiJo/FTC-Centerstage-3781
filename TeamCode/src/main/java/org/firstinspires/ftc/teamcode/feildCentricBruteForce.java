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

@TeleOp(name = "FeildCentricBF", group = "Pirhos")
public class feildCentricBruteForce extends LinearOpMode {
    double error;
    BHI260IMU imu;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    double StartingAngle;
    double adjustedYaw;
private double degrees;
    Orientation angles = new Orientation();
    public void strafe(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }







    public void runOpMode() throws InterruptedException {

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;

        imu = hardwareMap.get(BHI260IMU.class,"imu");
        imu.initialize();
        angles = imu.getRobotOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES);

        StartingAngle = angles.firstAngle;


        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");




        //BACK LEFT SHOULD BE NEGATIVE to go fwd

        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {


            double x = gamepad1.left_stick_x;
            double y = gamepad1.left_stick_y;
            double JSangle = Math.atan2(x,y);

            if (JSangle <-80&& JSangle>-100)
                strafe(-.7);


if (JSangle > 80&& JSangle<100)
            strafe(.7);

            if(JSangle > 40 && JSangle < 50) {


                frontRight.setPower(.7);
                backLeft.setPower(.7);

            }
            if(JSangle < -40 && JSangle > -50) {


                backRight.setPower(.7);
                frontLeft.setPower(.7);

            }
            if(JSangle > 135 && JSangle < 155) {


                frontRight.setPower(.7);
                backLeft.setPower(.7);

            }
            if(JSangle < -135 && JSangle > -155) {


                backRight.setPower(.7);
                frontLeft.setPower(.7);

            }
        }

            }
    }