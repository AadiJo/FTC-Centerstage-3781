package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
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

@TeleOp(name = "FeildCentric", group = "Pirhos")
public class FieldCentric extends LinearOpMode {
    double error;
    BHI260IMU imu;

    private double botSpeedL1 = 0.30f;
    private double botSpeedL2 = 0.50f;
    private double botSpeedL3 = 0.80f;
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    double StartingAngle;
    double adjustedYaw;
    private double degrees;
    Orientation angles = new Orientation();

    public void moveBotX(double speed){
        //strafing
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
    }

    public void moveBotY(double speed){
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }

    public void moveBot(double xSpeed, double ySpeed){
        frontLeft.setPower(ySpeed + xSpeed + xSpeed);
        frontRight.setPower(ySpeed - xSpeed - xSpeed);
        backLeft.setPower(ySpeed - xSpeed + xSpeed);
        backRight.setPower(ySpeed + xSpeed - xSpeed);
    }

    // BOT ANGLE GOES COUNTERCLOCKWISE 0 90 180 -90 0

    // JOYSTICK ANGLE READINGS GO COUNTERCLOCKWISE 180 -90 0 90 180

    // CONVERTING JOYSTICK ANGLE TO SAME DEFINITION AS BOT

    private double convertJoystickAngle(double joystickAngle){
        double stickAngle = 0;

        if (joystickAngle != -180) {
            stickAngle = joystickAngle - 180;
        }
        if (stickAngle < -180){
            stickAngle = stickAngle + 360;
        }

        if (stickAngle > 180){
            stickAngle = stickAngle - 360;
        }

        return stickAngle;
    }

    public double calcBotXSpeed(double speed, double angle){

        double botXSpeed = 0;

        botXSpeed = speed * Math.cos(angle);
        //smooth out the speed
        if(botXSpeed > 0.05) {
            return botXSpeed;
        }else{
            return 0;
        }
    }

    private double calcBotXSpeedNew(double stickX, double stickY, double botHeading){

        return (stickX * Math.cos(-botHeading) - stickY * Math.sin(-botHeading));

    }

    private double calcBotYSpeed(double speed, double angle){
        double botYSpeed = 0;

        botYSpeed = speed * Math.sin(angle);

        if(botYSpeed > 0.05) {
            return botYSpeed;
        }else{
            return 0;
        }
    }

    private double calcBotYSpeedNew(double stickX, double stickY, double botHeading){

        return (stickX * Math.sin(-botHeading) + stickY * Math.cos(-botHeading));

    }

    private double joystickVectorMagnitude(double x, double y){
        return Math.sqrt(Math.pow(x,2) + Math.pow(y,2));
    }

    private double calcFCBotAngle(double joystickAngle, double botHeading){
        double botFCAngle = 0;

        //botFCAngle = joystickAngle - botHeading;

        if (botHeading < -90){
            botFCAngle = joystickAngle + botHeading;
        }else{
            botFCAngle = joystickAngle - botHeading;
        }

        return botFCAngle;
    }

    private double radiansToDegrees(double radians){
        return (radians * 180) / Math.PI;
    }

    public void runOpMode() throws InterruptedException {

        //set parameters
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));


        //initialize imu
        imu = hardwareMap.get(BHI260IMU.class,"imu");
        imu.initialize(parameters);
        imu.resetYaw();

        //initialize motors
        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");

        //BACK LEFT SHOULD BE NEGATIVE to go fwd
        // NEGATIVE TO GO FORWARD
         backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
         frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        // backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {

            //read joy stick input
            double joyStickX = gamepad1.left_stick_x;
            double joyStickY = -gamepad1.left_stick_y;

            // Smoothing user input

            if (Math.abs(joyStickX) < 0.05f){
                joyStickX = 0;
            }

            if (Math.abs(joyStickY) < 0.05f){
                joyStickY = 0;
            }

            telemetry.addData("x",joyStickX);
            telemetry.addData("y",joyStickY);

            // TODO Check for divide by zero

            // Read joystick angle input
            double JoystickAngle = radiansToDegrees(Math.atan2(joyStickY,joyStickX));

            //converting it to match bot imu convention
            JoystickAngle = convertJoystickAngle(JoystickAngle);

            telemetry.addData("JS angle", JoystickAngle);

            // FC magnitude for bot = joystick magnitude
            //double joystickVM = joystickVectorMagnitude(joyStickX,joyStickY);
            //telemetry.addData("FC Magnitude", joystickVM);

            //read imu bot heading
            double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            telemetry.addData("Bot Heading", botHeading);

            // FC angle for bot = calcFCBotAngle(JoystickAngle, botHeading)
            // double FCBotAngle = calcFCBotAngle(JoystickAngle, botHeading);
            // telemetry.addData("FC angle", FCBotAngle);

            // moveBotY(calcBotYSpeed(joystickVM, FCBotAngle));
            // moveBotX(calcBotXSpeed(joystickVM, FCBotAngle));

            //double botXSpeed = calcBotXSpeed(joystickVM, FCBotAngle);
            //double botYSpeed = calcBotYSpeed(joystickVM, FCBotAngle);

            double botXSpeed = calcBotXSpeedNew(joyStickX, joyStickY, botHeading);
            double botYSpeed = calcBotYSpeedNew(joyStickX, joyStickY, botHeading);

            telemetry.addData("Bot X Speed", botXSpeed);
            telemetry.addData("Bot Y Speed", botYSpeed);

            moveBot(botXSpeed, botYSpeed);

            telemetry.update();
        }

    }
}