package org.firstinspires.ftc.teamcode;

// qualcomm

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

// FirstInspires
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

// OpenCV

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

// OpenFTC
import org.openftc.easyopencv.*;

public class Auto  extends LinearOpMode {
    double error;
    BHI260IMU imu;
    private Servo clawR;
    private Servo clawL;
    private CRServo arm;
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

    private void turn(double angle){
        // angle going from 0 90 180 - 90 0
        if (angle < 180 && angle > 0){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < angle){
                frontLeft.setPower(-0.5);
                frontRight.setPower(0.5);
                backLeft.setPower(-0.5);
                backRight.setPower(0.5);
            }
        }else if (angle > -180 && angle < 0){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > angle){
                frontLeft.setPower(0.5);
                frontRight.setPower(-0.5);
                backLeft.setPower(0.5);
                backRight.setPower(-0.5);
            }

        }

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

    public double calcBotXSpeedNew(double stickX, double stickY, double botHeading){

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

    public double calcBotYSpeedNew(double stickX, double stickY, double botHeading){

        return (stickX * Math.sin(-botHeading) + stickY * Math.cos(-botHeading));

    }

    public void moveBot(double xSpeed, double ySpeed){
        frontLeft.setPower(ySpeed + xSpeed);
        frontRight.setPower(ySpeed - xSpeed);
        backLeft.setPower(ySpeed - xSpeed);
        backRight.setPower(ySpeed + xSpeed);
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
        clawL = hardwareMap.servo.get("clawL");
        clawR = hardwareMap.servo.get("clawR");
        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");
        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        arm = hardwareMap.crservo.get("arm");


        //BACK LEFT SHOULD BE NEGATIVE to go fwd hi
        // NEGATIVE TO GO FORWARD
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // backRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        ElapsedTime time = new ElapsedTime();
        waitForStart();
        while (opModeIsActive()) {

            //read joy stick input
            double joyStickX = gamepad1.left_stick_x;
            double joyStickY = -gamepad1.left_stick_y;

            double rightJoyStickX = gamepad1.right_stick_x;
            double rightJoyStickY = -gamepad1.right_stick_y;

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
            double RightJoystickAngle = radiansToDegrees(Math.atan2(rightJoyStickY,rightJoyStickX));

            //converting it to match bot imu convention
            JoystickAngle = convertJoystickAngle(JoystickAngle);
            RightJoystickAngle = convertJoystickAngle(RightJoystickAngle);


            telemetry.addData("Left JS angle", JoystickAngle);
            telemetry.addData("Right JS angle", RightJoystickAngle);

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
            if(gamepad1.right_stick_x > 0){

                backLeft.setPower(gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x);
                backRight.setPower(-gamepad1.right_stick_x);
                frontRight.setPower(-gamepad1.right_stick_x);
            }
            if(gamepad1.right_stick_x < 0){

                backRight.setPower(-gamepad1.right_stick_x);
                frontRight.setPower(-gamepad1.right_stick_x);
                frontLeft.setPower(gamepad1.right_stick_x);
                backLeft.setPower(gamepad1.right_stick_x);

            }
            if (gamepad1.left_bumper){
                clawR.setPosition(0);
                clawL.setPosition(1);


            }
            if (gamepad1.right_bumper){
                clawR.setPosition(1);
                clawL.setPosition(0);
            }

            while (gamepad1.a)
            {

                arm.setPower(-1);
            }
            while(gamepad1.b){
                arm.setPower(1);
            }
            arm.setPower(0);
            telemetry.addData("backLeft", backLeft.getCurrentPosition());
            telemetry.addData("backRight", backRight.getCurrentPosition());

            telemetry.addData("frontLeft", frontLeft.getCurrentPosition());

            telemetry.addData("frontRight", frontRight.getCurrentPosition());
            telemetry.update();
        }

    }

    private class CameraAuto extends LinearOpMode {
        OpenCvWebcam webcam = null;
        double targetTicks;
        CRServo arm;
        Servo clawR;
        Servo clawL;

        private DcMotor frontLeft;
        private DcMotor frontRight;
        private DcMotor backLeft;
        private DcMotor backRight;
        DcMotor rightEncoder;
        DcMotor midEncoder;
        DcMotor leftEncoder;
        double ticksPerRevODO = 2048;
        double circumOfODO = 1.89;
        public void setPowerToAllMotorsTo(double speed){

            backLeft.setPower(speed);
            backRight.setPower(speed);
            frontRight.setPower(speed);
            frontLeft.setPower(speed);
        }
        public double getTargetTicks(double inches){
            return (inches / circumOfODO)*ticksPerRevODO/2;}
        @Override
        public void runOpMode() throws InterruptedException {
            ElapsedTime time = new ElapsedTime();

            //WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam_1");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);

            backLeft = hardwareMap.get(DcMotor.class, "bckLF");
            backRight = hardwareMap.get(DcMotor.class, "bckRT");
            frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
            frontRight = hardwareMap.get(DcMotor.class, "frntRT");
            leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
            rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
            midEncoder = hardwareMap.get(DcMotor.class, "frontEncoder");
            arm = hardwareMap.crservo.get("arm");
            clawR = hardwareMap.servo.get("clawR");
            clawL = hardwareMap.servo.get("clawL");
            backRight.setDirection(DcMotorSimple.Direction.FORWARD);
            frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
            frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

            leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            midEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            examplePipeline pipeline = new examplePipeline();

            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
                @Override
                public void onOpened() {
                    webcam.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);
                }
                public void onError(int n){

                }


            });
            clawR.setPosition(1);
            clawL.setPosition(0);
            telemetry.addLine("left "+pipeline.leftavgfinal);
            telemetry.addLine("right "+pipeline.rightavgfinal);
            telemetry.addLine("middle"+pipeline.midavgfinal);
            telemetry.update();
            waitForStart();
            time.reset();
            while (time.seconds() < 2){
                setPowerToAllMotorsTo(.2);
            }
            setPowerToAllMotorsTo(0);

            leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//left movement is positive
            midEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            if(pipeline.position == 1) {
                telemetry.addLine("left");
                telemetry.update();}

            else if (pipeline.position == 3){
                telemetry.addLine("Right");
                telemetry.update();
            }

            else if(pipeline.position == 2) {
                telemetry.addLine("middle");
                telemetry.update();
                time.reset();
                while(time.seconds()<3){
                    setPowerToAllMotorsTo(.4);
                }
                time.reset();
                while(time.seconds()<2){
                    setPowerToAllMotorsTo(-.4);
                }
                setPowerToAllMotorsTo(0);
                time.reset();
                while(time.seconds() < 2.8){
                    arm.setPower(1);
                }
                arm.setPower(0);
                clawR.setPosition(0);
                clawL.setPosition(1);


            }
            telemetry.addLine(""+pipeline.leftavgfinal);
            telemetry.addLine(""+pipeline.rightavgfinal);
            telemetry.addLine(""+pipeline.midavgfinal);

            telemetry.update();

            time.reset();
            while (!gamepad1.a){}







        }
    }
//
//private class LeftRightCenter {
//        if
//    }
}