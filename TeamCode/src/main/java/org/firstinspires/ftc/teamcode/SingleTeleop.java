package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.List;
import java.util.concurrent.TimeUnit;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp(name = "TeleOP using April Tags for One person", group = "Current")
public class SingleTeleop extends LinearOpMode {
    //level 3 is 9 inches (range not des dist)
    // level 1 is 13.5 inches
    //level 2 is 10 inches
    Motor leftFront, leftBack, rightBack, rightFront;

    double DESIRED_DISTANCE = 10.5; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.02  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = .65;  //.5 //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE= .7;  //.5 //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.7;   //  Clip the turn speed to this max value (adjust for your robot)

    private DcMotor leftFrontDrive   = null;  //  Used to control the left front drive wheel
    private DcMotor rightFrontDrive  = null;  //  Used to control the right front drive wheel
    private DcMotor leftBackDrive    = null;  //  Used to control the left back drive wheel
    private DcMotor rightBackDrive   = null;  //  Used to control the right back drive wheel

    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;


    public static class VARS{
        public double ticksPerRevDrone = 103.8;
        public double maxTicksPerSec = 1620 /60 * ticksPerRevDrone;
        public double droneRPM = .75 * 1620;

        double Kp = PIDConstants.Kp;
        double Ki = PIDConstants.Ki;
        double Kd = PIDConstants.Kd;

        double lastError = 0;
        double integralSum;
//        public double lastError = 0.009;

        ElapsedTime PIDTimer = new ElapsedTime();

        public boolean isOpen = true;
        public double dPadX;
        public double dPadY;
        public boolean dpPressed;
        public double dpStartSpeed = 0.2;
        public double dpSpeed = 0.2;
        public double dpMaxSpeed = 1;

        public boolean intake = false;

        public double ARM_TICKS_PER_REV = 384.5;
        public double ARM_START_POS;
        // RPM / 60 -> RPS / 1000 -> RPMS x 11.29 (degrees per rev)
        // 1 rotation is 270 degrees
        public double cassetteMoveIncrement = 0.05; //0.0818525/360 * CST_UNIT_PER_DEG;
        public double CST_UPPER_BOUND = 0;
        public double CST_LOWER_BOUND = 1;
        public double CST_INIT_POS = 0.7;

        double t0 = 0;// time in milliseconds
        double t1 = 0;
        double elapsedTime = 0;

        public int ARM_BD_L1_POS = Math.abs(-5477 + 65);
        public int ARM_BD_L2_POS = Math.abs(-4890 + 65);
        public int ARM_BD_L3_POS = Math.abs(-4303 + 65);
        public double CST_BD_L1_POS = 0.1;
        public double CST_BD_L2_POS = 0.1;
        public double CST_BD_L3_POS = 0.2;
        public boolean CST_DOWN = false;
        public boolean CST_UP = false;

        public int ARM_HALF_POS = Math.abs(4337 - 1425);

        int aprilTagDetId = 0;
    }

    public static VARS VARS = new VARS();



    private void log(String caption, Object message){
        if (true){
            telemetry.addData(caption, message);
        }

    }
    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        // Send powers to the wheels.
        leftFront.set(leftFrontPower);
        rightFront.set(-rightFrontPower);
        leftBack.set(leftBackPower);
        rightBack.set(-rightBackPower);
    }
    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.

        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

    }
    private void setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                //sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }


    void powerCassette(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            cassette.setPosition(cassette.getPosition());
        }else{
            cassette.setPosition(1);
        }
    }

    void moveCassetteDown(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() - 0.035 > VARS.CST_UPPER_BOUND){
                cassette.setPosition(cassette.getPosition() - 0.035);
                sleep(100);
            }else{
                cassette.setPosition(VARS.CST_UPPER_BOUND);
                sleep(100);
            }
        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }

    }

    void moveCassetteUp(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() + 0.035 < VARS.CST_LOWER_BOUND){
                cassette.setPosition(cassette.getPosition() + 0.035);
                sleep(100);
            }else{
                cassette.setPosition(VARS.CST_LOWER_BOUND);
                sleep(100);
            }

        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }
    }

    private void setArmPos(int position, DcMotorEx armMotor, Servo cassette, Encoder par0, Encoder par1, Encoder perp){
        sleep(50);
        double tolerance = 300;
        int par0Pos = par0.getPositionAndVelocity().position;
        int par1Pos = par1.getPositionAndVelocity().position;
        int perpPos = perp.getPositionAndVelocity().position;
        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();
        while (armMotor.getCurrentPosition() != position && Math.abs(position - armMotor.getCurrentPosition()) > tolerance) {

            // obtain the encoder position
            double encoderPosition = armMotor.getCurrentPosition();
            // calculate the error
            double error = position - encoderPosition;
            // set motor power proportional to the error
            armMotor.setPower(error);
            if (position > encoderPosition){
                moveCassetteUp(cassette);
            }else{
                moveCassetteDown(cassette);
            }

        }
        sleep(50);
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public double PIDControl(double reference, double state) {
        VARS.integralSum = 0;
        double error = angleWrap(reference - state);
        telemetry.addData("Error: ", error);
        VARS.integralSum += error * VARS.PIDTimer.seconds();
        double derivative = (error - VARS.lastError) / (VARS.PIDTimer.seconds());
        VARS.lastError = error;
        VARS.PIDTimer.reset();
        return (error * VARS.Kp) + (derivative * VARS.Kd) + (VARS.integralSum * VARS.Ki);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double targetHeading = Math.toRadians(0);
        double tolerance = 0.001;
        double multiplier = 0.7;

        double angleRateOfChange = 0.0;
        double lastAngle = 0.0;

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int n) {

            }

        });

        leftFront = new Motor(hardwareMap, "frntLF");
        leftBack = new Motor(hardwareMap, "bckLF");
        rightBack = new Motor(hardwareMap, "bckRT");
        rightFront = new Motor(hardwareMap, "frntRT");
        Encoder leftEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).par0;
        //par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"))); // leftEncoder
        Encoder rightEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).par1;
        // par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntLF"))); // frntLF
        Encoder frontEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).perp;
        //perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));OverflowEncoder frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));

        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive_           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;


        initAprilTag();

        setManualExposure(6, 250);

        Servo claw = hardwareMap.servo.get("claw");
        Servo cassette = hardwareMap.servo.get("cassette");
        CRServo flicker = hardwareMap.crservo.get("drone");
        Servo door = hardwareMap.servo.get("door");
        CRServo intake_L = hardwareMap.crservo.get("intakeL");
        CRServo intake_R = hardwareMap.crservo.get("intakeR");

        DcMotorEx pullMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        DcMotorEx droneMotor = hardwareMap.get(DcMotorEx.class,"droneMotor");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        {
            leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            droneMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        {
            leftBack.setInverted(true);
            rightBack.setInverted(true);
            rightFront.setInverted(true);
            pullMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftFront.setInverted(true);
            // cassette.setDirection(Servo.Direction.REVERSE);
            // armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        MecanumDrive drive = new MecanumDrive(
                leftFront, rightFront, leftBack, rightBack
        );


        ;

        double loopTime = 0.0;

        // initialize the IMU

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        IMU imu = (IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
        imu.resetYaw();
        ElapsedTime IMU_TIMER = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        VARS.ARM_START_POS = armMotor.getCurrentPosition();
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                -PIDControl(targetHeading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                false
        );

        waitForStart();
        IMU_TIMER.reset();
        cassette.setPosition(1);
        while (!isStopRequested()) {
            double loop = System.nanoTime();

            targetFound = false;
            desiredTag  = null;

//            if(gamepad1.a)
//                DESIRED_DISTANCE = 5.0;
//            if(gamepad1.y)
//                DESIRED_DISTANCE = 7;
//            if(gamepad1.b)
//                DESIRED_DISTANCE = 10.5;

            if(gamepad1.square) {
                DESIRED_TAG_ID = 4;
                telemetry.clearAll();
                telemetry.addLine("Red: 4");
                telemetry.update();
            }

            if(gamepad1.triangle) {
                DESIRED_TAG_ID = 5;

                telemetry.clearAll();
                telemetry.addLine(" Red: 5");
                telemetry.update();
            }
            if(gamepad1.circle) {
                DESIRED_TAG_ID = 6;

                telemetry.clearAll();
                telemetry.addLine("Red: Circle");
                telemetry.update();

            }

            if(gamepad1.dpad_left) {
                DESIRED_TAG_ID = 1;
                telemetry.clearAll();
                telemetry.addLine("Blue 1");
                telemetry.update();
            }

            if(gamepad1.dpad_up) {
                DESIRED_TAG_ID = 2;

                telemetry.clearAll();
                telemetry.addLine("Blue 2");
                telemetry.update();
            }
            if(gamepad1.dpad_right) {
                DESIRED_TAG_ID = 3;

                telemetry.clearAll();
                telemetry.addLine("Blue 3");
                telemetry.addLine("Blue 3");
                telemetry.update();

            }



            // Step through the list of detected tags and look for a matching tag
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((detection.id == DESIRED_TAG_ID)) {//   dtag <0??
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                        telemetry.addData("Skipping", "Tag ID %d is not desired", detection.id);
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                    telemetry.addData("Unknown", "Tag ID %d is not in TagLibrary", detection.id);
                }
            }

            // Tell the driver what we see, and what to do.
            if (targetFound) {
                telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                telemetry.update();
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            // If Left Bumper is being pressed, AND we have found the desired target, Drive to target Automatically .
            if (gamepad1.left_bumper && targetFound) {

                // Determine heading, range and Yaw (tag image rotation) error so we can use them to control the robot automatically.
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive_  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                telemetry.addLine("Going to AprilTag");

                telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive_, strafe, turn);
                telemetry.update();
                targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                moveRobot(drive_, strafe, turn);
            }
            else{
                drive_  = 0;
                strafe = 0;
                turn   = 0;

                telemetry.update();
            }
//            else {
//
//                // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
//                drive_  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
//                strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
//                turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
//                telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
//            }
            telemetry.update();

            // Apply desired axes motions to the drivetrain.


            sleep(10);

            if(!gamepad1.left_bumper) {
                if (driverOp.getRightX() != 0 || (Math.abs(VARS.lastError) <= tolerance) || gamepad1.b) {
                    PIDControl(targetHeading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                    drive.driveFieldCentric(
                            driverOp.getLeftX(),
                            driverOp.getLeftY(),
                            driverOp.getRightX() * 0.55,
                            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                            false
                    );
                    // movement of inertia = 0.0951078743
                    if (imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate > 0) {
                        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) + (0.0981078743 * (imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate * imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));

                    } else {
                        targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) - (0.0981078743 * (imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate * imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
                    }
                } else { // if no input from right joystick
                    drive.driveFieldCentric(
                            driverOp.getLeftX(),
                            driverOp.getLeftY(),
                            -PIDControl(targetHeading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), // negative because FTCLib input is inverted
                            imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                            false
                    );
                }
            }
            telemetry.addData("hz", 1000000000  / (loop - loopTime));
            telemetry.addData("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            log("Cassette", cassette.getPosition());
//            log("Arm Ticks", armMotor.getCurrentPosition());
//            log("Cassette Pos", cassette.getPosition());
//            log("Par 0", leftEncoder.getPositionAndVelocity().position);
//            log("Par 1", rightEncoder.getPositionAndVelocity().position);
//            log("Perp", frontEncoder.getPositionAndVelocity().position);

            loopTime = loop;
            //telemetry.addData("Angular Velocity", imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate + " degrees per second");
            //telemetry.addData("Last PID Error", VARS.lastError);
            telemetry.update();

            {

                if (gamepad1.back || imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) == 0.0) {
                    imu.initialize(parameters);
                    imu.resetYaw();
                    targetHeading = Math.toRadians(0);
                    gamepad1.rumble(1, 1, 200);
                }

                if (gamepad1.left_trigger > 0.5 && gamepad1.right_trigger > 0.5){
                    leftFront.set(0);
                    leftBack.set(0);
                    rightBack.set(0);
                    rightFront.set(0);
                    //droneMotor.setPower(-VARS.droneRPM / 1620);
                    droneMotor.setVelocity(-0.85 * VARS.maxTicksPerSec);
                    gamepad1.rumble(1, 1, 200);
                    telemetry.addLine("Drone Percent:" + droneMotor.getVelocity()/ VARS.maxTicksPerSec+"");
                    telemetry.update();
                    sleep(1000);
                    flicker.setPower(-1);
                    sleep(200);

                    //droneMotor.setPower(0);
                }
                else{
                    droneMotor.setPower(0);
                    droneMotor.setVelocity(0);
                    flicker.setPower(0);

                }
            }

            if (gamepad1.left_bumper) {
                claw.setPosition(0);

            }

            if (gamepad1.right_bumper){
                claw.setPosition(1);
            }

            if (gamepad1.a){
                multiplier = 0.35;
            }else{
                multiplier = 0.7;
            }

            if (gamepad1.left_stick_button){


                if (gamepad1.x){
                    // FIRST
                    //cassette.setPosition(VARS.CST_BD_L1_POS);
                    setArmPos((int) VARS.ARM_START_POS - VARS.ARM_BD_L1_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                    // arm ticks = -5477 start = -65

                }

                if (gamepad1.y){
                    // SECOND
                    //cassette.setPosition(VARS.CST_BD_L2_POS);
                    setArmPos((int) VARS.ARM_START_POS - VARS.ARM_BD_L2_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                }

                if (gamepad1.b){
                    // THIRD
                    //cassette.setPosition(VARS.CST_BD_L3_POS);
                    setArmPos((int) VARS.ARM_START_POS - VARS.ARM_BD_L3_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                }

                if (gamepad1.a){
                    // START
                    //cassette.setPosition(0.8);
                    setArmPos((int) VARS.ARM_START_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                }


            }

            if (gamepad1.right_stick_button){
                if (VARS.intake){
                    intake_L.setPower(0);
                    intake_R.setPower(0);
                    VARS.intake = false;
                }else{
                    intake_L.setPower(1);
                    intake_R.setPower(1);
                    VARS.intake = true;
                }
            }

            if (gamepad2.dpad_up){
                armMotor.setPower(1);
                while (true){
                    if (!(Math.abs(armMotor.getCurrent(CurrentUnit.AMPS)) < 6)) break;
                    // waiting for stall before starting pull motor
                }
                pullMotor.setPower(-0.75);

            }else if (!gamepad2.y && !gamepad2.a && !gamepad2.dpad_down){
                armMotor.setPower(0);
                pullMotor.setPower(0);
            }

            if (gamepad2.dpad_down){
                armMotor.setPower(-1);
                pullMotor.setPower(0.75);

            }else if (!gamepad2.y && !gamepad2.a && !gamepad2.dpad_up){
                armMotor.setPower(0);
                pullMotor.setPower(0);
            }

            if (gamepad1.y) {
                // UP
                // opposite to arm
                if (armMotor.getCurrentPosition() > (VARS.ARM_START_POS - 6166)){
                    VARS.CST_DOWN = true;
                    armMotor.setPower(-1);
                    powerCassette(cassette);
                }

            }else if (!gamepad1.a){
                VARS.t0 = 0;
                VARS.CST_DOWN = false;
                armMotor.setPower(0);
                powerCassette(cassette);
            }

            if (gamepad1.a) {
                // DOWN
                if (!gamepad1.back){
                    if (armMotor.getCurrentPosition() < VARS.ARM_START_POS){
                        // opposite to arm
                        VARS.CST_UP = true;
                        armMotor.setPower(1);

                        powerCassette(cassette);
                    }
                }else{
                    VARS.ARM_START_POS = armMotor.getCurrentPosition();
                    // opposite to arm
                    VARS.CST_UP = true;
                    armMotor.setPower(1);

                    powerCassette(cassette);
                }
            }else if (!gamepad1.y){
                VARS.t0 = 0;
                VARS.CST_UP = false;
                armMotor.setPower(0);
                powerCassette(cassette);
            }


            if (VARS.CST_UP){
                // switch to flat position

                if (!Double.isNaN(cassette.getPosition())){
                    if (cassette.getPosition() + 0.05 < VARS.CST_LOWER_BOUND){
                        cassette.setPosition(cassette.getPosition() + 0.05);
                        sleep(100);
                    }else{
                        cassette.setPosition(VARS.CST_LOWER_BOUND);
                        sleep(100);
                    }

                }else{
                    cassette.setPosition(0);
                    sleep(100);
                }


            }
            if (gamepad1.dpad_left){
                door.setPosition(0);
            }
            else if (gamepad1.dpad_right){
                door.setPosition(0);
            }else{
                door.setPosition(0.6);
            }
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -80 && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -100){
                if (gamepad1.x){
                    // switch to parallel to backdrop position
                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() - 0.04 > VARS.CST_UPPER_BOUND){
                            if (Math.abs(armMotor.getCurrentPosition() - VARS.ARM_START_POS) < 300){
                                if (cassette.getPosition() > 0.35){
                                    cassette.setPosition(cassette.getPosition() - 0.04);
                                }else{
                                    cassette.setPosition(0.3);
                                }

                            }else{
                                cassette.setPosition(cassette.getPosition() - 0.04);
                            }

                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_UPPER_BOUND);
                            sleep(30);
                        }
                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }

                }



                telemetry.update();



                if (gamepad1.b){
                    // switch to flat position

                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() + 0.04 < VARS.CST_LOWER_BOUND){
                            cassette.setPosition(cassette.getPosition() + 0.04);
                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_LOWER_BOUND);
                            sleep(30);
                        }

                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }


                }


            }else{
                if (gamepad1.b){
                    // switch to parallel to backdrop position
                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() - 0.04 > VARS.CST_UPPER_BOUND){
                            if (Math.abs(armMotor.getCurrentPosition() - VARS.ARM_START_POS) < 300){
                                if (cassette.getPosition() > 0.4){
                                    cassette.setPosition(cassette.getPosition() - 0.04);
                                }else{
                                    cassette.setPosition(0.4);
                                }

                            }else{
                                cassette.setPosition(cassette.getPosition() - 0.04);
                            }

                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_UPPER_BOUND);
                            sleep(30);
                        }
                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }

                }

                if (gamepad1.x){
                    // switch to flat position

                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() + 0.04 < VARS.CST_LOWER_BOUND){
                            cassette.setPosition(cassette.getPosition() + 0.04);
                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_LOWER_BOUND);
                            sleep(30);
                        }

                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }


                }
            }


            if (VARS.CST_DOWN){
                // switch to parallel to backdrop position
                if (!Double.isNaN(cassette.getPosition())){
                    if (cassette.getPosition() - 0.05 > VARS.CST_UPPER_BOUND){
                        if (Math.abs(armMotor.getCurrentPosition() - VARS.ARM_START_POS) < 300){
                            if (cassette.getPosition() > 0.35){
                                cassette.setPosition(cassette.getPosition() - 0.05);
                            }else{
                                cassette.setPosition(0.3);
                            }

                        }else{
                            cassette.setPosition(cassette.getPosition() - 0.05);
                        }

                        sleep(100);
                    }else{
                        cassette.setPosition(VARS.CST_UPPER_BOUND);
                        sleep(100);
                    }
                }else{
                    cassette.setPosition(0);
                    sleep(100);
                }

            }
            // powerCassette(cassette);

            if (!Double.isNaN(door.getPosition())){
                door.setPosition(door.getPosition());
            }else{
                door.setPosition(1);
            }

        }
        visionPortal.close();
    }

}