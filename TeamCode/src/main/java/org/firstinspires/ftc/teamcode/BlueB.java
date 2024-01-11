package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TimeTrajectory;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

// B side is the side with the backdrop
// One hand in from OUTSIDE SEAM on LEFT
@Autonomous(name = "Blue B")
// TODO CLAMP VALUES AS NEEDED
public class BlueB extends LinearOpMode {

    OpenCvWebcam webcam = null;
    // double targetTicks;
    private IMU imu;

    PropDirection propDirectionID;

    enum PropDirection{
        LEFT,
        RIGHT,
        MIDDLE
    }

    final double startXPos = -71.15;
    final double startYPos = 12.52;

    public double CST_UPPER_BOUND = 0;
    public double CST_LOWER_BOUND = 1;

    public double ARM_START_POS;

    public int ARM_BD_L1_POS = Math.abs(-5550 + 65); //-5477
    public int ARM_BD_L2_POS = Math.abs(-4890 + 65);
    public int ARM_BD_L3_POS = Math.abs(-4303 + 65);

    public int ARM_BD_X_POS = 6011;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;
    public boolean CST_DOWN = false;
    public boolean CST_UP = false;

    DetectionPipeline pipeline;

    //TrajectoryBuilder trajectoryBuilder;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack, armMotor;

    Servo clawL, clawR, cassette, door;

    double armTicksPerRev, armStartPos, armDropPos, armTickPerDeg, armPickPos;
    double cstUnitPerDeg, cstStartPos, cstPickPos, cstDropPos;

    // Orientation angles = new Orientation();

    // UTIL FUNCTIONS

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
                sleep(20);
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam_1"))
                .addProcessor(aprilTag)
                .build();

    }

    void powerCassette(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            cassette.setPosition(cassette.getPosition());
        }else{
            cassette.setPosition(0);
        }
    }

    void moveCassetteDown(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() - 0.05 > CST_UPPER_BOUND){
                cassette.setPosition(cassette.getPosition() - 0.05);
                sleep(100);
            }else{
                cassette.setPosition(CST_UPPER_BOUND);
                sleep(100);
            }
        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }

    }

    void moveCassetteUp(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() + 0.05 < CST_LOWER_BOUND){
                cassette.setPosition(cassette.getPosition() + 0.05);
                sleep(100);
            }else{
                cassette.setPosition(CST_LOWER_BOUND);
                sleep(100);
            }

        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }
    }
    private void setArmPos(int position, DcMotorEx armMotor, Servo cassette){
        armMotor.setTargetPosition(position);
        armMotor.setPower(1);
        powerCassette(cassette);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int currentArmPos = armMotor.getCurrentPosition();
        while (armMotor.isBusy()){
            if (position > currentArmPos){
                moveCassetteUp(cassette);

            }else{
                moveCassetteDown(cassette);

            }
            telemetry.addData("Cassette Pos", cassette.getPosition());
            telemetry.addLine("Waiting for arm to reach position");
            telemetry.addData("Target Pos", position);
            telemetry.addData("Arm Ticks", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setPower(0);
        // stopping cassette
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void turn(double angle){
        // angle going from 0 90 180 - 90 0
        if (angle < 180 && angle > 0){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < angle){
                leftFront.setPower(-0.5);
                rightFront.setPower(0.5);
                leftBack.setPower(-0.5);
                rightBack.setPower(0.5);
            }
        }else if (angle > -180 && angle < 0){
            while (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > angle){
                leftFront.setPower(0.5);
                rightFront.setPower(-0.5);
                leftBack.setPower(0.5);
                rightBack.setPower(-0.5);
            }
        }

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
    private void moveBot(double inches){
        telemetry.addData("Current X", drive.pose.position.x);
        telemetry.update();
        if (inches > 0){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToX(inches + drive.pose.position.x)
                            .build()
            );
        }else{
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .lineToX(drive.pose.position.x - inches)
                            .build()
            );
        }


        telemetry.addData("Final X", drive.pose.position.x);
        telemetry.update();
    }
    void pushPxlFromCassette(){
        // TODO push pxl from cassette
    }
    void moveArm(double angle){
        // angle in degrees ^
        // cassette will maintain angle with respect to arm

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
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }

    private void outtake() {
        // TODO outtake
        // Need to move arm, drop pxl, and get arm back to start position
        moveArm(armDropPos);

        // Moving bot slightly forward to let pixel fall
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y))
                        .build()
                // TODO find correct values for x and y
        );

        moveArm(armStartPos);

    }
    private void setupOuttakeFirstPxl(){
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    private void resetOuttakeFirstPxl(){
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    private void pickWhitePxl(){
        // TODO intake
    }
    private void outtakeWhitePxl(){
        // TODO white pxl outtake
    }

    private void pick(){
        clawR.setPosition(0.8); // 1
        clawL.setPosition(0.1);//0
    }

    private void drop(){
        clawR.setPosition(0); // 0.2 // 0
        clawL.setPosition(1);//1
    }

    private TimeTrajectory convertTraj(List<Trajectory> t){
        return new TimeTrajectory(t.get(0));
    }

    // PRGM STARTS HERE

    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
        leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
        rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
        rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        clawR = hardwareMap.servo.get("clawR");
        clawL = hardwareMap.servo.get("clawL");
        cassette = hardwareMap.servo.get("cassette");
        door = hardwareMap.servo.get("door");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);

        //trajectoryBuilder = new TrajectoryBuilder(drive.pose, 1e-6, 0.0, drive.defaultVelConstraint, drive.defaultAccelConstraint, 0.25, 0.1);

//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        cstDropPos = 50;
        armDropPos = 10;
        cstStartPos = 24;
        armStartPos = -24;
        cstUnitPerDeg = 1 / 300.0;
        cstPickPos = 25;
        armPickPos = 35;

        imu = (IMU) hardwareMap.get("imu");
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(0)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);
        pipeline = new DetectionPipeline(2);
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int n) {

            }

        });
    }

    private void dropFirstPxl() throws InterruptedException {
        findTeamProp();
        telemetry.addData("PropDirectionID", propDirectionID);
        telemetry.update();
        telemetry.addData("Heading", drive.pose.heading);
        dropPxlOne(propDirectionID);

    }
    private void findTeamProp(){
        final int propNumID = pipeline.position;
        if (propNumID == 1){
            propDirectionID = PropDirection.LEFT;
        }else if (propNumID == 2){
            propDirectionID = PropDirection.MIDDLE;
        }else if (propNumID == 3){
            propDirectionID = PropDirection.RIGHT;
        }
        // PropDirection propDirectionID = PropDirection.RIGHT;
    }
    private void dropPxlOne(PropDirection propDirectionID) throws InterruptedException {
        // Find team prop happens BEFORE function is called
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        drive.updatePoseEstimate();

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.addData("DIRECTION", propDirectionID);
        telemetry.update();

        if (propDirectionID == PropDirection.LEFT || propDirectionID == PropDirection.RIGHT){
            telemetry.addData("DIRECTION", propDirectionID);
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                            //.splineTo(new Vector2d(60, 0), Math.PI)
//                        .splineTo(new Vector2d(-36.50, -36.50), Math.toRadians(0))
                            .strafeToLinearHeading(new Vector2d(-40, 36), Math.toRadians(-90))//was 24, 24// why was this + 24 if it doesnt go up in y value
//                            .strafeTo(new Vector2d(-40, drive.pose.position.y))
//                            .strafeTo(new Vector2d(-40, 20))
                            .build());
//            Actions.runBlocking(
//                    (Action) (trajectoryBuilder
//                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
//                            //.splineTo(new Vector2d(60, 0), Math.PI)
////                        .splineTo(new Vector2d(-36.50, -36.50), Math.toRadians(0))
//                            .strafeToLinearHeading(new Vector2d(drive.pose.position.x + 35, drive.pose.position.y + 19), Math.toRadians(-90))//was 24, 24// why was this + 24 if it doesnt go up in y value
//                            .build()));

            drop();

            // turn(-90);

            if (propDirectionID == PropDirection.LEFT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 5))//-7
                                .build()
                );

                drop();
                sleep(50);

            }

            if (propDirectionID == PropDirection.RIGHT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 27))
                                .build()
                );

                drop();
            }

        }else if (propDirectionID == PropDirection.MIDDLE){
            // TODO
            telemetry.addData("DIRECTION", propDirectionID);
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x + 28, drive.pose.position.y))

                            .build()
            );
            drop();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x - 10, drive.pose.position.y))

                            .build()
            );

        }

    }
    private void dropSecondPxl(){
        goToBackdrop(propDirectionID);
        dropPxlTwo();

    }
    private void goToBackdrop(PropDirection propDirectionID){
        drive.updatePoseEstimate();
        if (propDirectionID == PropDirection.LEFT || propDirectionID == PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            // Right in front of backdrop
                            //.strafeToConstantHeading(new Vector2d(-36.36, 39.6))
                            .strafeTo(new Vector2d(drive.pose.position.x - 5, 37))//  was -36.36, 39.6, +4
                            .turn(Math.toRadians(180))
                            //.turn(Math.toRadians(180))
                            .build()
            );
        }else{
            Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        // Right in front of backdrop
                        //.strafeToConstantHeading(new Vector2d(-36.36, 39.6))
                        .strafeTo(new Vector2d(-45, 37))//  was -36.36, 39.6, +4
                        .turn(Math.toRadians(90))
                        //.turn(Math.toRadians(180))
                        .build()
        );

        }


//        if (propDirectionID == PropDirection.MIDDLE){
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            //.lineToXConstantHeading(-39)
//                            .strafeToConstantHeading(new Vector2d(-44, 43))
//                            .build()
//            );
//        }



        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive_          = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;
        int DESIRED_TAG_ID      = 2;
//        if (propDirectionID == PropDirection.LEFT){
//            DESIRED_TAG_ID      = 1;
//        }
//
//        if (propDirectionID == PropDirection.RIGHT){
//            DESIRED_TAG_ID      = 3;
//        }
          // Desired turning power/speed (-1 to +1)
        final double DESIRED_DISTANCE = 11; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        final double SPEED_GAIN  =    0.05;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        final double STRAFE_GAIN =  0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        final double TURN_GAIN   =   0.015;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.5;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.0;   //  Clip the turn speed to this max value (adjust for your robot)

        // Initialize the April tag Detection process
        initAprilTag();

        setManualExposure(6, 250);

        // Used to hold the data for a detected AprilTag
        AprilTagDetection desiredTag = null;

        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();

        // TODO change break condition
        while (time1.time() < 2){
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("Detections", aprilTag.getDetections());
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((detection.id == DESIRED_TAG_ID)) {
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
                //telemetry.addData("\n>","HOLD Left-Bumper to Drive to Target\n");
                telemetry.addData("Found", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                double  rangeError      = (desiredTag.ftcPose.range - DESIRED_DISTANCE);
                double  headingError    = desiredTag.ftcPose.bearing;
                double  yawError        = desiredTag.ftcPose.yaw;

                // Use the speed and turn "gains" to calculate how we want the robot to move.
                drive_  = Range.clip(rangeError * SPEED_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                turn   = Range.clip(headingError * TURN_GAIN, -MAX_AUTO_TURN, MAX_AUTO_TURN) ;
                strafe = Range.clip(-yawError * STRAFE_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);
                // Apply desired axes motions to the drivetrain.
                moveRobot(drive_, strafe, turn);

                if (Math.abs(desiredTag.ftcPose.yaw) < .5 && Math.abs(desiredTag.ftcPose.range - DESIRED_DISTANCE) < 0.2){
                    break;
                }
            } else {
                telemetry.addData("\n>","Waiting for target...\n");
            }
            telemetry.update();
            sleep(10);
        }

        drive.updatePoseEstimate();

        if (propDirectionID == PropDirection.LEFT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x - 5.5, drive.pose.position.y))
//                            .lineToX(drive.pose.position.x - 5)
                            .build()
            );
        }

        if (propDirectionID == PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x + 12.7, drive.pose.position.y))
//                            .lineToX(drive.pose.position.x + 5)
                            .build()
            );
        }

        if (propDirectionID == PropDirection.MIDDLE){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x + 6, drive.pose.position.y))
//                            .lineToX(drive.pose.position.x + 5)
                            .build()
            );
        }

        drive.updatePoseEstimate();


//        if (propDirectionID == PropDirection.MIDDLE){
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            // Right in front of backdrop
//                            //.strafeToConstantHeading(new Vector2d(-36.36, 39.6))
//                            .strafeTo(new Vector2d(drive.pose.position.x - 5, 42.1)) //  was -36.36, 39.6, +4
//                            //.turn(Math.toRadians(180))
//                            .build()
//            );
//        }
//
//        if (propDirectionID == PropDirection.LEFT){
//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            // Right in front of backdrop
//                            //.strafeToConstantHeading(new Vector2d(-36.36, 39.6))
//                            .strafeTo(new Vector2d(drive.pose.position.x - 10, 42.1)) //  was -36.36, 39.6, +4
//                            //.turn(Math.toRadians(180))
//                            .build()
//            );
//        }

    }
    private void dropPxlTwo(){
        // TODO outtake
        //outtake();

        setArmPos((int) ARM_START_POS - ARM_BD_X_POS + 60, armMotor, cassette); // was ARM_BD_L3_POS but want to change to 2 or 1

        sleep(200);


        door.setPosition(0.3);
        sleep(200);
        setArmPos((int) ARM_START_POS, armMotor, cassette);
        sleep(50);
        door.setPosition(1);

    }
    private void setupForLoops(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(-22.32, 38.50), Math.toRadians(-17.72))
                        .splineTo(new Vector2d(-9.53, -3.87), Math.toRadians(-86.93))
                        .splineTo(new Vector2d(-10.57, -67.96), Math.toRadians(268.59))
                        // TODO Fine tune on field
                        .build()
        );

    }
    private void dropWhitePxl(){
        goToBackdropInLoop();
        outtakeWhitePxl();
    }
    private void goToBackdropInLoop(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(-11.49, 31.70), Math.toRadians(91.43))
//                        .splineToConstantHeading(new Vector2d(-36.23, 51.49), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-3.03, -5.43), Math.toRadians(93.81))
                        .splineTo(new Vector2d(-26.16, 36.60), Math.toRadians(130.60))
                        .splineTo(new Vector2d(-33.92, 50.00), Math.toRadians(90.00))
                        .build()

        );
    }

    private void setupForPark(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(-17.68, 41.90), Math.toRadians(79.56))
                        .build()

        );
    }
    private void park(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(-66, drive.pose.position.y))
                        .build()
        );

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y + 12), Math.toRadians(0))
                        .build()
        );


    }
    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        initialize();

        telemetry.addLine("left " + pipeline.leftavgfinal);
        telemetry.addLine("right " + pipeline.rightavgfinal);
        telemetry.addLine("middle" + pipeline.midavgfinal);

        ARM_START_POS = armMotor.getCurrentPosition();

        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();
        // TODO set cassette to backdrop angle

        // Purple Pixel (first pixel) on floor to be pushed
        // Yellow Pixel (second pixel) in cassette
        cassette.setPosition(1);
        pick();
        dropFirstPxl();
        dropSecondPxl();
        //setupForLoops();
        // loop till T - 5
        //while (opModeIsActive() && time.seconds() < 25){
        //    pickAndDropWhitePxl();
        //}
        // end loop
        // setupForPark();
        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


