package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.core.Mat;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

// A Side is opposite to backdrop
@Autonomous(name = "Blue Audience")
public class BlueA extends LinearOpMode {

    OpenCvWebcam webcam = null;
    double targetTicks;
    CRServo arm;
    private Servo claw;
    private Servo cassette;
    private Servo door;
    private IMU imu;

    public Encoder par0, par1, perp;

    public double CST_UPPER_BOUND = 0;
    public double CST_LOWER_BOUND = 1;

    public double ARM_START_POS;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;

    public int ARM_BD_X_POS = 6011;
    enum PropDirection{
        LEFT,
        RIGHT,
        MIDDLE
    }

    final double startXPos = -40;
    final double startYPos = 60;

    PropDirection propDirectionID;

    DetectionPipeline pipeline;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack, armMotor;
    double armTicksPerRev, armStartPos, armDropPos, armTickPerDeg, armPickPos;
    double cstUnitPerDeg;
    double cstStartPos, cstPickPos, cstDropPos;

    void moveArm(double angle){
        // angle in degrees ^
        // cassette will maintain angle with respect to arm
        armMotor.setTargetPosition((int) (angle * armTickPerDeg));
        //cassette.setPosition(-angle * cstUnitPerDeg);
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

    private void moveBot(double inches){
        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        int startPar0 = par0.getPositionAndVelocity().position;
        int startPar1 = par1.getPositionAndVelocity().position;
        int startPerp = perp.getPositionAndVelocity().position;
        double targetTicks = Math.abs(inches) * (1/inPerTick);
        if (inches != 0){
            if (inches > 0){
                leftFront.setPower(0.4);
                leftBack.setPower(0.4);
                rightFront.setPower(0.4);
                rightBack.setPower(0.4);
                while (par0.getPositionAndVelocity().position < (targetTicks + startPar0) && par1.getPositionAndVelocity().position < (targetTicks + startPar1)){
                    if (par0.getPositionAndVelocity().position >= (targetTicks + startPar0)){//startPar0 + targetTicks){
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                    }

                    if (par1.getPositionAndVelocity().position >= (targetTicks + startPar1)){//startPar1 + targetTicks){
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                }

                leftFront.setPower(-0.2);
                leftBack.setPower(-0.2);
                rightFront.setPower(-0.2);
                rightBack.setPower(-0.2);

                while (par0.getPositionAndVelocity().position > (startPar0 + targetTicks) || par1.getPositionAndVelocity().position > (startPar1 + targetTicks)){
                    if (par0.getPositionAndVelocity().position <= (startPar0 + targetTicks)){//startPar0 + targetTicks){
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                    }

                    if (par1.getPositionAndVelocity().position <= (startPar1 + targetTicks)){//startPar1 + targetTicks){
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                }


            }
            if (inches < 0){
                leftFront.setPower(-0.4);
                leftBack.setPower(-0.4);
                rightFront.setPower(-0.4);
                rightBack.setPower(-0.4);
                while (par0.getPositionAndVelocity().position > (startPar0 - targetTicks) && par1.getPositionAndVelocity().position > (startPar1 - targetTicks)){
                    if (par0.getPositionAndVelocity().position <= (startPar0 - targetTicks)){//startPar0 + targetTicks){
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                    }

                    if (par1.getPositionAndVelocity().position <= (startPar1 - targetTicks)){//startPar1 + targetTicks){
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                }

                leftFront.setPower(0.2);
                leftBack.setPower(0.2);
                rightFront.setPower(0.2);
                rightBack.setPower(0.2);

                ////////

                while (par0.getPositionAndVelocity().position < (startPar0 - targetTicks) || par1.getPositionAndVelocity().position < (startPar1 - targetTicks)){
                    if (par0.getPositionAndVelocity().position >= (startPar0 - targetTicks)){//startPar0 + targetTicks){
                        leftFront.setPower(0);
                        leftBack.setPower(0);
                    }

                    if (par1.getPositionAndVelocity().position >= (startPar1 - targetTicks)){//startPar1 + targetTicks){
                        rightFront.setPower(0);
                        rightBack.setPower(0);
                    }

                }

            }

            leftFront.setPower(0);
            rightFront.setPower(0);
            leftBack.setPower(0);
            rightBack.setPower(0);
        }

    }

    private void strafeBot(double inches){
        double inPerTick = MecanumDrive.PARAMS.inPerTick;
        //double lateralInPerTick = MecanumDrive.PARAMS.lateralInPerTick;
        double startPerp = perp.getPositionAndVelocity().position;
        double targetTicks = Math.abs(inches) * (1/inPerTick);
        //double targetTicks = Math.abs(inches) * (1/lateralInPerTick);

        if (inches > 0){// strafe left
            leftFront.setPower(-0.38);
            rightBack.setPower(-0.2);
            rightFront.setPower(0.2);
            leftBack.setPower(0.2);
            float currPosition = perp.getPositionAndVelocity().position;
            while (currPosition < (startPerp + targetTicks)){
                if (currPosition >= (startPerp + targetTicks)){
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    sleep(20);
                    break;

                }
                currPosition = perp.getPositionAndVelocity().position;
            }
        }

        if (inches < 0){ // strafe right
            leftFront.setPower(0.38);
            rightBack.setPower(0.2);
            rightFront.setPower(-0.2);
            leftBack.setPower(-0.2);
            float currPosition = perp.getPositionAndVelocity().position;
            while (currPosition > (startPerp - targetTicks)){
                if (currPosition <= (startPerp - targetTicks)){
                    leftFront.setPower(0);
                    rightBack.setPower(0);
                    rightFront.setPower(0);
                    leftBack.setPower(0);
                    sleep(20);
                    break;

                }
                currPosition = perp.getPositionAndVelocity().position;
            }
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
        leftFront.setPower(leftFrontPower);
        rightFront.setPower(rightFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);
    }
    private void setupOuttakeFirstPxl(){
        leftFront.setPower(0.3);
        rightFront.setPower(0.3);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    private void resetOuttakeFirstPxl(){
        leftFront.setPower(-0.3);
        rightFront.setPower(-0.3);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }
    private void pick(){
        claw.setPosition(0.8); // 1
    }
    private void drop(){
        claw.setPosition(-0.3); // 0.2 // 0
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
            if (cassette.getPosition() - 0.055 > CST_UPPER_BOUND){
                cassette.setPosition(cassette.getPosition() - 0.055);
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
            if (cassette.getPosition() + 0.055 < CST_LOWER_BOUND){
                cassette.setPosition(cassette.getPosition() + 0.055);
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
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double tolerance = 200;
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
                moveCassetteUp(cassette);
            }else{
                moveCassetteDown(cassette);
            }

            if (((Math.abs(par0Pos - par0.getPositionAndVelocity().position) > 50 && Math.abs(par1Pos - par1.getPositionAndVelocity().position) > 50))){
                break;
            }

        }

        armMotor.setPower(0);
    }

    private void OLDsetArmPos(int position, DcMotorEx armMotor, Servo cassette){
        sleep(50);
        armMotor.setTargetPosition(position);
        armMotor.setPower(-1);
        int par0Pos = par0.getPositionAndVelocity().position;
        int par1Pos = par1.getPositionAndVelocity().position;
        int perpPos = perp.getPositionAndVelocity().position;
        powerCassette(cassette);
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();
        int currentArmPos = armMotor.getCurrentPosition();
        while ((armMotor.isBusy() || armMotor.getCurrentPosition() != position) && time1.time() < 6 && (Math.abs(par0Pos - par0.getPositionAndVelocity().position) < 100 && Math.abs(par1Pos - par1.getPositionAndVelocity().position) < 100 && Math.abs(perpPos - perp.getPositionAndVelocity().position) < 100)){
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
        sleep(70);
        // stopping cassette
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
        leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
        rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
        rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        par0 = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).par0;
        //par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"))); // leftEncoder
        par1 = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).par1;
        // par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntLF"))); // frntLF
        perp = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).perp;
        //perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));

        par0.setDirection(DcMotorSimple.Direction.FORWARD);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        claw = hardwareMap.servo.get("claw");
        cassette = hardwareMap.servo.get("cassette");
        door = hardwareMap.servo.get("door");
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cstDropPos = 50;
        armDropPos = 10;
        cstStartPos = 24;
        armStartPos = 24;
        cstUnitPerDeg = 1 / 300.0;
        cstPickPos = 25;
        armPickPos = 35;
        imu = (IMU) hardwareMap.get("imu");
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(270)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectionPipeline(2);
        webcam.setPipeline(pipeline);
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int n) {

            }

        });
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
                            .strafeToLinearHeading(new Vector2d(-54, 32), Math.toRadians(0), new TranslationalVelConstraint(70))//was 24, 24// why was this + 24 if it doesnt go up in y value
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


            drive.updatePoseEstimate();

            // turn(-90);

            if (propDirectionID == PropDirection.LEFT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(-28, drive.pose.position.y))//-7
                                .build()
                );

                drop();
                sleep(50);

            }

            if (propDirectionID == PropDirection.RIGHT){
                drive.updatePoseEstimate();
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
//                                .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y + 2))
                                .lineToX(-53, new TranslationalVelConstraint(60), new ProfileAccelConstraint(-30, 30))
                                //.strafeTo(new Vector2d(drive.pose.position.x, -56))
                                .build()
                );

                drop();
            }

        }else if (propDirectionID == PropDirection.MIDDLE){
            telemetry.addData("DIRECTION", propDirectionID);
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 28))

                            .build()
            );
            drop();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y + 5))

                            .build()
            );

        }

    }

    private void setupForPxlTwo(){

        {
            if (propDirectionID == PropDirection.MIDDLE){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-54, 32), Math.toRadians(0), new TranslationalVelConstraint(80), new ProfileAccelConstraint(- 30, 40))
                                //.strafeTo(new Vector2d(-40, -57), new TranslationalVelConstraint(80), new ProfileAccelConstraint(- 30, 40))
                                .build()
                );
            }else if (propDirectionID == PropDirection.LEFT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-40, 32), Math.toRadians(0), new TranslationalVelConstraint(80), new ProfileAccelConstraint(- 30, 40))
                                //.strafeTo(new Vector2d(-40, -57), new TranslationalVelConstraint(80), new ProfileAccelConstraint(- 30, 40))
                                .build()
                );
            }

            else{
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-54, 32), Math.toRadians(0), new TranslationalVelConstraint(80), new ProfileAccelConstraint(- 30, 40))
                                //.strafeTo(new Vector2d(-40, -57), new TranslationalVelConstraint(80), new ProfileAccelConstraint(- 30, 40))
                                .build()
                );
            }


            drive.updatePoseEstimate();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(new Vector2d(-52, 12.36), Math.toRadians(5), new TranslationalVelConstraint(80), new ProfileAccelConstraint(-30, 35))

                            .build()
            );

            drive.updatePoseEstimate();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
//                            .strafeTo(new Vector2d(36, drive.pose.position.y), new TranslationalVelConstraint(90) , new ProfileAccelConstraint(-30, 50))
//                            .strafeTo(new Vector2d(39, 34), new TranslationalVelConstraint(40))
                            .splineTo(new Vector2d(-24.28, 12.51), Math.toRadians(0))
                            .splineTo(new Vector2d(16.90, 13.50), Math.toRadians(0))
                            //.splineTo(new Vector2d(31.10, 17.89), Math.toRadians(0))
                            .splineTo(new Vector2d(50.73, 13.50), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-30, 50))
                            //.strafeTo(new Vector2d(44.73, 45.51), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-30, 50))
                            .splineToConstantHeading(new Vector2d(38.73, 45.51), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-30, 70))

                            .build()
            );

        }

    }
    private void dropSecondPxl(){
        goToBackdrop();
        dropPxlTwo();
    }
    private void goToBackdrop(){
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive_          = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;
        int DESIRED_TAG_ID      = 2;
        if (propDirectionID == PropDirection.LEFT){
            DESIRED_TAG_ID      = 1;
        }

        if (propDirectionID == PropDirection.RIGHT){
            DESIRED_TAG_ID      = 3;
        }
        // Desired turning power/speed (-1 to +1)
        double DESIRED_DISTANCE = 10.5; //  this is how close the camera should get to the target (inches)

        //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
        //  applied to the drive motors to correct the error.
        //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
        double SPEED_GAIN  =    0.05;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
        double STRAFE_GAIN =  0.015;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
        double TURN_GAIN   =   0.015;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

        final double MAX_AUTO_SPEED = 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_STRAFE= 0.7;   //  Clip the approach speed to this max value (adjust for your robot)
        final double MAX_AUTO_TURN  = 0.2;
        boolean hasMoved = false; //  Clip the turn speed to this max value (adjust for your robot)

        // Initialize the April tag Detection process
        initAprilTag();

        setManualExposure(6, 250);

        // Used to hold the data for a detected AprilTag
        AprilTagDetection desiredTag = null;

        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();

        while (time1.time() < 3.5){
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
                hasMoved = true;
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
                if (!hasMoved){
                    time1.reset();
                    if (!currentDetections.isEmpty()){
                        if (currentDetections.get(0).id < DESIRED_TAG_ID){
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 5))
                                            .build()
                            );
                        }
                        if (currentDetections.get(0).id > DESIRED_TAG_ID){
                            Actions.runBlocking(
                                    drive.actionBuilder(drive.pose)
                                            .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y + 5))
                                            .build()
                            );
                        }
                    }
                }

            }
            telemetry.update();
            sleep(10);
        }
        drive_ = 0;
        turn   = 0;
        strafe = 0;
        SPEED_GAIN = 0;
        STRAFE_GAIN = 0;
        TURN_GAIN = 0;
        sleep(300);
        strafeBot(-0.6);

        //drive.pose = new Pose2d(new Vector2d(desiredTag.ftcPose.x - desiredTag.ftcPose.range, desiredTag.ftcPose.y), Math.toRadians(0));

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);


        drive.updatePoseEstimate();

    }
    private void dropPxlTwo(){
        //outtake();
        int startPos = armMotor.getCurrentPosition();
        try{
            OLDsetArmPos((int) startPos - ARM_BD_X_POS, armMotor, cassette); // was ARM_BD_L3_POS but want to change to 2 or 1
        }catch (Exception e){
            telemetry.addLine(e.toString());
            telemetry.update();
        }

//        sleep(200);

        sleep(10);
        cassette.setPosition(cassette.getPosition() + 0.1);
        sleep(50);
        cassette.setPosition(cassette.getPosition() + 0.1);
        sleep(50);
        cassette.setPosition(cassette.getPosition() + 0.05);
        sleep(1000);
        door.setPosition(0);
        sleep(200);
//        // arm coming back
//        try {
//            OLDsetArmPos((int) startPos, armMotor, cassette);
//            //sleep(200);
//        }catch (Exception e){
//            telemetry.addLine(e.toString());
//            telemetry.update();
//        }

        armMotor.setPower(1);
        sleep(1500);
        armMotor.setPower(0);
        sleep(50);
        door.setPosition(0.6);


    }

    private void park(){
        drive.updatePoseEstimate();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x, 12))
                        .strafeToLinearHeading(new Vector2d(60, 13), Math.toRadians(270))
                        .strafeTo(new Vector2d(60, 12.3))
                        //.strafeToConstantHeading(new Vector2d(startXPos + 100, startYPos + 100))
                        .build()

        );

        moveBot(-7);


    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        initialize();

        claw = hardwareMap.servo.get("claw");

        telemetry.addLine("left " + pipeline.leftavgfinal);
        telemetry.addLine("right " + pipeline.rightavgfinal);
        telemetry.addLine("middle" + pipeline.midavgfinal);

        final double l_turn = Math.toRadians(90); // Due to turning error, making manual adjustment
        final double r_turn = Math.toRadians(270);

        telemetry.addData("L Turn", l_turn);
        telemetry.addData("Turn Error", Math.abs(Math.toRadians(90) - l_turn));
        telemetry.update();

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        ARM_START_POS = armMotor.getCurrentPosition();

        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();
        // Purple Pixel (first pixel) on floor to be pushed
        // Yellow Pixel (second pixel) in cassette
        cassette.setPosition(1);
        pick();
        strafeBot(2);
        dropFirstPxl();
        setupForPxlTwo();
        dropSecondPxl();
        // setupForLoops();
        // loop till T - 5
//        while (opModeIsActive() && time.seconds() < 25){
//            pickAndDropWhitePxl();
//        }
        // end loop

        // Only one loop was possible
        //pickAndDropWhitePxl();
        //setupForPark();
        sleep(500);
//        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


