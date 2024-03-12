package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.List;
import java.util.concurrent.TimeUnit;

// A Side is opposite to backdrop
@Autonomous(name = "Red Backdrop Testing")
public class RedBNEW extends LinearOpMode {

    OpenCvWebcam webcam = null;
    private Servo claw;
    private Servo cassette;
    private Servo door;
    private double delay = 0;
    private ParkLocation parkLocation = ParkLocation.AWAY;

    private GateLocation gateLocation = GateLocation.MIDDLE;
    private boolean willDropYellow = true;
    private boolean willPark = true;

    private IMU imu;

    public Encoder par0, par1, perp;

    Telemetry dashboardTelemetry;

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

    enum ParkLocation{
        CLOSE,
        AWAY
    }

    enum GateLocation{
        MIDDLE,
        TEAM
    }

    final double startXPos = 16;
    final double startYPos = -60;

    PropDirection propDirectionID;

    DetectionPipeline pipeline;
    OpenCvCamera viewWebcam;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack, armMotor;
    double armStartPos, armDropPos, armTickPerDeg, armPickPos;
    double cstUnitPerDeg;
    double cstStartPos, cstPickPos, cstDropPos;

    void moveArm(double angle){
        // angle in degrees ^
        // cassette will maintain angle with respect to arm
        armMotor.setTargetPosition((int) (angle * armTickPerDeg));
        //cassette.setPosition(-angle * cstUnitPerDeg);
    }

    private void sleep_(long ms){
        sleep(ms / 2);
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
                    //sleep_(5);

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
                    //sleep_(20);

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
    private void pick(){
        claw.setPosition(0.8); // 1
    }
    private void drop(){
        claw.setPosition(0); // 0.2 // 0
    }
    private void outtakeWhitePxl(){
        drop();
    }
    private void pickWhitePxl(){
        pick();
    }

    void powerCassette(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            cassette.setPosition(cassette.getPosition());
        }else{
            cassette.setPosition(0);
        }
    }

    void moveCassetteDown(){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() - 0.045 > CST_UPPER_BOUND){
                cassette.setPosition(cassette.getPosition() - 0.045);
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

    void moveCassetteUp(){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() + 0.045 < CST_LOWER_BOUND){
                cassette.setPosition(cassette.getPosition() + 0.045);
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

    private void setArmPos(int position){
        sleep_(50);
        double tolerance = 30;
        int par0Pos = par0.getPositionAndVelocity().position;
        int par1Pos = par1.getPositionAndVelocity().position;
        int perpPos = perp.getPositionAndVelocity().position;
        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();
        while (armMotor.getCurrentPosition() != position && Math.abs(position - armMotor.getCurrentPosition()) > tolerance && ((Math.abs(par0Pos - par0.getPositionAndVelocity().position) < 50 && Math.abs(par1Pos - par1.getPositionAndVelocity().position) < 50 && Math.abs(perpPos - perp.getPositionAndVelocity().position) < 50)) &&  time1.time() < 2.5) {

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

        }
        sleep_(50);
    }

    private void OLDsetArmPos(int position, DcMotorEx armMotor, Servo cassette){
        sleep_(50);
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
        while ((armMotor.isBusy() || armMotor.getCurrentPosition() != position) && time1.time() < 6 && (Math.abs(par0Pos - par0.getPositionAndVelocity().position) < 60 && Math.abs(par1Pos - par1.getPositionAndVelocity().position) < 60 && Math.abs(perpPos - perp.getPositionAndVelocity().position) < 60)){
            if (position > currentArmPos){
                moveCassetteUp();

            }else{
                moveCassetteDown();

            }
            telemetry.addData("Cassette Pos", cassette.getPosition());
            telemetry.addLine("Waiting for arm to reach position");
            telemetry.addData("Target Pos", position);
            telemetry.addData("Arm Ticks", armMotor.getCurrentPosition());
            telemetry.update();
        }
        armMotor.setPower(0);
        sleep_(70);
        // stopping cassette
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    private void DUMBsetArmPos(int position, DcMotorEx armMotor, Servo cassette){
        int currentArmPos = armMotor.getCurrentPosition();
        int par0Pos = par0.getPositionAndVelocity().position;
        int par1Pos = par1.getPositionAndVelocity().position;
        int perpPos = perp.getPositionAndVelocity().position;
        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();
        while ((armMotor.isBusy() || armMotor.getCurrentPosition() != position) && time1.time() < 6 && (Math.abs(par0Pos - par0.getPositionAndVelocity().position) < 100 && Math.abs(par1Pos - par1.getPositionAndVelocity().position) < 100 && Math.abs(perpPos - perp.getPositionAndVelocity().position) < 100)){
            if (position > currentArmPos){
                armMotor.setPower(1);
                moveCassetteUp();

            }else{
                armMotor.setPower(-1);
                moveCassetteDown();

            }
        }
        armMotor.setPower(0);
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
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
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
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(90)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        // TODO switch to red
        pipeline = new DetectionPipeline(1);
        webcam.setPipeline(pipeline);
        viewWebcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        FtcDashboard.getInstance().startCameraStream(webcam, 60);
        dashboardTelemetry = FtcDashboard.getInstance().getTelemetry();
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
                //sleep_(20);
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
                //sleep_(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            //sleep_(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            //sleep_(20);
        }
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

    private void dropPurplePixel(PropDirection propDirectionID) throws InterruptedException {
        // Find team prop happens BEFORE function is called
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        drive.updatePoseEstimate();

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.addData("DIRECTION", propDirectionID);
        telemetry.update();

        if (propDirectionID == PropDirection.MIDDLE){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
//                            .strafeTo(new Vector2d(-56, -30.5), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 50))
//                            .strafeTo(new Vector2d(-40, -30.5))
                            .strafeTo(new Vector2d(-60, -22))
                            .strafeTo(new Vector2d(-40, -22))
                            .strafeTo(new Vector2d(-40, -31.5))
                            .stopAndAdd(new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    drop();
                                    return false;
                                }
                            })
                            .strafeTo(new Vector2d(-40, -40), new TranslationalVelConstraint(20), new ProfileAccelConstraint(-40, 20))
                            .build()
            );
        }

        if (propDirectionID == PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(-40, -55))
                            .strafeToLinearHeading(new Vector2d(-32, -31.95), Math.toRadians(0))
                            .stopAndAdd(new Action() {
                                @Override
                                public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                    drop();
                                    return false;
                                }
                            })
                            .build()
            );
        }

        if (propDirectionID == PropDirection.LEFT){
            if (gateLocation == GateLocation.MIDDLE){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(-40, -55))
                                .strafeToLinearHeading(new Vector2d(-58, -31.95), Math.toRadians(-10))
                                .strafeToLinearHeading(new Vector2d(-54, -31.95), Math.toRadians(-10))
                                .stopAndAdd(new Action() {
                                    @Override
                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                        drop();
                                        return false;
                                    }
                                })
                                .strafeTo(new Vector2d(-57, -31.95))
                                .build()
                );
            }
            if (gateLocation == GateLocation.TEAM){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(-47.43, -38.20))
                                .stopAndAdd(new Action() {
                                    @Override
                                    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                                        drop();
                                        return false;
                                    }
                                })

                                .strafeTo(new Vector2d(-38.06, -49.99))
                                .build()
                );
            }

        }

    }

    private void goToBackdrop(){
        if (gateLocation == GateLocation.MIDDLE){

            if (propDirectionID == PropDirection.MIDDLE || propDirectionID == PropDirection.RIGHT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .setTangent(Math.toRadians(180))
                                .splineToLinearHeading(new Pose2d(-47, -11.5, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 50))
//                                .splineToLinearHeading(new Pose2d(-47, -9.5, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 50))
                                .build()
                );
            }

            if (propDirectionID == PropDirection.LEFT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(-54, -13))
                                .build()
                );
            }

            if (willDropYellow){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                //.splineToConstantHeading(new Vector2d(28, -17), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(28, -20), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(40, -45), Math.toRadians(0))
                                .build()
                );
            }
            if (!willDropYellow){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .splineToConstantHeading(new Vector2d(28, -17), Math.toRadians(0))
                                .build()
                );
            }

        }

        if (gateLocation == GateLocation.TEAM){

            if (propDirectionID == PropDirection.LEFT || propDirectionID == PropDirection.RIGHT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(-40, -55), Math.toRadians(0))
                                .build()
                );
            }

            if (willPark){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(startXPos + 2, -59), Math.toRadians(0))
                                .strafeTo(new Vector2d(31.67, -56))
                                .build()
                );
            }
            if (!willPark){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(startXPos + 2, -59), Math.toRadians(90))
                                .build()
                );
            }


            if (willDropYellow){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeToLinearHeading(new Vector2d(40, -33), Math.toRadians(0))
                                .build()
                );
            }
        }
    }
    private void readAprilTags(){
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive_          = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;
        int DESIRED_TAG_ID      = 5;
        if (propDirectionID == PropDirection.LEFT){
            DESIRED_TAG_ID      = 4;
        }

        if (propDirectionID == PropDirection.RIGHT){
            DESIRED_TAG_ID      = 6;
        }
        // Desired turning power/speed (-1 to +1)
        double DESIRED_DISTANCE = 10;
        //  this is how close the camera should get to the target (inches)

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

        while (time1.time() < 3){
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

                if (Math.abs(desiredTag.ftcPose.yaw) < 3 && Math.abs(desiredTag.ftcPose.range - DESIRED_DISTANCE) < 1){
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
            sleep_(10);
        }
        drive_ = 0;
        turn   = 0;
        strafe = 0;
        SPEED_GAIN = 0;
        STRAFE_GAIN = 0;
        TURN_GAIN = 0;

//        if (propDirectionID == PropDirection.RIGHT){
//            strafeBot(-0.2);
//        }if(propDirectionID == PropDirection.LEFT){
//            strafeBot(0.2);
//        }
//        if (propDirectionID == PropDirection.MIDDLE){
//            strafeBot(-0.6);
//        }

        if (propDirectionID == PropDirection.MIDDLE){
            strafeBot(-0.6);
        }
        if (propDirectionID == PropDirection.RIGHT){
            strafeBot(-0.6);
        }

        //drive.pose = new Pose2d(new Vector2d(desiredTag.ftcPose.x - desiredTag.ftcPose.range, desiredTag.ftcPose.y), Math.toRadians(0));
        leftBack.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
        rightFront.setPower(0);

        drive.updatePoseEstimate();

    }
    private void dropYellowPixel(){
        //outtake();
        int startPos = armMotor.getCurrentPosition();
        sleep(50);
        try{
            DUMBsetArmPos((int) startPos - ARM_BD_X_POS, armMotor, cassette);
        }catch (Exception e){
            telemetry.addLine(e.toString());
            telemetry.update();
        }

        sleep_(10);
        cassette.setPosition(cassette.getPosition() + 0.1);
        sleep_(50);
        cassette.setPosition(cassette.getPosition() + 0.1);
        sleep_(50);
        cassette.setPosition(cassette.getPosition() + 0.05);
        sleep_(500);
        door.setPosition(0);
        sleep_(200);
        armMotor.setPower(1);
        sleep_(2500);
        armMotor.setPower(0);
        sleep_(50);
        door.setPosition(0.6);


    }
    private void park(){
        if (parkLocation == ParkLocation.AWAY){
            if (gateLocation == GateLocation.MIDDLE){
                if (propDirectionID == PropDirection.LEFT){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(drive.pose.position.x, -25))
                                    .strafeToLinearHeading(new Vector2d(60, -23), Math.toRadians(90))
                                    .build()
                    );
                }else{
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(drive.pose.position.x, -17))
                                    .strafeToLinearHeading(new Vector2d(60, -17), Math.toRadians(90))
                                    .build()
                    );
                }

            }

            if (gateLocation == GateLocation.TEAM){
                if (willDropYellow){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(drive.pose.position.x, -12))
                                    .strafeToLinearHeading(new Vector2d(60, -12), Math.toRadians(90))
                                    .build()
                    );
                }
                if (!willDropYellow){
                    Actions.runBlocking(
                            drive.actionBuilder(drive.pose)
                                    .strafeTo(new Vector2d(drive.pose.position.x, -12))
                                    .strafeToLinearHeading(new Vector2d(60, -12), Math.toRadians(90))
                                    .build()
                    );
                }
            }

        }

        if (parkLocation == ParkLocation.CLOSE){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x, -61))
                            .strafeToLinearHeading(new Vector2d(60, -67), Math.toRadians(90))
                            .build()
            );
        }


    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();
        initialize();

        claw = hardwareMap.servo.get("claw");
        ARM_START_POS = armMotor.getCurrentPosition();
        cassette.setPosition(1);
        while (opModeInInit()){
            telemetry.addLine("Xbox Buttons");
            telemetry.addLine("");
            telemetry.addLine(" A / B           - Add / Remove Delay");
            if (!willDropYellow){
                telemetry.addLine(" Y / X         - Will / Will Not Park");
            }
            if (willPark){
                telemetry.addLine(" ↑ / ↓         - Away / Close Parking");
                telemetry.addLine("← / →         - Middle / Red Truss");
            }
            telemetry.addLine("  BACK         - Toggle Yellow Pixel Drop");
            //telemetry.addLine("Press A to add delay, Press B to remove delay");
            telemetry.addLine("");
            telemetry.addLine("Current Delay: " + delay);
            //telemetry.addLine("Press DPAD-UP for away parking, Press DPAD-DOWN for close parking");
            if (willPark){
                if (parkLocation == ParkLocation.AWAY){
                    telemetry.addLine("");
                    telemetry.addLine("Parking is AWAY");
                }
                if (parkLocation == ParkLocation.CLOSE){
                    telemetry.addLine("");
                    telemetry.addLine("Parking is CLOSE");
                }
            }
            //telemetry.addLine("Press DPAD-LEFT for going through middle gate, Press DPAD-RIGHT for going through red gate");
            if (gateLocation == GateLocation.MIDDLE && willPark){
                telemetry.addLine("");
                telemetry.addLine("Going through MIDDLE TRUSS");
            }
            if (gateLocation == GateLocation.TEAM && willPark){
                telemetry.addLine("");
                telemetry.addLine("Going through RED TRUSS");
            }

            if (willDropYellow){
                telemetry.addLine("");
                telemetry.addLine("WILL drop Yellow Pixel");
            }
            if (!willDropYellow){
                telemetry.addLine("");
                telemetry.addLine("WILL NOT drop Yellow Pixel");
                telemetry.addLine("");
                if (willPark){
                    telemetry.addLine("WILL park");
                }
                if (!willPark){
                    telemetry.addLine("WILL NOT park");
                }
            }
            telemetry.addLine("");
            telemetry.update();
            dashboardTelemetry.update();

            if (willDropYellow){
                willPark = true;
            }

            if (!willPark){
                gateLocation = GateLocation.TEAM;
            }

            if (gamepad1.a){
                if (delay < 5){
                    delay += 0.5;
                }
            }
            if (gamepad1.b){
                if (delay > 0){
                    delay -= 0.5;
                }
            }
            if (gamepad1.dpad_up){
                parkLocation = ParkLocation.AWAY;
            }
            if (gamepad1.dpad_down){
                parkLocation = ParkLocation.CLOSE;
            }
            if (gamepad1.dpad_left){
                gateLocation = GateLocation.MIDDLE;
            }
            if (gamepad1.dpad_right){
                gateLocation = GateLocation.TEAM;
            }

            if (gamepad1.back){
                willDropYellow = !willDropYellow;
            }

            if (gamepad1.y){
                willPark = true;
            }

            if (gamepad1.x){
                willPark = false;
            }

            sleep(70);
        }

        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();

        // Purple Pixel (first pixel) on floor to be pushed
        // Yellow Pixel (second pixel) in cassette
        propDirectionID = PropDirection.RIGHT;
        pick();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineToConstantHeading(new Vector2d(23.00, -39), Math.toRadians(90.00))
                        .stopAndAdd(telemetryPacket -> {
                            drop();
                            return false;
                        })
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(39.00, -42), Math.toRadians(90))
                        .splineToSplineHeading(new Pose2d(42.00, -36.78, Math.toRadians(0.00)), Math.toRadians(180.00), null, new ProfileAccelConstraint(-20, 20))
                        .build()
        );

        readAprilTags();
        dropYellowPixel();


        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setTangent(Math.toRadians(-90))
                        .splineToConstantHeading(new Vector2d(48.85, -56.50), Math.toRadians(0.00))
                        .splineToSplineHeading(new Pose2d(60.00, -57.94, Math.toRadians(90.00)), Math.toRadians(90.00), null, new ProfileAccelConstraint(-20, 20))
                        .build()
        );




    }
}


