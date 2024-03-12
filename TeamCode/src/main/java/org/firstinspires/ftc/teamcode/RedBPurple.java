package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.concurrent.TimeUnit;

// B side is the side with the backdrop
// One hand in from OUTSIDE SEAM on LEFT
@Autonomous(name = "Red Backdrop Purple")
public class RedBPurple extends LinearOpMode {

    OpenCvWebcam webcam = null;
    // double targetTicks;
    private IMU imu;

    PropDirection propDirectionID;

    TrajectoryActionBuilder trajectoryActionBuilder;

    enum PropDirection{
        LEFT,
        RIGHT,
        MIDDLE
    }

    final double startXPos = 16;
    final double startYPos = -60;

    public double CST_UPPER_BOUND = 0;
    public double CST_LOWER_BOUND = 1;

    public Encoder par0, par1, perp;

    public double ARM_START_POS;

    public int ARM_BD_X_POS = 6011;

    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;
    public boolean CST_DOWN = false;
    public boolean CST_UP = false;

    DetectionPipeline pipeline;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack, armMotor;

    Servo claw, cassette, door;

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
            // TODO optimize: see if sleep can be removed
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
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
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
            if (cassette.getPosition() - 0.04 > CST_UPPER_BOUND){
                cassette.setPosition(cassette.getPosition() - 0.04);
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
            if (cassette.getPosition() + 0.04 < CST_LOWER_BOUND){
                cassette.setPosition(cassette.getPosition() + 0.04);
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
        sleep(50);
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
                moveCassetteUp(cassette);
            }else{
                moveCassetteDown(cassette);
            }

        }
        sleep(50);
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
                    //sleep(5);

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
                    //sleep(20);

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
        leftFront.setPower(-0.5);
        rightFront.setPower(-0.5);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    private void pick(){
        claw.setPosition(0.8);
    }

    private void drop(){
        claw.setPosition(0);
    }


    // PRGM STARTS HERE

    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
        leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
        rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
        rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
        claw = hardwareMap.servo.get("claw");
        par0 = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).par0;
        //par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"))); // leftEncoder
        par1 = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).par1;
        // par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntLF"))); // frntLF
        perp = new ThreeDeadWheelLocalizer(hardwareMap, MecanumDrive.PARAMS.inPerTick).perp;
        //perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));

        par0.setDirection(DcMotorSimple.Direction.FORWARD);
        par1.setDirection(DcMotorSimple.Direction.REVERSE);
        perp.setDirection(DcMotorSimple.Direction.REVERSE);
        cassette = hardwareMap.servo.get("cassette");
        door = hardwareMap.servo.get("door");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



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
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(90)));
        trajectoryActionBuilder = drive.actionBuilder(drive.pose);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new DetectionPipeline(1);
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

                            .strafeToLinearHeading(new Vector2d(29.5, -28), Math.toRadians(180), new TranslationalVelConstraint(70), new ProfileAccelConstraint(-70, 70))//was 24, 24// why was this + 24 if it doesnt go up in y value
//                            .splineToSplineHeading(new Pose2d(35, 35, Math.toRadians(180)), Math.toRadians(180))
                            .build());
            // TODO optimize: remove sleep
            sleep(50);
            if (propDirectionID == PropDirection.RIGHT){
                drop();
                resetOuttakeFirstPxl();

            }
//
            if (propDirectionID == PropDirection.LEFT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(7, drive.pose.position.y))
//                                .splineToConstantHeading(new Vector2d(12.5, 35), Math.toRadians(0))
                                .build()
                );

                drop();
            }

        }else if (propDirectionID == PropDirection.MIDDLE){
            telemetry.addData("DIRECTION", propDirectionID);
            telemetry.update();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y + 28), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-30, 30))

                            .build()
            );
            drop();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 10))

                            .build()
            );

        }

    }

    void setupPark(){
        if (propDirectionID == PropDirection.LEFT || propDirectionID == PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(38, -28))
                            .build()
            );
        }
    }
    private void park(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(new Vector2d(57.23, -62.34), Math.toRadians(90))
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

        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        ARM_START_POS = armMotor.getCurrentPosition();

//        final VisionPortalStreamingOpMode.CameraStreamProcessor processor = new VisionPortalStreamingOpMode.CameraStreamProcessor();
//
//        new VisionPortal.Builder()
//                .addProcessor(processor)
//                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .build();
//
//        FtcDashboard.getInstance().startCameraStream(processor, 45);

        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();
        pick();

        // Purple Pixel (first pixel) on floor to be pushed
        // Yellow Pixel (second pixel) in cassettex`
        cassette.setPosition(1);
//        pick();
//        strafeBot(24);
        //moveBot(-24);
        strafeBot(2);
        // TODO optimize: remove sleep
        sleep(100);
        dropFirstPxl();
        setupPark();
        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


