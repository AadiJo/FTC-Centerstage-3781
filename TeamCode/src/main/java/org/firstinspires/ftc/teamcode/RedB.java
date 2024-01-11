package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// B side is the side with the backdrop
@Autonomous(name = "Red B")
public class RedB extends LinearOpMode {

    OpenCvWebcam webcam = null;
    Servo clawR;
    Servo clawL;
    private IMU imu;

    PropDirection propDirectionID;

    enum PropDirection{
        LEFT,
        RIGHT,
        MIDDLE
    }

    final double startXPos = 71.15;
    final double startYPos = 23.43;

    DetectionPipeline pipeline;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack;


    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
        leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
        rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
        rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");
        clawR = hardwareMap.servo.get("clawR");
        clawL = hardwareMap.servo.get("clawL");

        imu = (IMU) hardwareMap.get("imu");
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(180)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);
        pipeline = new DetectionPipeline(1);
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

    private void pick(){
        clawR.setPosition(0.5); // 1
        clawL.setPosition(0.4);//0
    }

    private void drop(){
        clawR.setPosition(0.2); // 0.2 // 0
        clawL.setPosition(0.85);//1
    }
    private void moveBot(double inches){
//        double ticksPerInch = MecanumDrive.PARAMS.inPerTick;
//        double ticks = inches * ticksPerInch;
//
//        double currentY = drive.pose.position.y;
//
//        while (drive.pose.position.y < (currentY + inches)){
//            telemetry.addData("Current Y", drive.pose.position.y);
//            telemetry.addData("Target Y", currentY + inches);
//            telemetry.addData("Current X", drive.pose.position.x);
//            telemetry.update();
//            leftFront.setPower(0.5);
//            leftBack.setPower(0.5);
//            rightFront.setPower(0.5);
//            rightBack.setPower(0.5);
//        }
//
//        leftFront.setPower(0);
//        leftBack.setPower(0);
//        rightFront.setPower(0);
//        rightBack.setPower(0);
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

    private void setupOuttakeFirstPxl(){
        leftFront.setPower(0.2);
        rightFront.setPower(0.2);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    private void resetOuttakeFirstPxl(){
        leftFront.setPower(-0.2);
        rightFront.setPower(-0.2);
        sleep(400);
        leftFront.setPower(0);
        rightFront.setPower(0);
    }

    private void findTeamProp(){
        // TODO Find team prop
        final int propNumID = pipeline.position;
        if (propNumID == 1){
            propDirectionID = RedB.PropDirection.LEFT;
        }else if (propNumID == 2){
            propDirectionID = RedB.PropDirection.MIDDLE;
        }else if (propNumID == 3){
            propDirectionID = RedB.PropDirection.RIGHT;
        }
    }
    private void dropFirstPxl(){
        findTeamProp();
        telemetry.addData("PropDirectionID", propDirectionID);
        telemetry.update();
        telemetry.addData("Heading", drive.pose.heading);
        dropPxlOne(propDirectionID);

    }
    private void dropPxlOne(PropDirection propDirectionID){
        // Find team prop happens BEFORE function is called
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        drive.updatePoseEstimate();

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        if (propDirectionID == PropDirection.LEFT || propDirectionID == PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            //.splineTo(new Vector2d(30, 30), Math.PI / 2)
                            //.splineTo(new Vector2d(60, 0), Math.PI)
//                        .splineTo(new Vector2d(-36.50, -36.50), Math.toRadians(0))
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x, drive.pose.position.y + 24))
                            .waitSeconds(0.2)
                            .strafeToConstantHeading(new Vector2d(drive.pose.position.x - 24, drive.pose.position.y + 24))
                            .waitSeconds(0.2)
                            .turn(Math.toRadians(-90))
                            .build());

            //turn(-90);

            if (propDirectionID == PropDirection.LEFT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 7))
                                .build()
                );

                drop();

            }

            if (propDirectionID == PropDirection.RIGHT){
                Actions.runBlocking(
                        drive.actionBuilder(drive.pose)
                                .strafeTo(new Vector2d(drive.pose.position.x, drive.pose.position.y - 28))
                                .build()
                );

                drop();
            }

        }else if (propDirectionID == PropDirection.MIDDLE){
            // TODO
            moveBot(26);
            drop();
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeTo(new Vector2d(drive.pose.position.x + 12, drive.pose.position.y))
                            .turn(Math.toRadians(-90))
                            .build()
            );


        }

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        // Right in front of backdrop
                        .strafeToConstantHeading(new Vector2d(36.36, 39.6))
                        .turn(Math.toRadians(180))
                        .build()
        );


    }

    private void dropSecondPxl(){
        goToBackdrop(propDirectionID);
        dropPxlTwo();
    }

    private void goToBackdrop(PropDirection propDirectionID){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        // Right in front of backdrop
                        //.strafeToConstantHeading(new Vector2d(36.36, 39.6))
                        .strafeToLinearHeading(new Vector2d(36.36, 39.6), Math.toRadians(90))
                        // turn to face backdrop
                        //.turn(Math.toRadians(180))
                        .build()
        );
    }
    private void dropPxlTwo(){
        // TODO outtake

    }

    private void setupForLoops(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(11.49, 31.08), Math.toRadians(-87.49))
//                        .splineTo(new Vector2d(11.60, -56.02), Math.toRadians(266.86))
                        .splineTo(new Vector2d(22.32, 38.50), Math.toRadians(-17.72))
                        .splineTo(new Vector2d(9.53, -3.87), Math.toRadians(-86.93))
                        .splineTo(new Vector2d(10.57, -67.96), Math.toRadians(268.59))
                        // TODO Fine tune on field
                        .build()
        );

    }

    private void pickAndDropWhitePxl(){
        pickWhitePxl();
        dropWhitePxl();
    }

    private void pickWhitePxl(){
        // TODO intake
    }

    private void goToBackdropInLoop(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(11.49, 31.70), Math.toRadians(91.43))
//                        .splineToConstantHeading(new Vector2d(36.23, 51.49), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-3.03, -5.43), Math.toRadians(93.81))
                        .splineTo(new Vector2d(-26.16, 36.60), Math.toRadians(130.60))
                        .splineTo(new Vector2d(-33.92, 50.00), Math.toRadians(90.00))
                        .build()

        );
    }

    private void outtakeWhitePxl(){
        // TODO white pxl outtake
    }

    private void dropWhitePxl(){
        goToBackdropInLoop();
        outtakeWhitePxl();
    }

    private void setupForPark(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(17.68, 41.90), Math.toRadians(79.56))
                        .build()

        );
    }

    private void park(){
        if (propDirectionID == RedB.PropDirection.LEFT || propDirectionID == RedB.PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(startXPos + 23, startYPos))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(startXPos + 23, startYPos - 70))
                            .build()
            );
        }else if (propDirectionID == RedB.PropDirection.MIDDLE){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(startXPos + 12, startYPos))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(startXPos + 12, startYPos - 45))
                            .build()
            );
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        initialize();

        telemetry.addLine("left " + pipeline.leftavgfinal);
        telemetry.addLine("right " + pipeline.rightavgfinal);
        telemetry.addLine("middle" + pipeline.midavgfinal);


        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();

        pick();
        dropFirstPxl();
        dropSecondPxl();
//        setupForLoops();
//        // loop till T - 5
//        while (opModeIsActive() && time.seconds() < 25){
//            pickAndDropWhitePxl();
//        }
//        // end loop
//        setupForPark();
//        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


