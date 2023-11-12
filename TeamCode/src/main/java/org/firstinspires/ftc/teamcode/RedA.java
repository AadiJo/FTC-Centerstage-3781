package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

// A Side is opposite to backdrop
@Autonomous(name = "Red A")
public class RedA extends LinearOpMode {

    OpenCvWebcam webcam = null;
    double targetTicks;
    CRServo arm;
    Servo clawR;
    Servo clawL;
    private IMU imu;

    enum PropDirection{
        LEFT,
        RIGHT,
        MIDDLE
    }

    final double startXPos = 71.15;
    final double startYPos = -35.75;

    RedA.PropDirection propDirectionID;

    examplePipeline pipeline;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack;



    Orientation angles = new Orientation();
    class examplePipeline extends OpenCvPipeline {

        int ColorToExtract;
        int position;
        int numOfTimesRPT = 0;
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfinal;
        double rightavgfinal;
        double midavgfinal;

        Mat output = new Mat();
        Scalar rectColor =  new Scalar(225,0,0); //225, 0, 0
        //    OpenCvPipeline examplePipeline;
        public examplePipeline(int R1B2){

            ColorToExtract = R1B2;
        }

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            Rect leftRect = new Rect(1,1,213,359);
            Rect rightRect = new Rect(426,1, 213, 359);
            Rect middleRect = new Rect(213,1,213,359);

            input.copyTo(output);
            Imgproc.rectangle(output,leftRect,rectColor,4);
            Imgproc.rectangle(output,rightRect,rectColor,4);
            Imgproc.rectangle(output,middleRect,rectColor,4);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop =YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop,ColorToExtract);
            Core.extractChannel(rightCrop, rightCrop,ColorToExtract);
            Core.extractChannel(midCrop,midCrop,ColorToExtract);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfinal= leftavg.val[0];
            rightavgfinal= rightavg.val[0];
            midavgfinal = midavg.val[0];

//        valuesOfFinal = leftavgfinal+" "+rightavgfinal+" "+midavgfinal;

            if (leftavgfinal>midavgfinal && leftavgfinal>rightavgfinal){
                position = 1;

            }
            if (midavgfinal > leftavgfinal && midavgfinal > rightavgfinal){
                position = 2;
            }
            if(rightavgfinal > leftavgfinal && rightavgfinal > midavgfinal){
                position = 3;
            }






            return output;

        }

    }

    private void turn(double angle){



        double currentHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

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


    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
        leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
        rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
        rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");

        imu = (IMU) hardwareMap.get("imu");
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(0)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);
        pipeline = new examplePipeline(1);
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

    private void strafe(double inches){

        // TODO Fine tune inPerTick on field

        // + = left
        // - = right

        telemetry.addData("Current Y", drive.pose.position.y);
        telemetry.update();
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(drive.pose.position.x, inches + drive.pose.position.y))
                        .build()
        );

        telemetry.addData("Final Y", drive.pose.position.y);
        telemetry.update();
    }

    private void old_dropFirstPxl(MecanumDrive drive, int propDirectionID, double l_turn, double r_turn){
        // Find team prop happens BEFORE function is called
        telemetry.addData("L Turn", l_turn);
        telemetry.addData("R Turn", r_turn);
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        drive.updatePoseEstimate();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(-35.61, -35.47), Math.toRadians(0.00))
                        .build()
        );

//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(-35.61, -35.47), Math.toRadians(0.00))
//                        .splineTo(new Vector2d(-35.61, -35.47), l_turn)
//                        .waitSeconds(1)
//                        .splineTo(new Vector2d(-35.61, -35.47), -l_turn)
//                        //.lineToXConstantHeading(-71.15)
//                        .build()
//        );



        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        if (propDirectionID == 0){
            // LEFT
            telemetry.addLine("left");
            telemetry.addData("IMU Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Pose Heading", drive.pose.heading);
            telemetry.update();
            telemetry.update();

            drive.updatePoseEstimate();

//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            .waitSeconds(2)
//                            //.turn(l_turn)
//                            .splineTo(new Vector2d(-35.61, -35.47), l_turn)
//                            .waitSeconds(1)
//                            .build()
//            );

            drive.updatePoseEstimate();

            telemetry.addData("Pose Heading", drive.pose.heading);
            telemetry.addData("IMU Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();

//            Actions.runBlocking(
//                    drive.actionBuilder(drive.pose)
//                            //.turn(-l_turn)
//                            .splineTo(new Vector2d(-35.61, -35.47), -l_turn)
//                            .lineToXConstantHeading(-71.15)
//                            .build()
//            );

            telemetry.addData("Pose Heading", drive.pose.heading);
            telemetry.addData("IMU Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();



        }else if (propDirectionID == 3){
            // RIGHT
            telemetry.addLine("Right");
            telemetry.update();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(r_turn)
                            .build()
            );
        }else if (propDirectionID == 2){
            // MIDDLE
            telemetry.addLine("middle");
            telemetry.update();

        }
    }

    private void pick(){
        clawR.setPosition(0.5); // 1
        clawL.setPosition(0.4);//0
    }

    private void drop(){
        clawR.setPosition(0.3); // 0.2 // 0
        clawL.setPosition(0.85);//1
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
            propDirectionID = RedA.PropDirection.LEFT;
        }else if (propNumID == 2){
            propDirectionID = RedA.PropDirection.MIDDLE;
        }else if (propNumID == 3){
            propDirectionID = RedA.PropDirection.RIGHT;
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



        drive.updatePoseEstimate();

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        if (propDirectionID == RedA.PropDirection.LEFT){
            moveBot(50);
            turn(90);
            //moveBot(2);
            setupOuttakeFirstPxl();
            drop();
            //moveBot(-2);
            resetOuttakeFirstPxl();
            sleep(1000);
            //turn(-85);

        }else if (propDirectionID == RedA.PropDirection.RIGHT){
            moveBot(40);
            turn(-90);
//            moveBot(2);
            setupOuttakeFirstPxl();
            drop();
//            moveBot(-2);
            resetOuttakeFirstPxl();
            sleep(1000);
            //turn(85);

        }else if (propDirectionID == RedA.PropDirection.MIDDLE){
            // TODO
            moveBot(45);
            drop();
        }
    }

    private void setupForPxlTwo(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(startXPos - 1.5, startYPos))
                        .build()
        );
    }
    private void dropSecondPxl(){
        goToBackdrop();
        dropPxlTwo();
    }

    private void goToBackdrop(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(59.84, 33.02))
                        .strafeTo(new Vector2d(32.34, 33.02))
                        .build()
        );

        turn(85); // Outtake faces backdrop
        sleep(500);
    }
    private void dropPxlTwo(){
        // TODO outtake

    }

    private void setupForLoops(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(8.40, -53.03), Math.toRadians(-77.99))
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
                        .splineTo(new Vector2d(3.03, -5.43), Math.toRadians(93.81))
                        .splineTo(new Vector2d(26.16, 36.60), Math.toRadians(130.60))
                        .splineTo(new Vector2d(33.92, 49.00), Math.toRadians(90.00))
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
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(19.95, 50.87), Math.toRadians(127.87))
//                        .build()
//
//        );
        if (propDirectionID == RedA.PropDirection.LEFT || propDirectionID == RedA.PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(startXPos + 78, startYPos))
                            .build()

            );
        }else if(propDirectionID == RedA.PropDirection.MIDDLE){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(startXPos + 35, startYPos))
                            .strafeToConstantHeading(new Vector2d(startXPos + 40, startYPos + 20))
                            .strafeToConstantHeading(new Vector2d(startXPos + 84, startYPos + 20))
                            .strafeToConstantHeading(new Vector2d(startXPos + 84, startYPos))
                            .build()

            );
    }
    }

    private void park(){

        if (propDirectionID == PropDirection.LEFT){
            turn(-90);
        }

        if (propDirectionID == PropDirection.RIGHT){
            turn(90);
        }

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(startXPos + 100, startYPos - 200))
                        .build()

        );
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        initialize();

        clawR = hardwareMap.servo.get("clawR");
        clawL = hardwareMap.servo.get("clawL");

        telemetry.addLine("left " + pipeline.leftavgfinal);
        telemetry.addLine("right " + pipeline.rightavgfinal);
        telemetry.addLine("middle" + pipeline.midavgfinal);


        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();

        pick();
        dropFirstPxl();
//        setupForPxlTwo();
//        dropSecondPxl();
//        setupForLoops();
//        // loop till T - 5
//        while (opModeIsActive() && time.seconds() < 25){
//            pickAndDropWhitePxl();
//        }
//        // end loop
        setupForPark();
        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


