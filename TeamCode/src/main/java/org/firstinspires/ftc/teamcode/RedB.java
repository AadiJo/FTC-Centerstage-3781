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

// B side is the side with the backdrop
@Autonomous(name = "Red B")
public class RedB extends LinearOpMode {

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
    final double startYPos = 12.52;

    examplePipeline pipeline;

    MecanumDrive drive;

    DcMotorEx leftFront, leftBack, rightFront, rightBack;



    Orientation angles = new Orientation();

    class examplePipeline extends OpenCvPipeline {

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

        // TODO Change Color to Blue
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

            Core.extractChannel(leftCrop, leftCrop,1);
            Core.extractChannel(rightCrop, rightCrop,1);
            Core.extractChannel(midCrop,midCrop,1);

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
        pipeline = new examplePipeline();
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

    private PropDirection findTeamProp(){
        // TODO Find team prop
        //final int propDirectionID = pipeline.position;
        PropDirection propDirectionID = PropDirection.RIGHT;
        return propDirectionID;
    }
    private void dropFirstPxl(){
        PropDirection propID = findTeamProp();
        telemetry.addData("PropDirectionID", propID);
        telemetry.update();
        telemetry.addData("Heading", drive.pose.heading);
        dropPxlOne(propID);

    }

    private void dropPxlOne(PropDirection propDirectionID){
        // Find team prop happens BEFORE function is called
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        drive.updatePoseEstimate();

        moveBot(36);

        drive.updatePoseEstimate();

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        if (propDirectionID == PropDirection.LEFT){
            turn(90);
            sleep(1000);
            turn(-90);

        }else if (propDirectionID == PropDirection.RIGHT){
            turn(-90);
            sleep(1000);
            turn(90);

        }else if (propDirectionID == PropDirection.MIDDLE){
            // TODO
        }
    }

    private void setupForPxlTwo(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeTo(new Vector2d(startXPos + 5, startYPos))
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
                        .strafeTo(new Vector2d(40.87, 47.16))
                        .build()
        );

        turn(90); // Outtake faces backdrop
        sleep(500);
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
                        .splineTo(new Vector2d(10.57, -53.96), Math.toRadians(268.59))
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
                        .splineToConstantHeading(new Vector2d(10.57, -53.96), Math.toRadians(268.59))
                        .splineToConstantHeading(new Vector2d(9.53, -3.87), Math.toRadians(-86.93))
                        .splineToConstantHeading(new Vector2d(22.32, 38.50), Math.toRadians(-17.72))
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
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(58.70, 49.22), Math.toRadians(35.34))
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

        final double l_turn = Math.toRadians(90); // Due to turning error, making manual adjustment
        final double r_turn = Math.toRadians(270);

        telemetry.addData("L Turn", l_turn);
        telemetry.addData("Turn Error", Math.abs(Math.toRadians(90) - l_turn));
        telemetry.update();

        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();


        dropFirstPxl();
        setupForPxlTwo();
        dropSecondPxl();
        setupForLoops();
        // loop till T - 5
        while (opModeIsActive() && time.seconds() < 25){
            pickAndDropWhitePxl();
        }
        // end loop
        setupForPark();
        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


