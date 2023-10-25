package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.Mat;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.TankDrive;


@Autonomous
public class AutoTest extends LinearOpMode {

    OpenCvWebcam webcam = null;
    double targetTicks;
    CRServo arm;
    Servo clawR;
    Servo clawL;
    private BHI260IMU imu;
    Orientation angles = new Orientation();

    class examplePipeline extends OpenCvPipeline {

        byte position;
        Mat YCbCr = new Mat();
        Mat leftCrop;
        Mat rightCrop;
        Mat midCrop;
        double leftavgfinal;
        double rightavgfinal;
        double midavgfinal;

        OpenCvCamera webcam = null;

        Mat output = new Mat();
        Scalar rectColor =  new Scalar(225,0,0); //225, 0, 0
//    OpenCvPipeline examplePipeline;

        @Override
        public Mat processFrame(Mat input) {

            Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


            Rect leftRect = new Rect(1,1,213,359);
            Rect rightRect = new Rect(426,1, 213, 359);
            Rect middleRect = new Rect(213,179,213,180);

            input.copyTo(output);
            Imgproc.rectangle(output,leftRect,rectColor,4);
            Imgproc.rectangle(output,rightRect,rectColor,4);
            Imgproc.rectangle(output,middleRect,rectColor,4);

            leftCrop = YCbCr.submat(leftRect);
            rightCrop = YCbCr.submat(rightRect);
            midCrop =YCbCr.submat(middleRect);

            Core.extractChannel(leftCrop, leftCrop,2);
            Core.extractChannel(rightCrop, rightCrop,2);
            Core.extractChannel(midCrop,midCrop,2);

            Scalar leftavg = Core.mean(leftCrop);
            Scalar rightavg = Core.mean(rightCrop);
            Scalar midavg = Core.mean(midCrop);

            leftavgfinal= leftavg.val[0];
            rightavgfinal= rightavg.val[0];
            midavgfinal = midavg.val[0];

            if ((leftavgfinal)<midavgfinal && leftavgfinal < rightavgfinal){
                position = 1;

            }
            else if ((midavgfinal)<leftavgfinal && midavgfinal < rightavgfinal){
                position = 2;

            }
            else if(rightavgfinal<leftavgfinal && rightavgfinal<midavgfinal){ position = 3;}


            return output;

        }

    }


    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(66.78, -37.02, Math.toRadians(181.10)));

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);

        examplePipeline pipeline = new examplePipeline();

        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640, 360, OpenCvCameraRotation.UPRIGHT);
            }

            public void onError(int n) {

            }

        });

        telemetry.addLine("left " + pipeline.leftavgfinal);
        telemetry.addLine("right " + pipeline.rightavgfinal);
        telemetry.addLine("middle" + pipeline.midavgfinal);
        telemetry.update();

        waitForStart();

        time.reset();

        final int propDirectionID = pipeline.position;

        telemetry.addData("PropDirectionID", propDirectionID);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .turn(90)
                        //.splineTo(new Vector2d(2.33, -37.73), Math.toRadians(124.11))
                        //.splineTo(new Vector2d(2.05, -3.60), Math.toRadians(81.61))
                        //.splineTo(new Vector2d(0.21, 55.08), Math.toRadians(90.00))
                        .build()
        );

        telemetry.addData("Heading", drive.pose.heading);

        if (propDirectionID == 1){
            // LEFT
            telemetry.addLine("left");
            telemetry.update();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(-36.50, -36.50), Math.toRadians(90.00))
                            .build()
            );




        }

        else if (propDirectionID == 3){
            // RIGHT
            telemetry.addLine("Right");
            telemetry.update();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(90)
                            .build()
            );

        }

        else if (propDirectionID == 2){
            // MIDDLE
            telemetry.addLine("middle");
            telemetry.update();

        }


    }
}


