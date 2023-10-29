package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.openftc.easyopencv.OpenCvCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.opencv.core.Core;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.*;
import org.opencv.core.Mat;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "Blue A")
public class BlueA extends LinearOpMode {

    OpenCvWebcam webcam = null;
    double targetTicks;
    CRServo arm;
    Servo clawR;
    Servo clawL;
    private IMU imu;
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

    private void dropFirstPxl(MecanumDrive drive, int propDirectionID, double l_turn, double r_turn){
        // Find team prop happens BEFORE function is called
        telemetry.addData("L Turn", l_turn);
        telemetry.addData("R Turn", r_turn);
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(-35.61, -35.47), Math.toRadians(0.00))
                        .build()
        );

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        if (propDirectionID == 1){
            // LEFT
            telemetry.addLine("left");
            telemetry.addData("IMU Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.addData("Pose Heading", drive.pose.heading);
            telemetry.update();
            telemetry.update();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(l_turn)
                            .waitSeconds(1)
                            .build()
            );

            telemetry.addData("Pose Heading", drive.pose.heading);
            telemetry.addData("IMU Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();

            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .turn(-l_turn)
                            .lineToXConstantHeading(-71.15)
                            .build()
            );

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

    private void dropSecondPxl(MecanumDrive drive, double startXPos, double startYPos , double l_turn, double r_turn){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        // Back up
                        .lineToXConstantHeading(startXPos + 5)
                        // Go to backdrop
                        .strafeTo(new Vector2d(startXPos, 65))
                        .lineToYConstantHeading(65)
                        .lineToXConstantHeading(startXPos + 20)

                        .build()
        );

    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        final double startXPos = -71.15;
        final double startYPos = -35.75;

        imu = (IMU) hardwareMap.get("imu");
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(0)));
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

        final double l_turn = Math.toRadians(90); // Due to turning error, making manual adjustment
        final double r_turn = Math.toRadians(270);

        telemetry.addData("L Turn", l_turn);
        telemetry.addData("Turn Error", Math.abs(Math.toRadians(90) - l_turn));
        telemetry.update();

        waitForStart();
        time.reset();
        imu.resetYaw();

        //final int propDirectionID = pipeline.position;
        final int propDirectionID = 1;
        telemetry.addData("PropDirectionID", propDirectionID);
        telemetry.update();
        telemetry.addData("Heading", drive.pose.heading);

        dropFirstPxl(drive, propDirectionID, Math.toRadians(90), Math.toRadians(270));


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


