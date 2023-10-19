package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

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

@Autonomous(name = "AutoCamera", group = "Pirhos")
public class CameraAuto extends LinearOpMode {
    OpenCvWebcam webcam = null;



    @Override
    public void runOpMode() throws InterruptedException {
        try {
            WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam_1");

            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);

            //webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            examplePipeline x = new examplePipeline();

            webcam.setPipeline(x);
        }
        catch(Exception e){
            telemetry.addLine(e.getMessage());
            telemetry.update();
        }

        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int n){

            }


        });

        waitForStart();




    }
}
class examplePipeline extends OpenCvPipeline{
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    double leftavgfinal;
    double rightavgfinal;
    Mat output = new Mat();
    Scalar rectColor =  new Scalar(255.0, 0.0, 0.0);
    OpenCvPipeline examplePipeline;

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


        Rect leftRect = new Rect(1,1,319,359);
        Rect rightRect = new Rect(320,1, 319, 359);

        input.copyTo(output);
        Imgproc.rectangle(output,leftRect,rectColor,4);
        Imgproc.rectangle(output,rightRect,rectColor,4);

        leftCrop = YCbCr.submat(leftRect);
        rightCrop = YCbCr.submat(rightRect);

        Core.extractChannel(leftCrop, leftCrop,2);
        Core.extractChannel(rightCrop, rightCrop,2);

        Scalar leftavg = Core.mean(leftCrop);
        Scalar rightavg = Core.mean(rightCrop);
        leftavgfinal= leftavg.val[0];
        rightavgfinal= rightavg.val[0];



        return output;

    }

}
