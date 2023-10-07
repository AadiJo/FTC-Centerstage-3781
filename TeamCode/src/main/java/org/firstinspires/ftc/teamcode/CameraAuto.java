package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.openftc.easyopencv.*;
import org.opencv.core.Mat;

@Autonomous(name = "AutoCamera", group = "Pirhos")
public class CameraAuto extends LinearOpMode {
    OpenCvCamera webcam;





    @Override
    public void runOpMode() throws InterruptedException {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID","id",hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);

        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
        examplePipeline x = new examplePipeline();
        webcam.setPipeline(x);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int n){

            }


        });

        waitForStart();

        telemetry.addData("left avg: ", x.returnLeftAvg());


    }
}
