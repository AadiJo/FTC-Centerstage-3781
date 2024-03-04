package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class DetectionPipeline extends OpenCvPipeline {

    int ColorToExtract;
    int position;
    Mat YCbCr = new Mat();
    Mat leftCrop;
    Mat rightCrop;
    Mat midCrop;
    double leftavgfinal;
    double rightavgfinal;
    double midavgfinal;

    Mat output = new Mat();
    Scalar rectColor =  new Scalar(225,0,0,50); //225, 0, 0
    //    OpenCvPipeline examplePipeline;

    public DetectionPipeline(int R1B2){

        ColorToExtract = R1B2;
    }

    @Override
    public Mat processFrame(Mat input) {

        Imgproc.cvtColor(input, YCbCr, Imgproc.COLOR_RGB2YCrCb);


//        Rect leftRect = new Rect(1,1,213,359);
        Rect leftRect = new Rect(110,65,100,123);
        Rect middleRect = new Rect(323,45,90,110);
        Rect rightRect = new Rect(536,55, 100, 123);
//        Rect middleRect = new Rect(213,1,213,359);


        input.copyTo(output);
        Imgproc.rectangle(output,leftRect,rectColor,1);
        Imgproc.rectangle(output,rightRect,rectColor,1);
        Imgproc.rectangle(output,middleRect,rectColor,1);

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