package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;


@Autonomous(name = "AutoCamera with three crops", group = "Pirhos")
public class CameraAuto extends LinearOpMode {
    OpenCvWebcam webcam = null;
    double targetTicks;
    CRServo arm;
    Servo clawR;
    Servo clawL;

private DcMotor frontLeft;
private DcMotor frontRight;
private DcMotor backLeft;
    private DcMotor backRight;
    DcMotor rightEncoder;
    DcMotor midEncoder;
    DcMotor leftEncoder;
 double ticksPerRevODO = 2048;
double circumOfODO = 1.89;
public void setPowerToAllMotorsTo(double speed){

    backLeft.setPower(speed);
    backRight.setPower(speed);
    frontRight.setPower(speed);
    frontLeft.setPower(speed);
}
public double getTargetTicks(double inches){
    return (inches / circumOfODO)*ticksPerRevODO/2;}
    @Override
    public void runOpMode() throws InterruptedException {
        ElapsedTime time = new ElapsedTime();

            //WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam_1");
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);

        backLeft = hardwareMap.get(DcMotor.class, "bckLF");
        backRight = hardwareMap.get(DcMotor.class, "bckRT");
        frontLeft = hardwareMap.get(DcMotor.class, "frntLF");
        frontRight = hardwareMap.get(DcMotor.class, "frntRT");
        leftEncoder = hardwareMap.get(DcMotor.class, "leftEncoder");
        rightEncoder = hardwareMap.get(DcMotor.class, "rightEncoder");
        midEncoder = hardwareMap.get(DcMotor.class, "frontEncoder");
        arm = hardwareMap.crservo.get("arm");
        clawR = hardwareMap.servo.get("clawR");
        clawL = hardwareMap.servo.get("clawL");
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        leftEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        midEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

//        webcam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);
            examplePipeline pipeline = new examplePipeline();

            webcam.setPipeline(pipeline);
            webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                webcam.startStreaming(640,360,OpenCvCameraRotation.UPRIGHT);
            }
            public void onError(int n){

            }


        });
        clawR.setPosition(1);
        clawL.setPosition(0);
        telemetry.addLine("left "+pipeline.leftavgfinal);
        telemetry.addLine("right "+pipeline.rightavgfinal);
        telemetry.addLine("middle"+pipeline.midavgfinal);
        telemetry.update();
        waitForStart();
        time.reset();
while (time.seconds() < 2){
    setPowerToAllMotorsTo(.2);
}
        setPowerToAllMotorsTo(0);

        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);//left movement is positive
        midEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        if(pipeline.position == 1) {
            telemetry.addLine("left");
            telemetry.update();}

        else if (pipeline.position == 3){
        telemetry.addLine("Right");
        telemetry.update();
        }

        else if(pipeline.position == 2) {
            telemetry.addLine("middle");
            telemetry.update();
            time.reset();
            while(time.seconds()<3){
                setPowerToAllMotorsTo(.4);
            }
            time.reset();
            while(time.seconds()<2){
                setPowerToAllMotorsTo(-.4);
            }
            setPowerToAllMotorsTo(0);
            time.reset();
            while(time.seconds() < 2.8){
                arm.setPower(1);
            }
            arm.setPower(0);
            clawR.setPosition(0);
            clawL.setPosition(1);


        }
        telemetry.addLine(""+pipeline.leftavgfinal);
        telemetry.addLine(""+pipeline.rightavgfinal);
        telemetry.addLine(""+pipeline.midavgfinal);

        telemetry.update();

        time.reset();
        while (!gamepad1.a){}







    }
}
