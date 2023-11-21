import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.List;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

public class SimpleAprilTags extends OpMode {
private AprilTagProcessor aprilTagProcessor;
private VisionPortal visionPortal;
private static final boolean USE_WEBCAM = true;
    private static final int DESIRED_TAG_ID = 0;
    private AprilTagProcessor aprilTag;
    private org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;

    @TeleOp(name = "April Tag Test", group = "current")
public class AprilTagDetection extends LinearOpMode {
    OpenCvWebcam webcam = null;

        //units are pixels
        //edit calibration for our camera
        double fx = 578.272;
        double fy = 578.272;
        double cx = 402.145;
        double cy = 221.506;


        private DcMotor leftFrontDrive = null;
        private DcMotor rightFrontDrive = null;
        private DcMotor leftBackDrive = null;
        private DcMotor rightBackDrive = null;

        final double distance_to_target = 12.0;
        final double gain_speed  =  0.02  ;
        final double gain_strafe =  0.015 ;
        final double gain_turn =  0.01  ;

        final double max_speed = 0.5;
        final double max_strafe = 0.5;
        final double max_turn = 0.3;

        private static final boolean webcam_on = true;
        private static final int DESIRED_TAG_ID = 0;     // Choose the tag you want to approach or set to -1 for ANY tag.
        private AprilTagProcessor aprilTag;
        private org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;


        @Override public void runOpMode(){
        boolean targetFound= false;
        double  drive = 0;
        double  strafe = 0;
        double  turn = 0;

        initAprilTag();

        leftFrontDrive  = hardwareMap.get(DcMotor.class, "leftfront_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "leftback_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "rightback_drive");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);
        int tagsize;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder = null;
        AprilTagProcessor myAprilTagProcessor = null;
        //Create a new AprilTagProcessor Builder = new AprilTagProcessor.Builder();
        myAprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());
        myAprilTagProcessorBuilder.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary());

        aprilTagProcessor = AprilTagProcessor.easyCreateWithDefaults();

        AprilTagDetection myAprilTagDetection = null;

        int myAprilTagIdCode = myAprilTagDetection.id;

        VisionPortal myVisionPortal = null;

        // Enable or disable the AprilTag processor.
        myVisionPortal.setProcessorEnabled(myAprilTagProcessor, true);

        telemetry.addData("Camera preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            targetFound = false;
            desiredTag  = null;

        //look through the tags and find the matching one
        //if found target, then drive to it automatically
        }
    }






}
