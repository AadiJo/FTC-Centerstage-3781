import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import java.util.List;
import java.util.concurrent.TimeUnit;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
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

public class SimpleAprilTags extends LinearOpMode {
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


        @Override public void runOpMode() {
            boolean targetFound = false;
            double drive = 0;
            double strafe = 0;
            double turn = 0;

            //init??

            leftFrontDrive = hardwareMap.get(DcMotor.class, "leftfront_drive");
            rightFrontDrive = hardwareMap.get(DcMotor.class, "rightfront_drive");
            leftBackDrive = hardwareMap.get(DcMotor.class, "leftback_drive");
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

            while (opModeIsActive()) {
                targetFound = false;
                desiredTag = null;

                List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
                for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
                    if ((detection.metadata != null)
                            && ((DESIRED_TAG_ID >= 0) || (detection.id == DESIRED_TAG_ID))) {
                        targetFound = true;
                        desiredTag = detection;
                        break;

                        //look through the tags and find the matching one
                        //if found target, then drive to it automatically
                    }

            }

                if (targetFound) {
                    telemetry.addData(">","Left-Bumper to Drive to Target");
                    telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                    telemetry.addData("Range",  "%5.1f inches", desiredTag.ftcPose.range);
                    telemetry.addData("Bearing","%3.0f degrees", desiredTag.ftcPose.bearing);
                    telemetry.addData("Yaw","%3.0f degrees", desiredTag.ftcPose.yaw);
                } else {
                    telemetry.addData(">","Drive using joystick");
                }

                if (gamepad1.left_bumper && targetFound) {
                    //drive to the target by itself

                    double  FixRoll = (desiredTag.ftcPose.range - distance_to_target);
                    double  FixPitch = desiredTag.ftcPose.bearing;
                    double  FixYaw = desiredTag.ftcPose.yaw;


                    drive  = Range.clip(FixRoll * gain_speed, -max_speed, max_speed);
                    turn   = Range.clip(FixPitch * gain_turn, -max_turn, max_turn) ;
                    strafe = Range.clip(-FixYaw * gain_strafe, -gain_strafe, gain_strafe);

                    telemetry.addData("Auto","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);

                } else {

                    // drive using manual POV Joystick mode.  Slow things down to make the robot more controlable.
                    drive  = -gamepad1.left_stick_y  / 2.0;  // Reduce drive rate to 50%.
                    strafe = -gamepad1.left_stick_x  / 2.0;  // Reduce strafe rate to 50%.
                    turn   = -gamepad1.right_stick_x / 3.0;  // Reduce turn rate to 33%.
                    telemetry.addData("Manual","Drive %5.2f, Strafe %5.2f, Turn %5.2f ", drive, strafe, turn);
                }
                telemetry.update();

                // Apply desired axes motions to the drivetrain.
                moveRobot(drive, strafe, turn);
                sleep(10);
                }
                }

        public void moveRobot(double x, double y, double yaw) {
            // Calculate wheel powers.
            double leftFrontPower    =  x -y -yaw;
            double rightFrontPower   =  x +y +yaw;
            double leftBackPower     =  x +y -yaw;
            double rightBackPower    =  x -y +yaw;

            double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            if (max > 1.0) {
                leftFrontPower /= max;
                rightFrontPower /= max;
                leftBackPower /= max;
                rightBackPower /= max;
            }

            // Send powers to the wheels.
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
        }

        private void initAprilTag() {

            aprilTag = new AprilTagProcessor.Builder().build();


            if (USE_WEBCAM) {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                        .addProcessor(aprilTag)
                        .build();
            } else {
                visionPortal = new VisionPortal.Builder()
                        .setCamera(BuiltinCameraDirection.BACK)
                        .addProcessor(aprilTag)
                        .build();
            }
        }


        private void    setManualExposure(int exposureMS, int gain) {
            // Wait for the camera to be open, then use the controls

            if (visionPortal == null) {
                return;
            }


            if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
                telemetry.addData("Camera", "Waiting");
                telemetry.update();
                while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                    sleep(20);
                }
                telemetry.addData("Camera", "Ready");
                telemetry.update();
            }

            // Set camera controls unless we are stopping.
            if (!isStopRequested())
            {
                ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
                if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                    exposureControl.setMode(ExposureControl.Mode.Manual);
                    sleep(50);
                }
                exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
                sleep(20);
                GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
                gainControl.setGain(gain);
                sleep(20);
            }
        }
            }


        }
