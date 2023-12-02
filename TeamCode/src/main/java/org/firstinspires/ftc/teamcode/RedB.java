package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.roadrunner.MecanumDrive;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

// B side is the side with the backdrop
@Autonomous(name = "Red B")
public class RedB extends LinearOpMode {

    OpenCvWebcam webcam = null;
    double targetTicks;
    CRServo arm;
    Servo clawR;
    Servo clawL;
    Servo cassette;
    private IMU imu;

    RedB.PropDirection propDirectionID;

    enum PropDirection{
        LEFT,
        RIGHT,
        MIDDLE
    }

    final double startXPos = 71.15;
    final double startYPos = 12.52;

    DetectionPipeline pipeline;
    MecanumDrive drive;
    DcMotorEx leftFront, leftBack, rightFront, rightBack, armMotor;
    double armTicksPerRev, armStartPos, armDropPos, armTickPerDeg, armPickPos;
    double cstUnitPerDeg;
    double cstStartPos, cstPickPos, cstDropPos;
    void moveArm(double angle){
        // angle in degrees ^
        // cassette will maintain angle with respect to arm
        armMotor.setTargetPosition((int) (angle * armTickPerDeg));
        cassette.setPosition(-angle * cstUnitPerDeg);
    }
    void moveCst(double position){
        cassette.setPosition(position);
    }
    private void outtake() {
        // TODO outtake
        // Need to move arm, drop pxl, and get arm back to start position
        moveArm(armDropPos);

        // Moving bot slightly forward to let pixel fall
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToConstantHeading(new Vector2d(0, 0))
                        .build()
                // TODO find correct values for x and y
        );

        moveArm(armStartPos);

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
    private void pickWhitePxl(){
        // TODO intake
    }
    private void outtakeWhitePxl(){
        // TODO white pxl outtake
    }

    private void initialize(){
        leftFront = hardwareMap.get(DcMotorEx.class, "frntLF");
        leftBack = hardwareMap.get(DcMotorEx.class, "bckLF");
        rightBack = hardwareMap.get(DcMotorEx.class, "bckRT");
        rightFront = hardwareMap.get(DcMotorEx.class, "frntRT");
        // TODO armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");
//        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        cstDropPos = 50;
        armDropPos = 10;
        cstStartPos = 24;
        armStartPos = 24;
        cstUnitPerDeg = 1 / 300.0;
        cstPickPos = 25;
        armPickPos = 35;
        imu = (IMU) hardwareMap.get("imu");
        drive = new MecanumDrive(hardwareMap, new Pose2d(startXPos, startYPos, Math.toRadians(0)));
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("CameraMonitorViewID", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam_1"), cameraMonitorViewId);
        pipeline = new DetectionPipeline(1);
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

    private void dropFirstPxl(){
        findTeamProp();
        telemetry.addData("PropDirectionID", propDirectionID);
        telemetry.update();
        telemetry.addData("Heading", drive.pose.heading);
        dropPxlOne(propDirectionID);

    }
    private void findTeamProp(){
        // TODO Find team prop
        final int propNumID = pipeline.position;
        if (propNumID == 1){
            propDirectionID = RedB.PropDirection.LEFT;
        }else if (propNumID == 2){
            propDirectionID = RedB.PropDirection.MIDDLE;
        }else if (propNumID == 3){
            propDirectionID = RedB.PropDirection.RIGHT;
        }
    }
    private void dropPxlOne(PropDirection propDirectionID){
        // Find team prop happens BEFORE function is called
        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();

        drive.updatePoseEstimate();

        telemetry.addData("Heading", drive.pose.heading);
        telemetry.update();
        // TODO test on field and update all values
        if (propDirectionID == RedB.PropDirection.LEFT){
            moveBot(50);
            turn(90);
            //moveBot(2);
            setupOuttakeFirstPxl();
            //moveBot(-2);
            resetOuttakeFirstPxl();
            sleep(1000);
            //turn(-85);

        }else if (propDirectionID == RedB.PropDirection.RIGHT){
            moveBot(50);
            turn(-90);
//            moveBot(2);
            setupOuttakeFirstPxl();
//            moveBot(-2);
            resetOuttakeFirstPxl();
            sleep(1000);
            //turn(85);

        }else if (propDirectionID == RedB.PropDirection.MIDDLE){
            // TODO
            moveBot(42);
        }
    }

    private void setupForPxlTwo(){
        // TODO setupForPxlTwo()
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeTo(new Vector2d(startXPos - 1.5, startYPos))
//                        .build()
//        );
    }
    private void dropSecondPxl(){
        goToBackdrop();
        dropPxlTwo();
    }
    private void goToBackdrop(){
//        Actions.runBlocking(
//                drive.actionBuilder(drive.pose)
//                        .strafeTo(new Vector2d(59.84, 33.02))
//                        .strafeTo(new Vector2d(32.34, 33.02))
//                        .build()
//        );
//
//        turn(85); // Outtake faces backdrop
//        sleep(500);
        // TODO
    }
    private void dropPxlTwo(){
        outtake();

    }

    private void setupForLoops(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .splineTo(new Vector2d(11.49, 31.08), Math.toRadians(-87.49))
//                        .splineTo(new Vector2d(11.60, -56.02), Math.toRadians(266.86))
                        .splineTo(new Vector2d(22.32, 38.50), Math.toRadians(-17.72))
                        .splineTo(new Vector2d(9.53, -3.87), Math.toRadians(-86.93))
                        .splineTo(new Vector2d(10.57, -67.96), Math.toRadians(268.59))
                        // TODO Fine tune on field
                        .build()
        );

    }
    private void dropWhitePxl(){
        goToBackdropInLoop();
        outtakeWhitePxl();
    }
    private void goToBackdropInLoop(){

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
//                        .splineToConstantHeading(new Vector2d(11.49, 31.70), Math.toRadians(91.43))
//                        .splineToConstantHeading(new Vector2d(36.23, 51.49), Math.toRadians(90.00))
                        .splineTo(new Vector2d(-3.03, -5.43), Math.toRadians(93.81))
                        .splineTo(new Vector2d(-26.16, 36.60), Math.toRadians(130.60))
                        .splineTo(new Vector2d(-33.92, 50.00), Math.toRadians(90.00))
                        .build()

        );
    }

    private void setupForPark(){
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(17.68, 41.90), Math.toRadians(79.56))
                        .build()

        );
    }
    private void park(){
        if (propDirectionID == RedB.PropDirection.LEFT || propDirectionID == RedB.PropDirection.RIGHT){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(startXPos + 23, startYPos))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(startXPos + 23, startYPos - 70))
                            .build()
            );
        }else if (propDirectionID == RedB.PropDirection.MIDDLE){
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToConstantHeading(new Vector2d(startXPos + 12, startYPos))
                            .waitSeconds(0.5)
                            .strafeToConstantHeading(new Vector2d(startXPos + 12, startYPos - 45))
                            .build()
            );
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        ElapsedTime time = new ElapsedTime();

        initialize();

        telemetry.addLine("left " + pipeline.leftavgfinal);
        telemetry.addLine("right " + pipeline.rightavgfinal);
        telemetry.addLine("middle" + pipeline.midavgfinal);


        waitForStart();
        time.reset();
        imu.resetYaw();
        drive.updatePoseEstimate();
        // TODO set cassette to backdrop angle
        moveCst(cstDropPos);

        // Purple Pixel (first pixel) on floor to be pushed
        // Yellow Pixel (second pixel) in cassette

        dropFirstPxl();
        setupForPxlTwo();
        dropSecondPxl();
//        setupForLoops();
//        // loop till T - 5
//        while (opModeIsActive() && time.seconds() < 25){
//            pickAndDropWhitePxl();
//        }
//        // end loop
//        setupForPark();
        park();


        //dropSecondPxl(drive, startXPos, startYPos, l_turn, r_turn);


    }
}


