package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.ftc.Encoder;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

@TeleOp(name = "TeleOP", group = "Current")
public class FC extends LinearOpMode {

    public static class VARS{
        public double ticksPerRevDrone = 103.8;
        public double maxTicksPerSec = 1620 /60 * ticksPerRevDrone;
        public double droneRPM = .75 * 1620;

        double Kp = PIDConstants.Kp;
        double Ki = PIDConstants.Ki;
        double Kd = PIDConstants.Kd;

        double lastError = 0;
        double integralSum;
//        public double lastError = 0.009;

        ElapsedTime PIDTimer = new ElapsedTime();

        public boolean isOpen = true;
        public double dPadX;
        public double dPadY;
        public boolean dpPressed;
        public double dpStartSpeed = 0.2;
        public double dpSpeed = 0.2;
        public double dpMaxSpeed = 1;

        public boolean intake = false;

        public double ARM_TICKS_PER_REV = 384.5;
        public double ARM_START_POS;
        // RPM / 60 -> RPS / 1000 -> RPMS x 11.29 (degrees per rev)
        // 1 rotation is 270 degrees
        public double cassetteMoveIncrement = 0.05; //0.0818525/360 * CST_UNIT_PER_DEG;
        public double CST_UPPER_BOUND = 0;
        public double CST_LOWER_BOUND = 1;
        public double CST_INIT_POS = 0.7;

        double t0 = 0;// time in milliseconds
        double t1 = 0;
        double elapsedTime = 0;

        public int ARM_BD_L1_POS = Math.abs(-5477 + 65);
        public int ARM_BD_L2_POS = Math.abs(-4890 + 65);
        public int ARM_BD_L3_POS = Math.abs(-4303 + 65);
        public double CST_BD_L1_POS = 0.1;
        public double CST_BD_L2_POS = 0.1;
        public double CST_BD_L3_POS = 0.2;
        public boolean CST_DOWN = false;
        public boolean CST_UP = false;

        public int ARM_HALF_POS = Math.abs(4337 - 1425);
    }

    public static VARS VARS = new VARS();

    private void log(String caption, Object message){
        if (true){
            telemetry.addData(caption, message);
        }

    }

    void powerCassette(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            cassette.setPosition(cassette.getPosition());
        }else{
            cassette.setPosition(1);
        }
    }

    void moveCassetteDown(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() - 0.035 > VARS.CST_UPPER_BOUND){
                cassette.setPosition(cassette.getPosition() - 0.035);
                sleep(100);
            }else{
                cassette.setPosition(VARS.CST_UPPER_BOUND);
                sleep(100);
            }
        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }

    }

    void moveCassetteUp(Servo cassette){
        if (!Double.isNaN(cassette.getPosition())){
            if (cassette.getPosition() + 0.035 < VARS.CST_LOWER_BOUND){
                cassette.setPosition(cassette.getPosition() + 0.035);
                sleep(100);
            }else{
                cassette.setPosition(VARS.CST_LOWER_BOUND);
                sleep(100);
            }

        }else{
            cassette.setPosition(0.8);
            sleep(100);
        }
    }

    private void setArmPos(int position, DcMotorEx armMotor, Servo cassette, Encoder par0, Encoder par1, Encoder perp){
        sleep(50);
        double tolerance = 300;
        int par0Pos = par0.getPositionAndVelocity().position;
        int par1Pos = par1.getPositionAndVelocity().position;
        int perpPos = perp.getPositionAndVelocity().position;
        ElapsedTime time1 = new ElapsedTime(ElapsedTime.Resolution.SECONDS);
        time1.reset();
        while (armMotor.getCurrentPosition() != position && Math.abs(position - armMotor.getCurrentPosition()) > tolerance) {

            // obtain the encoder position
            double encoderPosition = armMotor.getCurrentPosition();
            // calculate the error
            double error = position - encoderPosition;
            // set motor power proportional to the error
            armMotor.setPower(error);
            if (position > encoderPosition){
                moveCassetteUp(cassette);
            }else{
                moveCassetteDown(cassette);
            }

        }
        sleep(50);
    }
    public double angleWrap(double radians){
        while(radians > Math.PI){
            radians -= 2 * Math.PI;
        }
        while(radians < -Math.PI){
            radians += 2 * Math.PI;
        }
        return radians;
    }
    public double PIDControl(double reference, double state) {
        VARS.integralSum = 0;
        double error = angleWrap(reference - state);
        telemetry.addData("Error: ", error);
        VARS.integralSum += error * VARS.PIDTimer.seconds();
        double derivative = (error - VARS.lastError) / (VARS.PIDTimer.seconds());
        VARS.lastError = error;
        VARS.PIDTimer.reset();
        return (error * VARS.Kp) + (derivative * VARS.Kd) + (VARS.integralSum * VARS.Ki);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        double targetHeading = Math.toRadians(0);
        double tolerance = 0.001;
        double multiplier = 0.7;
        Motor leftFront = new Motor(hardwareMap, "frntLF");
        Motor leftBack = new Motor(hardwareMap, "bckLF");
        Motor rightBack = new Motor(hardwareMap, "bckRT");
        Motor rightFront = new Motor(hardwareMap, "frntRT");
        Encoder leftEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).par0;
        //par0 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftEncoder"))); // leftEncoder
        Encoder rightEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).par1;
        // par1 = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntLF"))); // frntLF
        Encoder frontEncoder = new ThreeDeadWheelLocalizer(hardwareMap, org.firstinspires.ftc.teamcode.MecanumDrive.PARAMS.inPerTick).perp;
        //perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));OverflowEncoder frontEncoder = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frntRT")));

        Servo claw = hardwareMap.servo.get("claw");
        Servo cassette = hardwareMap.servo.get("cassette");
        CRServo flicker = hardwareMap.crservo.get("drone");
        Servo door = hardwareMap.servo.get("door");
        CRServo intake_L = hardwareMap.crservo.get("intakeL");
        CRServo intake_R = hardwareMap.crservo.get("intakeR");

        DcMotorEx pullMotor = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        DcMotorEx droneMotor = hardwareMap.get(DcMotorEx.class,"droneMotor");
        DcMotorEx armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        {
            leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
            droneMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            // armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        {
            leftBack.setInverted(true);
            rightBack.setInverted(true);
            rightFront.setInverted(true);
            pullMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftFront.setInverted(true);
            // cassette.setDirection(Servo.Direction.REVERSE);
            // armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        MecanumDrive drive = new MecanumDrive(
                leftFront, rightFront, leftBack, rightBack
        );

        double loopTime = 0.0;

        // initialize the IMU

        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        IMU imu = (IMU) hardwareMap.get("imu");
        imu.initialize(parameters);
        imu.resetYaw();


        // the extended gamepad object
        GamepadEx driverOp = new GamepadEx(gamepad1);

        VARS.ARM_START_POS = armMotor.getCurrentPosition();
        drive.driveFieldCentric(
                driverOp.getLeftX(),
                driverOp.getLeftY(),
                -PIDControl(targetHeading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)),
                imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                false
        );

        waitForStart();

        cassette.setPosition(1);

        while (!isStopRequested()) {
            double loop = System.nanoTime();

            if (driverOp.getRightX() != 0 || (Math.abs(VARS.lastError) <= tolerance) || gamepad1.b){
                PIDControl(targetHeading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                drive.driveFieldCentric(
                        driverOp.getLeftX() * multiplier,
                        driverOp.getLeftY() * multiplier,
                        driverOp.getRightX() * 0.5,
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                        false
                );
                targetHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            }else{ // if no input from right joystick
                drive.driveFieldCentric(
                        driverOp.getLeftX(),
                        driverOp.getLeftY(),
                        -PIDControl(targetHeading, imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), // negative because FTCLib input is inverted
                        imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                        false
                );
            }

            log("Heading", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
            log("Cassette", cassette.getPosition());
            log("Arm Ticks", armMotor.getCurrentPosition());
            log("Cassette Pos", cassette.getPosition());
            log("Par 0", leftEncoder.getPositionAndVelocity().position);
            log("Par 1", rightEncoder.getPositionAndVelocity().position);
            log("Perp", frontEncoder.getPositionAndVelocity().position);
//            telemetry.addData("hz", 1000000000  / (loop - loopTime));
            loopTime = loop;
            telemetry.addData("Last Error", VARS.lastError);
            telemetry.update();

            {

                if (gamepad1.back || imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) == 0.0) {
                    imu.initialize(parameters);
                    imu.resetYaw();
                    targetHeading = Math.toRadians(0);
                    gamepad1.rumble(1, 1, 200);
                }

                if (gamepad2.left_trigger > 0.5 && gamepad2.right_trigger > 0.5){
                    leftFront.set(0);
                    leftBack.set(0);
                    rightBack.set(0);
                    rightFront.set(0);
                    //droneMotor.setPower(-VARS.droneRPM / 1620);
                    droneMotor.setVelocity(-0.85 * VARS.maxTicksPerSec);
                    gamepad2.rumble(1, 1, 200);
                    telemetry.addLine("Drone Percent:" + droneMotor.getVelocity()/ VARS.maxTicksPerSec+"");
                    telemetry.update();
                    sleep(1000);
                    flicker.setPower(-1);
                    sleep(200);

                    //droneMotor.setPower(0);
                }
                else{
                    droneMotor.setPower(0);
                    droneMotor.setVelocity(0);
                    flicker.setPower(0);

                }
            }

            if (gamepad2.left_bumper) {
                claw.setPosition(0);

            }

            if (gamepad2.right_bumper){
                claw.setPosition(1);
            }

            if (gamepad1.a){
                multiplier = 0.35;
            }else{
                multiplier = 0.7;
            }

            if (gamepad2.left_stick_button){


                if (gamepad2.x){
                    // FIRST
                    //cassette.setPosition(VARS.CST_BD_L1_POS);
                    setArmPos((int) VARS.ARM_START_POS - VARS.ARM_BD_L1_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                    // arm ticks = -5477 start = -65

                }

                if (gamepad2.y){
                    // SECOND
                    //cassette.setPosition(VARS.CST_BD_L2_POS);
                    setArmPos((int) VARS.ARM_START_POS - VARS.ARM_BD_L2_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                }

                if (gamepad2.b){
                    // THIRD
                    //cassette.setPosition(VARS.CST_BD_L3_POS);
                    setArmPos((int) VARS.ARM_START_POS - VARS.ARM_BD_L3_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                }

                if (gamepad2.a){
                    // START
                    //cassette.setPosition(0.8);
                    setArmPos((int) VARS.ARM_START_POS, armMotor, cassette,leftEncoder, rightEncoder, frontEncoder);
                }


            }

            if (gamepad2.right_stick_button){
                if (VARS.intake){
                    intake_L.setPower(0);
                    intake_R.setPower(0);
                    VARS.intake = false;
                }else{
                    intake_L.setPower(1);
                    intake_R.setPower(1);
                    VARS.intake = true;
                }
            }

            if (gamepad2.dpad_up){
                armMotor.setPower(1);
                while (true){
                    if (!(Math.abs(armMotor.getCurrent(CurrentUnit.AMPS)) < 6)) break;
                    // waiting for stall before starting pull motor
                }
                pullMotor.setPower(-0.75);

            }else if (!gamepad2.y && !gamepad2.a && !gamepad2.dpad_down){
                armMotor.setPower(0);
                pullMotor.setPower(0);
            }

            if (gamepad2.dpad_down){
                armMotor.setPower(-1);
                pullMotor.setPower(0.75);

            }else if (!gamepad2.y && !gamepad2.a && !gamepad2.dpad_up){
                armMotor.setPower(0);
                pullMotor.setPower(0);
            }

            if (gamepad2.y) {
                // UP
                // opposite to arm
                if (armMotor.getCurrentPosition() > (VARS.ARM_START_POS - 6166)){
                    VARS.CST_DOWN = true;
                    armMotor.setPower(-1);
                    powerCassette(cassette);
                }

            }else if (!gamepad2.a){
                VARS.t0 = 0;
                VARS.CST_DOWN = false;
                armMotor.setPower(0);
                powerCassette(cassette);
            }

            if (gamepad2.a) {
                // DOWN
                if (!gamepad2.back){
                    if (armMotor.getCurrentPosition() < VARS.ARM_START_POS){
                        // opposite to arm
                        VARS.CST_UP = true;
                        armMotor.setPower(1);

                        powerCassette(cassette);
                    }
                }else{
                    VARS.ARM_START_POS = armMotor.getCurrentPosition();
                    // opposite to arm
                    VARS.CST_UP = true;
                    armMotor.setPower(1);

                    powerCassette(cassette);
                }
            }else if (!gamepad2.y){
                VARS.t0 = 0;
                VARS.CST_UP = false;
                armMotor.setPower(0);
                powerCassette(cassette);
            }


            if (VARS.CST_UP){
                // switch to flat position

                if (!Double.isNaN(cassette.getPosition())){
                    if (cassette.getPosition() + 0.05 < VARS.CST_LOWER_BOUND){
                        cassette.setPosition(cassette.getPosition() + 0.05);
                        sleep(100);
                    }else{
                        cassette.setPosition(VARS.CST_LOWER_BOUND);
                        sleep(100);
                    }

                }else{
                    cassette.setPosition(0);
                    sleep(100);
                }


            }
            if (gamepad2.dpad_left){
                door.setPosition(0);
            }
            else if (gamepad2.dpad_right){
                door.setPosition(0);
            }else{
                door.setPosition(0.6);
            }
            if (imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) > -80 && imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) < -100){
                if (gamepad2.x){
                    // switch to parallel to backdrop position
                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() - 0.04 > VARS.CST_UPPER_BOUND){
                            if (Math.abs(armMotor.getCurrentPosition() - VARS.ARM_START_POS) < 300){
                                if (cassette.getPosition() > 0.35){
                                    cassette.setPosition(cassette.getPosition() - 0.04);
                                }else{
                                    cassette.setPosition(0.3);
                                }

                            }else{
                                cassette.setPosition(cassette.getPosition() - 0.04);
                            }

                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_UPPER_BOUND);
                            sleep(30);
                        }
                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }

                }

                if (gamepad2.b){
                    // switch to flat position

                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() + 0.04 < VARS.CST_LOWER_BOUND){
                            cassette.setPosition(cassette.getPosition() + 0.04);
                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_LOWER_BOUND);
                            sleep(30);
                        }

                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }


                }


            }else{
                if (gamepad2.b){
                    // switch to parallel to backdrop position
                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() - 0.04 > VARS.CST_UPPER_BOUND){
                            if (Math.abs(armMotor.getCurrentPosition() - VARS.ARM_START_POS) < 300){
                                if (cassette.getPosition() > 0.35){
                                    cassette.setPosition(cassette.getPosition() - 0.04);
                                }else{
                                    cassette.setPosition(0.3);
                                }

                            }else{
                                cassette.setPosition(cassette.getPosition() - 0.04);
                            }

                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_UPPER_BOUND);
                            sleep(30);
                        }
                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }

                }

                if (gamepad2.x){
                    // switch to flat position

                    if (!Double.isNaN(cassette.getPosition())){
                        if (cassette.getPosition() + 0.04 < VARS.CST_LOWER_BOUND){
                            cassette.setPosition(cassette.getPosition() + 0.04);
                            sleep(30);
                        }else{
                            cassette.setPosition(VARS.CST_LOWER_BOUND);
                            sleep(30);
                        }

                    }else{
                        cassette.setPosition(0);
                        sleep(30);
                    }


                }
            }


            if (VARS.CST_DOWN){
                // switch to parallel to backdrop position
                if (!Double.isNaN(cassette.getPosition())){
                    if (cassette.getPosition() - 0.05 > VARS.CST_UPPER_BOUND){
                        if (Math.abs(armMotor.getCurrentPosition() - VARS.ARM_START_POS) < 300){
                            if (cassette.getPosition() > 0.35){
                                cassette.setPosition(cassette.getPosition() - 0.05);
                            }else{
                                cassette.setPosition(0.3);
                            }

                        }else{
                            cassette.setPosition(cassette.getPosition() - 0.05);
                        }

                        sleep(100);
                    }else{
                        cassette.setPosition(VARS.CST_UPPER_BOUND);
                        sleep(100);
                    }
                }else{
                    cassette.setPosition(0);
                    sleep(100);
                }

            }
            powerCassette(cassette);

            if (!Double.isNaN(door.getPosition())){
                door.setPosition(door.getPosition());
            }else{
                door.setPosition(1);
            }

        }
            }

}