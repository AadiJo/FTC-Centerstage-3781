package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * Created by Lance He 9/16/2017. Hello Guys
 */

//Working
public class Hardware2 {

    // Motor variable names
    public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;
    public Servo servoLeft = null;
    public DistanceSensor distSensor;
    public TouchSensor magnetSens;
    public ColorSensor colorSensor;
    public int hello;

    public boolean runThisWithEncoder = true;

    // Other variable names
    HardwareMap hwMap;
    public ElapsedTime period = new ElapsedTime();


    public Hardware2() {
        hwMap = null;
        this.runThisWithEncoder = true;
    }


    public Hardware2(boolean runThisWithEncoder) {
        hwMap = null;
        this.runThisWithEncoder = runThisWithEncoder;
    }


    public void initTeleOpIMU(HardwareMap hwMap) {

        // Save reference to Hardware map
        this.hwMap = hwMap;

        period.reset();

        // Define Motors I have changed these below for Mercury

        frontLeftMotor = hwMap.dcMotor.get("front_left");
        frontRightMotor = hwMap.dcMotor.get("front_right");
        backLeftMotor = hwMap.dcMotor.get("back_left");
        backRightMotor = hwMap.dcMotor.get("back_right");

        // Define Motors these are right
        // The reference variables will point to the correct motor
        /*
        frontLeftMotor = hwMap.dcMotor.get("back_left");   // swapped this
        frontRightMotor = hwMap.dcMotor.get("front_right");
        backLeftMotor = hwMap.dcMotor.get("front_left");   // swapped this
        backRightMotor = hwMap.dcMotor.get("back_right");
        */

        // Initialize Motors

        // ******MAY CHANGE *******  Fix Forward/Reverse under testing
        // these are the old ones - I have changed them for Mercury
        /*
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
        */

        // these are correct for Mercury

        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);


        // May use RUN_USING_ENCODERS if encoders are installed

        if (runThisWithEncoder)
        {
            // Do if encoders are installed
            frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            // get the encoders reset
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        }
        else
        {
            // get the encoders reset if it was in that mode previously
            frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


            frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }



        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The motor power has a range of -1.0 to 1.0
        // Negative is backwards, Positive is forwards, 0 is off
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);




    }


    public double getTime(){

        return period.time();

    }
    public void sleep(int time){
        period.reset();
        while(period.time() < time / 1000.0){

        }
    }
    public void Go(double speedl, double speedr, int time) {
        frontRightMotor.setPower(speedr);
        frontLeftMotor.setPower(speedl);
        backRightMotor.setPower(speedr);
        backLeftMotor.setPower(speedl);
        sleep(time);
    }
    public void setAllMotors(double powerl, double powerr) {
        frontLeftMotor.setPower(powerl);
        frontRightMotor.setPower(powerr);
        backLeftMotor.setPower(powerl);
        backRightMotor.setPower(powerr);

    }
    public void runUsingEncoders(double power, int newBlPos, int newBrPos, int newFlPos, int newFrPos) {
        //set the new target

        frontLeftMotor.setTargetPosition(frontLeftMotor.getCurrentPosition() + newFlPos);
        frontRightMotor.setTargetPosition(frontRightMotor.getCurrentPosition() + newFrPos);
        backLeftMotor.setTargetPosition(backLeftMotor.getCurrentPosition() + newBlPos);
        backRightMotor.setTargetPosition(backRightMotor.getCurrentPosition() + newBrPos);

        //set mode

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //set power

        setAllMotors(power, power);

        while (frontRightMotor.isBusy()) {

        }
    }

    public void backLeftMotor(int i) {
        backLeftMotor.setPower(i);

    }
    
    /*
    public int getBlue(){
        return colorSensor.blue();
    }
    public int getRed(){
        return colorSensor.red();
    }
    public int getGreen(){
        return colorSensor.green();
    }
    public boolean isRed(){
        return getRed() > getBlue() && getRed() > getGreen();
    }

    public boolean isBlue(){
        return getBlue() > getRed() && getBlue() > getGreen();
    }
    public boolean isGreen(){
        return getGreen() > getBlue() && getGreen() > getRed() && getGreen > 10000;
    }
    public boolean isBrown() {
        return getGreen() <  4000 && getRed() < 4000 && getBlue() < 4000;
    }
    public void whileColor() {
        while (isBrown() == true) {
            setAllMotors(0.2);
        }
        setAllMotors(0.0);

    }
    */
}

