package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ArmSubsystem implements Subsystem{
    DcMotorEx arm;
    DcMotorEx pulley;
    Servo cassette;
    Servo door;
    int ARM_START_POS;
    enum ArmState {
        UP,
        DOWN,
        STOPPED
    }
    enum DoorState {
        OPEN,
        CLOSED
    }

    static class CST{
        double LOWER_BOUND = 1.0;
        double UPPER_BOUND = 0.0;
        double PICK_BOUND = 0.3;
    }

    ArmState armState = ArmState.STOPPED;
    ArmState cassetteState = ArmState.STOPPED;
    DoorState doorState = DoorState.CLOSED;
    boolean override = false;
    CST CST = new CST();
    @Override
    public void initialize(HardwareMap hardwareMap) {
        arm = hardwareMap.get(DcMotorEx.class, "armMotor");
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        cassette = hardwareMap.servo.get("cassette");
        door = hardwareMap.servo.get("door");
        pulley = hardwareMap.get(DcMotorEx.class, "frontEncoder");
        pulley.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pulley.setDirection(DcMotorSimple.Direction.REVERSE);
        ARM_START_POS = arm.getCurrentPosition();
    }

    @Override
    public void update() {
        switch (armState) {
            case UP:
                // Move arm up
                if (arm.getCurrentPosition() > (ARM_START_POS - 6166)){
                    cassetteState = ArmState.DOWN;
                    arm.setPower(-1);
                }
                break;
            case DOWN:
                // Move arm down
                if (override){
                    ARM_START_POS = arm.getCurrentPosition();
                }
                if (arm.getCurrentPosition() < ARM_START_POS){
                    // opposite to arm
                    cassetteState = ArmState.UP;
                    arm.setPower(1);
                }
                break;
            case STOPPED:
                // Stop arm
                arm.setPower(0);
                cassetteState = ArmState.STOPPED;
                break;
        }
        switch (cassetteState){
            case UP:
                // Move cassette up
                cassette.setPosition(Math.min(cassette.getPosition() + 0.05, CST.LOWER_BOUND));
                new WaitCommand(100);
                break;
            case DOWN:
                // Move cassette down
                if (cassette.getPosition() - 0.05 > CST.UPPER_BOUND){
                    if (Math.abs(arm.getCurrentPosition() - ARM_START_POS) < 300){
                        if (cassette.getPosition() > (CST.PICK_BOUND + 0.05)){
                            cassette.setPosition(cassette.getPosition() - 0.05);
                        }else{
                            cassette.setPosition(CST.PICK_BOUND);
                        }

                    }else{
                        cassette.setPosition(cassette.getPosition() - 0.05);
                    }

                }else{
                    cassette.setPosition(CST.UPPER_BOUND);
                }
                new WaitCommand(100);
                break;
            case STOPPED:
                // Stop cassette
                cassette.setPosition(cassette.getPosition());
                break;
        }

        switch (doorState){
            case OPEN:
                // Open door
                door.setPosition(0);
                break;
            case CLOSED:
                // Close door
                door.setPosition(0.6);
                break;
        }

        if (Double.isNaN(cassette.getPosition())){
            cassette.setPosition(0);
        }
    }

    @Override
    public void stop() {
        // Stop arm
        armState = ArmState.STOPPED;
        cassetteState = ArmState.STOPPED;
    }

    public void enableOverride() {override = true;}

    public void disableOverride() {override = false;}

    public void hang() {pulley.setPower(-0.75);} // TODO implement hanging

    public void suspend() {pulley.setPower(0);}

    public void openDoor() {doorState = DoorState.OPEN;}

    public void closeDoor() {doorState = DoorState.CLOSED;}

    public void moveArmUp() {armState = ArmState.UP;}

    public void moveArmDown() {armState = ArmState.DOWN;}

    public void stopArm() {armState = ArmState.STOPPED;}

    public void moveCassetteUp() {cassetteState = ArmState.UP;}

    public void moveCassetteDown() {cassetteState = ArmState.DOWN;}

    public void stopCassette() {cassetteState = ArmState.STOPPED;}
}
