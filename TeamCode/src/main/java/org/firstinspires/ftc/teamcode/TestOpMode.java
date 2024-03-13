package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.ArmSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DroneSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.HeadingPIDController;

@TeleOp(name = "Command Op Mode")
public class TestOpMode extends CommandOpMode {

    ClawSubsystem claw = new ClawSubsystem();
    ArmSubsystem arm = new ArmSubsystem();
    DroneSubsystem drone = new DroneSubsystem();
    static HeadingPID headingPID = new HeadingPID();
    static Drivetrain drivetrain = new Drivetrain();
    HeadingPIDController headingPIDController = new HeadingPIDController();

    GamepadEx gamepad_1;
    GamepadEx gamepad_2;

    static class Drivetrain {
        Motor leftFront;
        Motor leftBack;
        Motor rightBack;
        Motor rightFront;
        MecanumDrive drive;
        IMU imu;
        IMU.Parameters parameters;
    }

    static class HeadingPID {
        double targetHeading = Math.toRadians(0);
    }
    @Override
    public void initialize() {
        gamepad_1 = new GamepadEx(gamepad1);
        gamepad_2 = new GamepadEx(gamepad2);

        claw.initialize(hardwareMap);
        arm.initialize(hardwareMap);

        drivetrain.leftFront = new Motor(hardwareMap, "frntLF");
        drivetrain.leftBack = new Motor(hardwareMap, "bckLF");
        drivetrain.rightBack = new Motor(hardwareMap, "bckRT");
        drivetrain.rightFront = new Motor(hardwareMap, "frntRT");
        drivetrain.leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drivetrain.leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drivetrain.rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drivetrain.rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        drivetrain.leftBack.setInverted(true);
        drivetrain.rightBack.setInverted(true);
        drivetrain. rightFront.setInverted(true);

        drivetrain.parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT));
        drivetrain.imu = (IMU) hardwareMap.get("imu");
        drivetrain.imu.initialize(drivetrain.parameters);

        drivetrain.drive = new MecanumDrive(
                drivetrain.leftFront, drivetrain.rightFront, drivetrain.leftBack, drivetrain.rightBack
        );

        CommandScheduler.getInstance().reset();

        gamepad_2.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> claw.open())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> claw.close())));
        gamepad_1.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(() -> CommandScheduler.getInstance().schedule(
                new InstantCommand(() -> drivetrain.imu.initialize(drivetrain.parameters)),
                new InstantCommand(() -> drivetrain.imu.resetYaw()),
                new InstantCommand(() -> headingPID.targetHeading = Math.toRadians(0)),
                new InstantCommand(() -> gamepad1.rumble(10, 10, 200))));
        gamepad_2.getGamepadButton(GamepadKeys.Button.BACK).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.enableOverride())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.BACK).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.disableOverride())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.openDoor())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.closeDoor())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.openDoor())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_RIGHT).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.closeDoor())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.Y).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.moveArmUp())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.Y).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.stopArm())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.A).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.moveArmDown())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.A).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.stopArm())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.X).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.moveCassetteDown())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.X).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.stopCassette())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.B).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.moveCassetteUp())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.B).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.stopCassette())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.hang())));
        gamepad_2.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenReleased(() -> CommandScheduler.getInstance().schedule(new InstantCommand(() -> arm.suspend())));
        if (gamepad_2.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5 && gamepad_2.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5){
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> drone.launch()));
        }else{
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> drone.stop()));
        }
        TriggerReader leftTrigger = new TriggerReader(gamepad_2, GamepadKeys.Trigger.LEFT_TRIGGER);
        TriggerReader rightTrigger = new TriggerReader(gamepad_2, GamepadKeys.Trigger.RIGHT_TRIGGER);

        if (leftTrigger.wasJustPressed() && rightTrigger.wasJustPressed()){
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> drone.launch()));
        }else{
            CommandScheduler.getInstance().schedule(new InstantCommand(() -> drone.stop()));
        }


    }
    @Override
    public void run() {
        drivetrain.imu.resetYaw();
        double loopTime = 0.0;
        while (opModeIsActive()) {
            // Run the command scheduler
            CommandScheduler.getInstance().run();
            double loop = System.nanoTime();
            telemetry.addData("hz", 1000000000  / (loop - loopTime));
            loopTime = loop;
            telemetry.update();

            if (gamepad_1.getRightX() != 0){
                headingPIDController.PIDControl(headingPID.targetHeading, drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
                drivetrain.drive.driveFieldCentric(
                        gamepad_1.getLeftX(),
                        gamepad_1.getLeftY(),
                        gamepad_1.getRightX() * 0.55,
                        drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                        false
                );
                // movement of inertia = 0.0951078743
                if (drivetrain.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate > 0){
                    headingPID.targetHeading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) +
                            (0.0981078743 * (drivetrain.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate * drivetrain.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));

                }else{
                    headingPID.targetHeading = drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS) -
                            (0.0981078743 * (drivetrain.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate * drivetrain.imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate));
                }
            }else{ // if no input from right joystick
                drivetrain.drive.driveFieldCentric(
                        gamepad_1.getLeftX(),
                        gamepad_1.getLeftY(),
                        -headingPIDController.PIDControl(headingPID.targetHeading, drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS)), // negative because FTCLib input is inverted
                        drivetrain.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES),   // gyro value passed in here must be in degrees
                        false
                );
            }
            arm.update();
            claw.update();


        }
    }
}
