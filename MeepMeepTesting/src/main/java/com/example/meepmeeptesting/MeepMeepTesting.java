package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.File;
import java.io.IOException;

import javax.imageio.ImageIO;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16, 18)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-71.90, -36.75, 0))
                                .splineTo(new Vector2d(-34.79, -36.95), Math.toRadians(1.18))
                                .splineTo(new Vector2d(-31.28, -35.61), Math.toRadians(270.00))
                                .splineTo(new Vector2d(-49.63, -36.23), Math.toRadians(180.00))
                                .splineTo(new Vector2d(-45.41, -57.98), Math.toRadians(-42.66))
                                .splineTo(new Vector2d(-33.86, -47.46), Math.toRadians(270.00))
                                .splineTo(new Vector2d(-36.64, 48.50), Math.toRadians(90.00))
                                .build());

        Image img = null;
        try { img = ImageIO.read(new File("C:\\Users\\AadiJ\\Projects\\FTC\\roadrunner-updated-blank\\MeepMeepTesting\\src\\main\\java\\com\\example\\meepmeeptesting\\centerstage_bg.png")); }
        catch (IOException ignored) {}
        assert img != null;
        meepMeep.setBackground(img)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}