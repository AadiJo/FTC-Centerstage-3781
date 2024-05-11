package com.pirhos.visualization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Graphics2D;

public class Visualization {
    public static void main(String[] args) {
        MeepMeep pirhos = new MeepMeep(800);

        RoadRunnerBotEntity BlueB = new DefaultBotBuilder(pirhos)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();
        RoadRunnerBotEntity BlueA = new DefaultBotBuilder(pirhos)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();
        TrajectoryActionBuilder trajectoryActionBuilder2 = BlueB.getDrive().actionBuilder(new Pose2d(45, 60, Math.toRadians(180)));

        BlueB.runAction(
                trajectoryActionBuilder2

                        .strafeTo(new Vector2d(45, 33))
                        .strafeTo(new Vector2d(25, 33))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(-38, 33))
                        .turn(Math.toRadians(90))
                        .strafeTo(new Vector2d(-38, -52))
                        .strafeTo(new Vector2d(-60, -52))
                        .build());
        BlueA.runAction(
                trajectoryActionBuilder2
                        .strafeTo(new Vector2d(45, 33))
                        .strafeTo(new Vector2d(25, 33))
                        .waitSeconds(1)
                        .strafeTo(new Vector2d(-38, 33))
                        .turn(Math.toRadians(-90))
                        .strafeTo(new Vector2d(-38, 53))
                        .strafeTo(new Vector2d(-60, 53))
                        .build());

        pirhos.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(BlueA)
                .addEntity(BlueB);
        pirhos.getWindowFrame().setName("Visualization");
        pirhos.start();
    }
}