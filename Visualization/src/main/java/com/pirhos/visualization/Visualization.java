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

public class Visualization {
    public static void main(String[] args) {
        MeepMeep pirhos = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(pirhos)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();
        RoadRunnerBotEntity myBot1 = new DefaultBotBuilder(pirhos)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();
        TrajectoryActionBuilder trajectoryActionBuilder = myBot.getDrive().actionBuilder(new Pose2d(-40, -60, Math.toRadians(90)));
        myBot.runAction(trajectoryActionBuilder
//                .splineToSplineHeading(new Pose2d(35, 35, Math.toRadians(180)), Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(12.5, 35), Math.toRadians(0))
//                        .waitSeconds(0.1)
//                .splineToConstantHeading(new Vector2d(20, 35), Math.toRadians(0))
//                        .strafeToLinearHeading(new Vector2d(42, 35), Math.toRadians(0))

                .strafeToLinearHeading(new Vector2d(-54, -32.5), Math.toRadians(0))
                        .waitSeconds(0.05)
                .strafeTo(new Vector2d(-34, -32.5))
                .strafeToLinearHeading(new Vector2d(-40, -32.5), Math.toRadians(0))
                .strafeToLinearHeading(new Vector2d(-52, -12.36), Math.toRadians(0))
                .strafeTo(new Vector2d(50.73, -12.36))
                .splineToConstantHeading(new Vector2d(39, -33.5), Math.toRadians(0))
                        .waitSeconds(0.5)
                .strafeTo(new Vector2d(40, -12))
                .strafeToLinearHeading(new Vector2d(60, -12), Math.toRadians(90))
                .strafeTo(new Vector2d(60, -12.3))


                .build());

        pirhos.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
        pirhos.getWindowFrame().setName("Visualization");
        pirhos.start();
    }
}