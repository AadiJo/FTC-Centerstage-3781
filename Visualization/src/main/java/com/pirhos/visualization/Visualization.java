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
        TrajectoryActionBuilder trajectoryActionBuilder = myBot.getDrive().actionBuilder(new Pose2d(16, -60, Math.toRadians(90)));
        myBot.runAction(trajectoryActionBuilder
                .splineToConstantHeading(new Vector2d(23.00, -37.35), Math.toRadians(90.00))
                        .waitSeconds(0.2)
                        .setTangent(Math.toRadians(-90))
//                .splineToConstantHeading(new Vector2d(31.38, -57.80), Math.toRadians(90))
                .splineToConstantHeading(new Vector2d(39.00, -42), Math.toRadians(90))
//                .splineToSplineHeading(new Pose2d(35.08, -53.82, Math.toRadians(31.22)), Math.toRadians(31.22))
//                .splineToConstantHeading(new Vector2d(38.91, -37.35), Math.toRadians(90.00))
//                        .splineToConstantHeading(new Vector2d(36.07, -48.00), Math.toRadians(90.00))
                        .splineToSplineHeading(new Pose2d(42.00, -36.78, Math.toRadians(0.00)), Math.toRadians(180.00))
                        .waitSeconds(1)
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(48.85, -56.50), Math.toRadians(0.00))
                .splineToSplineHeading(new Pose2d(60.00, -57.94, Math.toRadians(90.00)), Math.toRadians(90.00))

//                // park close
//                        .setTangent(Math.toRadians(180))
//                .splineToConstantHeading(new Vector2d(48.85, -56.00), Math.toRadians(0.00))
//                .splineToSplineHeading(new Pose2d(60.00, -57.94, Math.toRadians(90.00)), Math.toRadians(90.00))
                .build()                );

        pirhos.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
        pirhos.getWindowFrame().setName("Visualization");
        pirhos.start();
    }
}