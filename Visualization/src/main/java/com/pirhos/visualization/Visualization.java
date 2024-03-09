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
        TrajectoryActionBuilder trajectoryActionBuilder = myBot.getDrive().actionBuilder(new Pose2d(-40, 60, Math.toRadians(-90)));
        myBot.runAction(trajectoryActionBuilder
                .splineToConstantHeading(new Vector2d(-40.00, 53.40), Math.toRadians(270.00))
                .splineToSplineHeading(new Pose2d(-32.00, 31.95, Math.toRadians(0.00)), Math.toRadians(0.00))
                .setTangent(Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-47, 11.5, Math.toRadians(0)), Math.toRadians(0), new TranslationalVelConstraint(60), new ProfileAccelConstraint(-60, 50))
                .splineToConstantHeading(new Vector2d(28, 20), Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(40, 45), Math.toRadians(0))
                .strafeTo(new Vector2d(40, 17))
                        .strafeToLinearHeading(new Vector2d(60, 17), Math.toRadians(-90))









                        .build());

        pirhos.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
        pirhos.getWindowFrame().setName("Visualization");
        pirhos.start();
    }
}