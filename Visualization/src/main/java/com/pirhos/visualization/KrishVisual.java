package com.pirhos.visualization;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.TurnConstraints;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class KrishVisual {
    public static void main(String[] args) {
        MeepMeep pirhos = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(pirhos)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 17)
                .build();

        TrajectoryActionBuilder trajectoryActionBuilder = myBot.getDrive().actionBuilder(new Pose2d(0, 0, Math.toRadians(0)));
        myBot.runAction(trajectoryActionBuilder
                .strafeToSplineHeading(new Vector2d(60,60), Math.toRadians(270),  new TranslationalVelConstraint(100))               .build()      );

        pirhos.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot);
        pirhos.getWindowFrame().setName("Visualization");
        pirhos.start();
    }
}