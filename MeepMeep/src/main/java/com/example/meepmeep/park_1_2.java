package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class park_1_2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57, 30, Math.toRadians(180), Math.toRadians(180), 13.369)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(35.5, -60, Math.toRadians(90)))
                                //setup
                                .waitSeconds(1)
                                .turn(Math.toRadians(45))
                                //first cone
                                .lineToLinearHeading(new Pose2d(35.5, -23.8, Math.toRadians(180)))
                                .forward(5)
                                .waitSeconds(1)
                                .back(5)
                                //head to stack
                                .lineToLinearHeading(new Pose2d(35.5, -12, Math.toRadians(100)))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))

                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(35.5, -12, Math.toRadians(100)))
                                .lineToLinearHeading(new Pose2d(35.5, -23.8, Math.toRadians(180)))
                                .forward(5)
                                .waitSeconds(1)
                                .back(5)

                                //head to stack
                                .lineToLinearHeading(new Pose2d(35.5, -12, Math.toRadians(100)))
                                .lineToLinearHeading(new Pose2d(60, -12, Math.toRadians(0)))

                                .waitSeconds(1)

                                .lineToLinearHeading(new Pose2d(35.5, -12, Math.toRadians(100)))
                                .lineToLinearHeading(new Pose2d(35.5, -23.8, Math.toRadians(180)))
                                .forward(5)
                                .waitSeconds(1)
                                .back(5)

                                .strafeLeft(11.5)

                                .forward(23)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

}