package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepT {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(224.92705213553745), Math.toRadians(224.92705213553745), 13.369)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(Cord(2) ,Cord(-3)-5, Math.toRadians(90)))
                                //setup
                                .addTemporalMarker(0, () -> {

                                })
                                .forward(64)
                                .addTemporalMarker(1, () -> {

                                })
                                .back(11)
                                //high
                                .strafeLeft(12)
                                //preload drop
                                .forward(5)
                                .addTemporalMarker(0, () -> {

                                })
                                .waitSeconds(0.5)
                                .back(5)
                                //go to stack
                                .turn(Math.toRadians(-90))

                                .addTemporalMarker(0, () -> {

                                })

                                .forward(40)
                                .addTemporalMarker(0, () -> {

                                })
                                .waitSeconds(0.5)
                                //high
                                .addTemporalMarker(0, () -> {

                                })
                                .back(40)
                                .turn(Math.toRadians(90))
                                //1st drop
                                .forward(5)
                                .addTemporalMarker(0, () -> {

                                })
                                .waitSeconds(0.5)
                                .back(5)
                                //go to stack
                                .turn(Math.toRadians(-90))
                                .addTemporalMarker(0, () -> {

                                })
                                .forward(40)
                                .addTemporalMarker(0, () -> {

                                })
                                .waitSeconds(0.5)
                                //high
                                .addTemporalMarker(0, () -> {

                                })
                                .back(40)
                                .turn(Math.toRadians(90))
                                //2nd drop
                                .forward(5)
                                .addTemporalMarker(0, () -> {

                                })
                                .waitSeconds(0.5)
                                .back(5)
                                //right park
                                .strafeRight(12)

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public static double Cord(double mat){
        if (mat > 0){
            mat = (mat * 24) - 12;
        }else if (mat < 0){
            mat = (mat * 24) + 12;
        }else{
            mat = 0;
        }

        return mat;
    }
}