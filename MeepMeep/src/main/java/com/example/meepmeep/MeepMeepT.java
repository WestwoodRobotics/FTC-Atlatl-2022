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
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    liftMove(3100);
                                })
                                .back(11)
                                //high
                                .strafeLeft(12)
                                //preload drop
                                .forward(6.5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    liftMove(2000);

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    clawToggle();
                                })
                                .waitSeconds(0.5)
                                .back(7)


                                //stack

                                .strafeRight(30)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    liftMove(650);

                                })
                                .turn(Math.toRadians(-90))
                                .forward(10)
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    clawToggle();
                                })
                                .waitSeconds(0.5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    liftMove(2000);

                                })



                                .back(10)


                                //high
                                .turn(Math.toRadians(90))
                                .strafeLeft(30)

                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    liftMove(0);
                                })

                                .forward(6.5)
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    liftMove(3100);

                                })
                                .UNSTABLE_addTemporalMarkerOffset(0.25, () -> {
                                    clawToggle();
                                })
                                .waitSeconds(0.5)
                                .back(7)
//
                                .UNSTABLE_addTemporalMarkerOffset(0, () -> {
                                    liftMove(0);

                                })
                                .strafeRight(37)


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

    public static void clawToggle(){
        return;
    }

    public static void liftMove(int pos){
        return;
    }
}