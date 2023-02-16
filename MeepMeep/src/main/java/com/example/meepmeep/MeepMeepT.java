package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepT {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(57.01908330528, 52.48291908330528, Math.toRadians(224.92705213553745), Math.toRadians(224.92705213553745), 12.6)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(Cord(2) ,Cord(-3), Math.toRadians(90)))
                                .forward(60)
                                .back(12)

                                //first cone
                                .lineToLinearHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                                .forward(5)

                                .waitSeconds(0.5)

                                .back(5)


                                //1st stack
                                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                                .forward(5)

                                .waitSeconds(0.5)
                                .back(5)


                                //2nd stack
                                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                                .forward(5)
                                .back(5)

                                .waitSeconds(0.5)

                                //3rd stack
                                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(0)))

                                .waitSeconds(0.5)
                                .lineToSplineHeading(new Pose2d(Cord(1.5) ,Cord(-1), Math.toRadians(90)))
                                .forward(5)
                                .back(5)

                                .waitSeconds(0.5)


                                .lineToSplineHeading(new Pose2d(Cord(3.1) ,Cord(-1), Math.toRadians(90)))
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