package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepT {
    public int inchesPerMat = 24;

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(72 ,0, Math.toRadians(90)))
                                //setup
                                .waitSeconds(1)
                                .turn(Math.toRadians(0.01))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    public double xCord(double x){
        if (x > 0){
            x = (x * 24) - 12;
        }else if (x < 0){
            x = (x * 24) + 12;
        }else{
            x = 0;
        }

        return x;
    }

    public double yCord(double y){
        if (y > 0){
            y = (y * 24) - 12;
        }else if (y < 0){
            y = (y * 24) + 12;
        }else{
            y = 0;
        }

        return y;
    }
}