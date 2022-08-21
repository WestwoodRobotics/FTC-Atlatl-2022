package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dIncrementDual;

public interface Localizer {
    Twist2dIncrementDual<Time> updateAndGetIncr();
}
