package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2IncrementDual;

public interface Localizer {
    Twist2IncrementDual<Time> updateAndGetIncr();
}
