package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;
import com.qualcomm.robotcore.eventloop.opmode.OpModeRegistrar;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeMeta;

import java.util.Arrays;
import java.util.List;

public final class TuningOpModes {
    public static final String GROUP = "quickstart";
    public static final boolean DISABLED = false;

    private TuningOpModes() {

    }

    @OpModeRegistrar
    public void register(OpModeManager manager) {
        if (DISABLED) return;

        List<Class<?>> opModes = Arrays.asList(
                AccelLogger.class,
                AngularVelocity.class,
                ForwardPushTest.class,
                ForwardRampLogger.class,
                LateralPushTest.class
        );

        for (Class<?> o : opModes) {
            manager.register(new OpModeMeta.Builder()
                    .setName(o.getSimpleName())
                    .setGroup(GROUP)
                    .setFlavor(OpModeMeta.Flavor.TELEOP)
                    .build(), (Class<? extends OpMode>) o);
        }
    }
}
