package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface Action {
    void init();
    boolean loop(TelemetryPacket p);

    default void draw(Canvas c) {

    }

    default void runBlocking() {
        init();

        boolean b = true;
        while (!Thread.currentThread().isInterrupted() && b) {
            TelemetryPacket p = new TelemetryPacket();

            draw(p.fieldOverlay());
            b = loop(p);

            FtcDashboard.getInstance().sendTelemetryPacket(p);
        }
    }

    default double clock() {
        return 1e-9 * System.nanoTime();
    }

    // not persistent!
    interface Builder<T extends Builder<T>> {
        T add(Action a);
    }

    class BaseBuilder implements Builder<BaseBuilder> {
        private Action a;

        @Override
        public BaseBuilder add(Action a) {
            this.a = a;
            return this;
        }

        public SeqBuilder<BaseBuilder> seq() {
            return new SeqBuilder<>(this);
        }

        public ParBuilder<BaseBuilder> par() {
            return new ParBuilder<>(this);
        }

        public Action build() {
            return a;
        }
    }

    class SeqBuilder<T extends Builder<T>> implements Builder<SeqBuilder<T>> {
        private final T parent;
        private final List<Action> as = new ArrayList<>();

        private SeqBuilder(T parent) {
            this.parent = parent;
        }

        @Override
        public SeqBuilder<T> add(Action a) {
            as.add(a);
            return this;
        }

        public T end() {
            parent.add(new SequentialAction(as));
            return parent;
        }

        public SeqBuilder<SeqBuilder<T>> seq() {
            return new SeqBuilder<>(this);
        }

        public ParBuilder<SeqBuilder<T>> par() {
            return new ParBuilder<>(this);
        }

        public SeqBuilder<T> sleep(double duration) {
            return add(new SleepAction(duration));
        }

        public SeqBuilder<T> instant(InstantAction.Fun f) {
            return add(new InstantAction(f));
        }
    }

    class ParBuilder<T extends Builder<T>> implements Builder<ParBuilder<T>> {
        private final T parent;
        private final List<Action> as = new ArrayList<>();

        private ParBuilder(T parent) {
            this.parent = parent;
        }

        @Override
        public ParBuilder<T> add(Action a) {
            as.add(a);
            return this;
        }

        public T end() {
            parent.add(new ParallelAction(as));
            return parent;
        }

        public SeqBuilder<ParBuilder<T>> seq() {
            return new SeqBuilder<>(this);
        }

        public ParBuilder<ParBuilder<T>> par() {
            return new ParBuilder<>(this);
        }

        public ParBuilder<T> sleep(double duration) {
            return add(new SleepAction(duration));
        }

        public ParBuilder<T> instant(InstantAction.Fun f) {
            return add(new InstantAction(f));
        }
    }
}

final class SleepAction implements Action {
    public final double duration;
    private double endTs;

    public SleepAction(double duration) {
        this.duration = duration;
    }

    @Override
    public void init() {
        endTs = clock() + duration;
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        return clock() <= endTs;
    }
}

final class InstantAction implements Action {
    public interface Fun {
        void run();
    }

    public final Fun f;

    public InstantAction(Fun f) {
        this.f = f;
    }

    @Override
    public void init() {
        f.run();
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        return false;
    }
}

final class ParallelAction implements Action {
    public final List<Action> actions;
    private List<Action> remaining;

    public ParallelAction(List<Action> actions) {
        this.actions = Collections.unmodifiableList(actions);
    }

    @Override
    public void init() {
        remaining = new ArrayList<>(actions);
        for (Action a : remaining) {
            a.init();
        }
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        List<Action> newRem = new ArrayList<>(remaining);
        for (Action a : remaining) {
            if (a.loop(p)) {
                newRem.add(a);
            }
        }

        remaining = newRem;

        return remaining.size() > 0;
    }

    @Override
    public void draw(Canvas c) {
        for (Action a : actions) {
            draw(c);
        }
    }
}

final class SequentialAction implements Action {
    public final List<Action> actions;
    private int index;
    private boolean needsInit = true;

    public SequentialAction(List<Action> actions) {
        this.actions = Collections.unmodifiableList(actions);
    }

    @Override
    public void init() {
    }

    @Override
    public boolean loop(TelemetryPacket p) {
        if (index >= actions.size()) {
            return false;
        }

        Action a = actions.get(index);

        if (needsInit) {
            a.init();
            needsInit = false;
        }

        if (!a.loop(p)) {
            index++;
            needsInit = true;
        }

        return true;
    }

    @Override
    public void draw(Canvas c) {
        for (Action a : actions) {
            draw(c);
        }
    }
}
