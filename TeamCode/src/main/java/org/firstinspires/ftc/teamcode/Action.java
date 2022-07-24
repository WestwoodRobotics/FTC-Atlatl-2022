package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public interface Action {
    void init();
    boolean loop();

    default void runBlocking() {
        init();
        while (loop());
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
    public boolean loop() {
        return clock() <= endTs;
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
    public boolean loop() {
        List<Action> newRem = new ArrayList<>(remaining);
        for (Action a : remaining) {
            if (a.loop()) {
                newRem.add(a);
            }
        }

        remaining = newRem;

        return remaining.size() > 0;
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
    public boolean loop() {
        if (index >= actions.size()) {
            return false;
        }

        Action a = actions.get(index);

        if (needsInit) {
            a.init();
            needsInit = false;
        }

        if (!a.loop()) {
            index++;
            needsInit = true;
        }

        return true;
    }
}
