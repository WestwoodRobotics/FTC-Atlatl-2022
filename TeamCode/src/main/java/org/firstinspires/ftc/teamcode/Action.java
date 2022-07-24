package org.firstinspires.ftc.teamcode;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public interface Action {
    Action step();

    default void runBlocking() {
        Action a = this;
        while (a != null) {
            a = a.step();
        }
    }

    default double clock() {
        return 1e-9 * System.nanoTime();
    }
}

class DeadlineAction implements Action {
    public final double timestamp;

    public DeadlineAction(double timestamp) {
        this.timestamp = timestamp;
    }

    @Override
    public Action step() {
        if (clock() >= timestamp) {
            return null;
        } else {
            return this;
        }
    }
}

class SleepAction implements Action {
    public final double duration;

    public SleepAction(double duration) {
        this.duration = duration;
    }

    @Override
    public Action step() {
        return new DeadlineAction(clock() + duration).step();
    }
}

class ParallelAction implements Action {
    public final List<Action> actions;

    public ParallelAction(List<Action> actions) {
        if (actions.isEmpty()) {
            throw new IllegalArgumentException();
        }

        this.actions = actions;
    }

    public ParallelAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public Action step() {
        List<Action> stillPending = new ArrayList<>();
        for (Action a : actions) {
            a = a.step();
            if (a != null) {
                stillPending.add(a);
            }
        }

        if (stillPending.isEmpty()) {
            return null;
        }

        return new ParallelAction(stillPending);
    }
}

class SequentialAction implements Action {
    public final List<Action> actions;

    public SequentialAction(List<Action> actions) {
        if (actions.isEmpty()) {
            throw new IllegalArgumentException();
        }

        this.actions = actions;
    }

    public SequentialAction(Action... actions) {
        this(Arrays.asList(actions));
    }

    @Override
    public Action step() {
        List<Action> newActions = new ArrayList<>(actions);
        Action a = actions.get(0).step();
        if (a == null) {
            newActions.remove(0);
            if (newActions.isEmpty()) {
                return null;
            }
        } else {
            newActions.set(0, a);
        }
        return new SequentialAction(newActions);
    }
}
