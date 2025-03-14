package com.team254.lib.util;

/**
 * An iterative boolean latch that delays the transition from false to true.
 */
public class DelayedBoolean {
    private boolean mLastValue;
    private double mTransitionTimestamp;
    private double mDelay;

    public DelayedBoolean(double timestamp, double delay) {
        this(timestamp, delay, false);
    }

    public DelayedBoolean(double timestamp, double delay, boolean initialValue) {
        mDelay = delay;
        mLastValue = initialValue;

        if (initialValue) {
            mTransitionTimestamp = timestamp - delay;
        } else {
            mTransitionTimestamp = timestamp;
        }
    }

    public boolean update(double timestamp, boolean value) {
        boolean result = false;

        if (value && !mLastValue) {
            mTransitionTimestamp = timestamp;
        }

        // If we are still true and we have transitioned.
        if (value && (timestamp - mTransitionTimestamp > mDelay)) {
            result = true;
        }

        mLastValue = value;
        return result;
    }
}
