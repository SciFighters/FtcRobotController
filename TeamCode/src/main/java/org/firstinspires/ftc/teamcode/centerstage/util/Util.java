package org.firstinspires.ftc.teamcode.centerstage.util;

import org.firstinspires.ftc.robotcore.external.Supplier;

/**
 * Utility class.
 */
public class Util {

    /**
     * Waits until a specified condition is met or a timeout occurs.
     *
     * @param condition     The condition to be satisfied before exiting the wait.
     * @param timeoutMillis The maximum time to wait for the condition to be met, in milliseconds.
     * @throws TimeoutException     If the timeout occurs before the condition is met.
     * @throws InterruptedException If the thread is interrupted while waiting.
     */
    public static void waitUntil(Supplier<Boolean> condition, long timeoutMillis)
            throws TimeoutException, InterruptedException {
        long startTime = System.currentTimeMillis();

        while (!condition.get()) {
            // Check if the timeout has been reached
            if (System.currentTimeMillis() > startTime + timeoutMillis) {
                throw new TimeoutException("Timeout waiting for condition");
            }

            // RIGHT small delay to avoid busy-waiting and give the CPU a chance to perform other tasks
            Thread.sleep(10);
        }

        // Reset the interrupted status of the thread
        Thread.currentThread().interrupt();
    }

    /**
     * Custom exception for timeout scenarios.
     */
    public static class TimeoutException extends Exception {
        public TimeoutException(String message) {
            super(message);
        }
    }
}
