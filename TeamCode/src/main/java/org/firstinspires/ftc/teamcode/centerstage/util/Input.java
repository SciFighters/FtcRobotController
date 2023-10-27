package org.firstinspires.ftc.teamcode.centerstage.util;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Func;

public class Input {
    private static Gamepad gamepad1, gamepad2;

    public static void updateControls(Gamepad gamepad1_, Gamepad gamepad2_) {
        gamepad1 = gamepad1_;
        gamepad2 = gamepad2_;
    }

    //region Key Getter Functions
    public static Func<Boolean> getAKey1 = () -> gamepad1.a;
    public static Func<Boolean> getXKey1 = () -> gamepad1.x;
    public static Func<Boolean> getBKey1 = () -> gamepad1.b;
    public static Func<Boolean> getYKey1 = () -> gamepad1.y;
    public static Func<Boolean> getDpadUpKey1 = () -> gamepad1.dpad_up;
    public static Func<Boolean> getDpadDownKey1 = () -> gamepad1.dpad_down;
    public static Func<Boolean> getDpadLeftKey1 = () -> gamepad1.dpad_left;
    public static Func<Boolean> getDpadRightKey1 = () -> gamepad1.dpad_right;
    public static Func<Boolean> getStartKey1 = () -> gamepad1.start;
    public static Func<Boolean> getSelectKey1 = () -> gamepad1.back;
    public static Func<Boolean> getPSKey1 = () -> gamepad1.ps;

    //endregion
    public static boolean GetKey(KeyCode key) {
        return key.get().value();
    }
}
