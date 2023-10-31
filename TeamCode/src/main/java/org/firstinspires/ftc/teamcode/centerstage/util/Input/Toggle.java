package org.firstinspires.ftc.teamcode.centerstage.util.Input;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.teamcode.centerstage.util.Input.Input;

/**
 * Toggles of the KeyCode type are updated automatically if the UpdateAutomatically annotation is present and there's no need to call the update function on them
 */
public class Toggle {

    private boolean lastInput = false, pressed = false, changed = false, state = false;
    private Input.KeyCode mapping = null;
    private Func<Boolean> returnVal;

    public Toggle() {
    }

    public Toggle(Func<Boolean> returnVal) {
        this.returnVal = returnVal;
    }

    public Toggle(boolean initialState) {
        this.state = initialState;
    }

    public Toggle(Input.KeyCode mapping) {
        this.mapping = mapping;
    }

    public void update(boolean input) {
        if (lastInput != input) { // if the input has changed,
            changed = true; // (it has changed)
            pressed = input;
            if (pressed)
                state = !state; // The toggle changes only when it's pressed and not when it's released
            lastInput = input;
        } else {
            changed = false; // otherwise it didn't
        }
    }

    public void update() {
        boolean input;
        if (mapping != null) {
            input = Input.GetKeyPressed(mapping);
        } else {
            input = returnVal.value();
        }
        if (lastInput != input) { // if the input has changed,
            changed = true; // (it has changed)
            pressed = input;
            if (pressed)
                state = !state; // The toggle changes only when it's pressed and not when it's released
            lastInput = input;
        } else {
            changed = false; // otherwise it didn't
        }
    }

    public void set(boolean input) {
        changed = lastInput != input; // simplified
        state = input;
        lastInput = input;
    }

    public boolean toggle() {
        state = !state;
        changed = true;
        return state;
    }

    public boolean isPressed() {
        return pressed;
    }

    public boolean isChanged() {
        return changed;
    }

    public boolean getState() {
        return state;
    }

    public boolean isClicked() {
        return changed && pressed;
    }

    public boolean isReleased() {
        return changed && !pressed;
    }

    public Input.KeyCode getMapping() {
        return mapping;
    }

    public void remap(Input.KeyCode newMapping) {
        this.mapping = newMapping;
    }
    //THIS CODE IS NICE DON'T LAUGH :(
}
