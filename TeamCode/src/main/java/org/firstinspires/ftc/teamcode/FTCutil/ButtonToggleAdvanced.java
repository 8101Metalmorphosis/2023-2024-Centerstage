package org.firstinspires.ftc.teamcode.FTCutil;

public class ButtonToggleAdvanced {

    public static final int NEUTRAL_STATE = 0;
    public static final int ON_STATE = 1;
    public static final int OFF_STATE = -1;

    int toggleState = 0;
    int lastPressed = 0;

    public ButtonToggleAdvanced(int startState) {
        toggleState = startState;
    }

    public void update(boolean pressed) {
        if(toggleState == NEUTRAL_STATE) {
            toggleState = ON_STATE;
            return;
        }

        if(pressed && lastPressed == OFF_STATE) {
            toggleState = toggleState * -1;
        }

        if(pressed == true) {
            lastPressed = ON_STATE;
            return;
        }

        lastPressed = OFF_STATE;
    }

    public void setState(int state) {
        this.toggleState = state;
    }

    public int getState() {
        return toggleState;
    }
}
