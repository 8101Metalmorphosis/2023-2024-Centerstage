package org.firstinspires.ftc.teamcode.FTCutil;

public class ButtonToggle {

    boolean toggleState = false;
    boolean lastPressed = false;

    public ButtonToggle() {}

    public void update(boolean pressed) {
        if(pressed && !lastPressed) {
            toggleState = !toggleState;
        }

        lastPressed = pressed;
    }

    public void setState(boolean state) {
        this.toggleState = state;
    }

    public boolean getState() {
        return toggleState;
    }
}
