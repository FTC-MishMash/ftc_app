package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.robotcore.internal.vuforia.VuforiaLocalizerImpl;

public class VuforiaLocalizerEx extends VuforiaLocalizerImpl {
    public VuforiaLocalizerEx(Parameters parameters) {
        super(parameters);
    }

    @Override
    protected void close() {
        super.close();
    }
}
