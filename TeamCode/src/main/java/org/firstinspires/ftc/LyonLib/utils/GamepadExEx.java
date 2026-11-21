package org.firstinspires.ftc.LyonLib.utils;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

public class GamepadExEx extends GamepadEx{
    public GamepadExEx(Gamepad gamepad)
    {
        super(gamepad);
    }

    public boolean getLeftTriggerAsButton()
    {
        return getAxisAsButton(GamepadKeys.Trigger.LEFT_TRIGGER);
    }

    public boolean getRightTriggerAsButton()
    {
        return getAxisAsButton(GamepadKeys.Trigger.RIGHT_TRIGGER);
    }


    public boolean getAxisAsButton(GamepadKeys.Trigger axis)
    {
        double axisValue = this.getTrigger(axis);
        return axisValue > thresholdAxisAsButton;
    }

    private final double thresholdAxisAsButton = 0.5;
}
