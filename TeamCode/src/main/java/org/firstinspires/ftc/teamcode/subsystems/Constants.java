package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

public class Constants {
    //--- Buttons ---
    public static final GamepadKeys.Button PI2_CCW_BUTTON = GamepadKeys.Button.SQUARE;
    public static final GamepadKeys.Button PI2_CW_BUTTON = GamepadKeys.Button.CIRCLE;
    public static final GamepadKeys.Button PI_BUTTON = GamepadKeys.Button.SQUARE;
    public static final GamepadKeys.Button ZERO_HEADING_BUTTON = GamepadKeys.Button.CROSS;
    public static final GamepadKeys.Button RIGHT_DASH_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;
    public static final GamepadKeys.Button LEFT_DASH_BUTTON = GamepadKeys.Button.LEFT_BUMPER;

    //--- Poses ---
    public static final Pose2d BLUE_STARTING_POS = new Pose2d(0.0,0.0,0.0);
    public static final Pose2d RED_STARTING_POS = new Pose2d(0.0,0.0,0.0);
}
