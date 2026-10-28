package org.firstinspires.ftc.teamcode.subsystems;

import com.seattlesolvers.solverslib.gamepad.GamepadKeys;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;

public class Constants {
    // --- Robot ---
    public enum RobotModel
    {
        TRAINING,
        COMPETITION,
        DEMO
    }
    public static final RobotModel ROBOT_MODEL = RobotModel.TRAINING;
    // --- Gamepad 1 ---
    public static final GamepadKeys.Button PI2_CCW_BUTTON = GamepadKeys.Button.SQUARE;
    public static final GamepadKeys.Button PI2_CW_BUTTON = GamepadKeys.Button.CIRCLE;
    public static final GamepadKeys.Button PI_BUTTON = GamepadKeys.Button.SQUARE;
    public static final GamepadKeys.Button ZERO_HEADING_BUTTON = GamepadKeys.Button.CROSS;
    public static final GamepadKeys.Button RIGHT_DASH_BUTTON = GamepadKeys.Button.RIGHT_BUMPER;
    public static final GamepadKeys.Button LEFT_DASH_BUTTON = GamepadKeys.Button.LEFT_BUMPER;

    // --- Gamepad 2 ---
    public static final GamepadKeys.Button TRACK_RED_PUBLIC_CELL = GamepadKeys.Button.LEFT_BUMPER;
    public static final GamepadKeys.Button TRACK_BLUE_OPPOSITE_CELL = GamepadKeys.Button.RIGHT_BUMPER;
    public static final GamepadKeys.Button STOP_TURRET = GamepadKeys.Button.TRIANGLE;
    // --- Poses ---
    public static final Pose2d BLUE_STARTING_POS = new Pose2d(0.0,0.0,0.0);
    public static final Pose2d RED_STARTING_POS = new Pose2d(0.0,0.0,0.0);
}
