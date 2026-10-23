package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.CommandScheduler;

import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretSubsystem;

@Config
@TeleOp
public class TuretTest extends OpMode {

    miniRobot robot;
    ElapsedTime timer;
    TurretIO turret;


    @Override
    public void init(){

        robot = new miniRobot(hardwareMap, telemetry, miniRobot.alliance.BLUE, gamepad1);
        robot.getTurret().getTurretIO().resetMotor();
        timer = new ElapsedTime();

        //robot.getTurret().init();
    }

    @Override
    public void start()
    {
        robot.start();
    }

    @Override
    public void loop(){

        robot.update();
        CommandScheduler.getInstance().run();

       double error = robot.getTurret().getPidTurret().getSetpoint() - robot.getTurret().getTurretIO().getHeadingTicks();
       double errorDegrees = error / TurretConstants.TICKS_PER_DEGREES;
       TelemetryPacket packet  = new TelemetryPacket();
       packet.put("target", robot.getTurret().getPidTurret().getSetpoint());
       packet.put("current ticks", robot.getTurret().getTurretIO().getHeadingTicks());
       packet.put("error", error);
       packet.put("error in degrees", errorDegrees);
       FtcDashboard.getInstance().sendTelemetryPacket(packet);


       telemetry.update();


        //if (gamepad1.aWasPressed()) {
        //    robot.getTurret().getPidTurret().setSetpoint( (Math.PI/2) * TurretConstants.TICKS_PER_RADIANT);
        //}
        //robot.getTurret().getTurretIO().applyPower(robot.getTurret().getPidTurret().calculateWithRealTime(robot.getTurret().getTurretIO().getHeadingTicks(), timer.milliseconds()));

        }
        @Override
        public void stop(){
            CommandScheduler.getInstance().reset();
        }





}
