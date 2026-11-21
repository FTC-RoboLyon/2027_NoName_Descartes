package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.control.FeedForwardModel;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.teamcode.subsystems.shooterParametersCalculator.ShooterParametersCalculator;
import org.firstinspires.ftc.teamcode.subsystems.shooter.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Flywheel.FlywheelIO;
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood.HoodIO;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.utils.VoltageCompensator;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@TeleOp
@Config
public class ShooterCompressionTest extends OpMode {

    private ShooterSubsystem shooterSubsystem;
    ShooterParametersCalculator parametersCalculator;
    MultipleTelemetry Telemetry;

    VoltageCompensator voltageCompensator; 

    @Override
    public void init()
    {
        Telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        parametersCalculator = new ShooterParametersCalculator(()-> new Pose2d(0.0,0.0,0.0), Telemetry);
        shooterSubsystem = new ShooterSubsystem(new HoodIO(hardwareMap), new FlywheelIO(hardwareMap, Telemetry), parametersCalculator, Telemetry);

        shooterSubsystem.setSuppliersManualSpeedSuppliers(()-> gamepad1.a, 
                                                            ()-> gamepad1.b, 
                                                            ()-> gamepad1.x,
                                                            ()-> gamepad1.y);

        shooterSubsystem.setFlywheelManualVoltageSupplier(()-> gamepad2.left_trigger);
        shooterSubsystem.setFlywheelControlMode(ControlMode.MANUAL_VELOCITY);

        voltageCompensator = new VoltageCompensator(hardwareMap);
    }

    @Override
    public void loop()
    {
        voltageCompensator.update();

        shooterSubsystem.periodic();

        if (gamepad1.leftBumperWasPressed()){
            shooterSubsystem.setFlywheelControlMode(ControlMode.VELOCITY_DUTYCYCLE_PID_FF);
            shooterSubsystem.setHoodControlMode(ControlMode.POSITION_VOLTAGE_PID);
        }
        if (gamepad2.leftBumperWasPressed())
            shooterSubsystem.setFlywheelControlMode(ControlMode.MANUAL_DUTY_CYCLE);

        if (gamepad1.rightBumperWasPressed()){
            shooterSubsystem.setFlywheelControlMode(ControlMode.MANUAL_VELOCITY);
            shooterSubsystem.setHoodControlMode(ControlMode.MANUAL_POSITION);
        }

        if (gamepad1.rightTriggerWasPressed())
            {
                shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.SHOOT_PUBLIC_CELL);
            }
            

        if (gamepad1.leftTriggerWasPressed())
        {
            shooterSubsystem.setWantedState(ShooterSubsystem.WantedState.SHOOT_OPPOSITE_CELL); 
            
        }


        Telemetry.addData("flywheel control mode", shooterSubsystem.getFlywheelControlMode());
        Telemetry.addLine();
        Telemetry.addData("flywheel WANTED state", shooterSubsystem.getWantedState());
        Telemetry.addLine();
        Telemetry.addData("flywheel SYSTEM state", shooterSubsystem.getSystemState());
        Telemetry.update();
    }
}
