package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.ControlMode;
import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.shooterParametersCalculator.ShooterParametersCalculator;

@Config
public class TurretSubsystem extends SubsystemBase {

    private Telemetry telemetry;

    private ElapsedTime timer;
    private Pose2d targetPos = new Pose2d();
    private double targetAngle = 0.0;
    private PidRBL pidTurret;
    private TurretIO turretIO;
    private ShooterParametersCalculator parametersCalculator;

    private double actualTime = 0.0;
    boolean isInit = true;



    public enum WantedState{
        STAND_BY,
        STOP,
        TRACK_PUBLIC_CELL,
        TRACK_OPPOSITE_CELL,
        TRACK_PUBLIC_SIDE_CENTER,
        TRACK_OPPOSITE_SIDE_CENTER
    }
    public enum SystemState{
        IDLE,

        SEARCHING_PUBLIC_CELL,
        SEARCHING_OPPOSITE_CELL,

        SEARCHING_PUBLIC_SIDE_CENTER,
        SEARCHING_OPPOSITE_SIDE_CENTER,

        ALIGNED_WITH_PUBLIC_CELL,
        ALIGNED_WITH_OPPOSITE_CELL,
        ALIGNED_WITH_PUBLIC_SIDE_CENTER,
        ALIGNED_WITH_OPPOSITE_SIDE_CENTER

    }
    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = SystemState.IDLE;
    private SystemState previousState = SystemState.IDLE;
    private ControlMode controlMode = ControlMode.DISABLED;

    public ControlMode getControlMode() {return controlMode;}
    public void setControlMode(ControlMode controlMode) {this.controlMode = controlMode;}

    public void setWantedState(WantedState state){wantedState = state;}
    public SystemState getSystemState(){return systemState;}

    public TurretIO getTurretIO() {
        return turretIO;
    }

    public void setTargetPos(Pose2d target){targetPos = target;}

    public void setTargetAngle(double angle){targetAngle = angle;}

    public PidRBL getPidTurret(){return pidTurret;}

    public TurretSubsystem(HardwareMap hmap, Telemetry telemetry,
                           ShooterParametersCalculator parametersCalculator){

        this.parametersCalculator = parametersCalculator;
        this.telemetry = telemetry;

        turretIO = new TurretIO(hmap);

        pidTurret = new PidRBL(TurretConstants.turretKP,TurretConstants.turretKI , TurretConstants.turretKD);
        pidTurret.reset(turretIO.getHeadingTicks());
        pidTurret.setOutputLimits(-1, 1);
        pidTurret.setInputLimits(-180.0, 180.0);
        pidTurret.setContinuous(true);

        systemState = SystemState.IDLE;
        wantedState = WantedState.STAND_BY;

        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public void periodic(){
        turretIO.update();
        pidTurret.setGains(TurretConstants.turretKP, TurretConstants.turretKI , TurretConstants.turretKD); //TODO : suppress after tuning coefs (or before adding a updatePIDcoef() fonction)

        //if (robot.getRobotPos().yMeters < publicLauchZone && robot.getAlliance() == miniRobot.alliance.RED)
        //    targetPos = redPublicCellPos;
        //
        //else if (robot.getRobotPos().yMeters < publicLauchZone && robot.getAlliance() == miniRobot.alliance.BLUE)
        //    targetPos = bluePublicCellePos;
        //
        //else if (robot.getRobotPos().yMeters > oppositeLauchZone && robot.getAlliance() == miniRobot.alliance.RED)
        //    targetPos = redOppositeCellPos;
        //
        //else if (robot.getRobotPos().yMeters > oppositeLauchZone && robot.getAlliance() == miniRobot.alliance.BLUE)
        //    targetPos = blueOppositeCellPos;

        actualTime = timer.milliseconds();

        if (isInit)
        {
            if (controlMode.allowsStateMachine())
                RunStateMachine();

            switch (controlMode)
            {
                case POSITION_DUTYCYCLE_PID :
                    switch (systemState)
                    {
                        case IDLE:
                            turretIO.applyPower(0.0);
                            break;
                        case SEARCHING_PUBLIC_CELL:
                        case ALIGNED_WITH_PUBLIC_CELL:
                            pidTurret.setSetpoint(
                                    0.0//parametersCalculator.getTurretRadiantTargetWithTargetPosition(TurretConstants.bluePublicCellePos) * TurretConstants.TICKS_PER_RADIANT
                            );
                            turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                            break;
                        case SEARCHING_OPPOSITE_CELL:
                        case ALIGNED_WITH_OPPOSITE_CELL:
                            pidTurret.setSetpoint(
                                    0.0//parametersCalculator.getTurretRadiantTargetWithTargetPosition(TurretConstants.blueOppositeCellPos) * TurretConstants.TICKS_PER_RADIANT
                            );
                            turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                            break;
                        case SEARCHING_PUBLIC_SIDE_CENTER:
                        case ALIGNED_WITH_PUBLIC_SIDE_CENTER:
                            pidTurret.setSetpoint(
                                    0.0//parametersCalculator.getTurretRadiantTargetWithTargetPosition(TurretConstants.publicSideCenter) * TurretConstants.TICKS_PER_RADIANT
                            );
                            turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                            break;
                        case SEARCHING_OPPOSITE_SIDE_CENTER:
                        case ALIGNED_WITH_OPPOSITE_SIDE_CENTER:
                            pidTurret.setSetpoint(
                                    0.0//parametersCalculator.getTurretRadiantTargetWithTargetPosition(TurretConstants.oppositeSideCenter) * TurretConstants.TICKS_PER_RADIANT
                            );
                            turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                            break;
                        default:
                            throw new IllegalStateException("turret system state isn't valid");
                    }
                    break;
            }

        }

        telemetry.addData("TargetPosTicks", pidTurret.getSetpoint());
        telemetry.addData("TargetPosRadiant", pidTurret.getSetpoint() / TurretConstants.TICKS_PER_RADIANT);
        telemetry.addLine();
        telemetry.addData("CurrentHeadingTicks", turretIO.getHeadingTicks());
        telemetry.addData("CurrentHeadingRadiant", turretIO.getHeadingRadiant());
        telemetry.addData("CurrentHeadingDegrees", turretIO.getHeadingTicks() / TurretConstants.TICKS_PER_DEGREES);
        telemetry.addLine();
        telemetry.addData("TurretMotorPower", turretIO.getTurretVoltage());
        telemetry.addLine();
        //ttTelemetry().addData("degrees angle from origin to red public cell",Math.toDegrees(calculateTargetRadiantWithPosition(TurretConstants.redPublicCellPos) ));
        telemetry.addData("Turret wanted state", wantedState);
        telemetry.addData("Turret system state", systemState);
    }

    public void resetTurret()
    {
        turretIO.resetMotor();
        pidTurret.reset(turretIO.getHeadingTicks());
    }

    private void RunStateMachine(){
        switch (wantedState){
            case STAND_BY:
                if (systemState != SystemState.IDLE)
                    pidTurret.reset(turretIO.getHeadingTicks());

                systemState = SystemState.IDLE;
                break;
            case STOP:
                break;
            case TRACK_PUBLIC_CELL:
                break;
            case TRACK_OPPOSITE_CELL:
                break;
            case TRACK_PUBLIC_SIDE_CENTER:

                if (systemState != SystemState.ALIGNED_WITH_PUBLIC_SIDE_CENTER)
                    systemState = SystemState.SEARCHING_PUBLIC_SIDE_CENTER;
                break;

            case TRACK_OPPOSITE_SIDE_CENTER:

                if (systemState != SystemState.ALIGNED_WITH_OPPOSITE_SIDE_CENTER)
                    systemState = SystemState.SEARCHING_OPPOSITE_SIDE_CENTER;
                break;
            default:
                telemetry.addLine("Please enter a valid turret Wanted state");
        }

        boolean alignedWithTarget = UtilsRBL.isInRange(turretIO.getHeadingTicks(), pidTurret.getSetpoint(), TurretConstants.turretTolerance);

        switch (systemState)
        {
            case IDLE:
                break;

            case SEARCHING_PUBLIC_CELL:
                break;
            case SEARCHING_OPPOSITE_CELL:
                break;
            case SEARCHING_PUBLIC_SIDE_CENTER:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_PUBLIC_SIDE_CENTER;
                break;

            case SEARCHING_OPPOSITE_SIDE_CENTER:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_OPPOSITE_SIDE_CENTER;
                break;

            case ALIGNED_WITH_PUBLIC_CELL:
                break;
            case ALIGNED_WITH_OPPOSITE_CELL:
                break;
            case ALIGNED_WITH_PUBLIC_SIDE_CENTER:
            case ALIGNED_WITH_OPPOSITE_SIDE_CENTER:
                break;

            default:
                throw new IllegalStateException("turret system state isn't valid");
        }
    }

    //public void init(){
    //    if (!turretIO.isHallSensorTrigered()){
    //        turretIO.applyPower(0.5);
    //    }
    //    else
    //    {
    //        if (turretIO.getHeadingRadiant() >180){
    //        turretIO.resetMotor();
    //        isInit = true;
    //        }
    //    }
    //}


}
