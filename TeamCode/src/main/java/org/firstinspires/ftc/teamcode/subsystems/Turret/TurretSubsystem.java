package org.firstinspires.ftc.teamcode.subsystems.turret;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxNackException;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.command.SubsystemBase;

import org.firstinspires.ftc.LyonLib.control.PidRBL;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.teamcode.tests.miniRobot;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretConstants;
import org.firstinspires.ftc.teamcode.subsystems.turret.TurretIO;

import java.util.function.Supplier;

@Config
public class TurretSubsystem extends SubsystemBase {

    private miniRobot robot;

    private ElapsedTime timer;
    private Pose2d targetPos = new Pose2d();
    private double targetAngle = 0.0;
    private PidRBL pidTurret;
    private TurretIO turretIO;
    private Supplier<Pose2d> getRobotPos;

    private double actualTime = 0.0;
    boolean isInit = true;



    public enum WantedState{
        STAND_BY,
        TRACK_BLUE_PUBLIC_CELL,
        TRACK_RED_PUBLIC_CELL,
        TRACK_BLUE_OPPOSITE_CELL,
        TRACK_RED_OPPOSITE_CELL,
        TRACK_PUBLIC_SIDE_CENTER,
        TRACK_OPPOSITE_SIDE_CENTER,
        TRACK_TARGET_POS,
        GO_TO_ANGLE
    }
    public enum SystemState{
        IDLE,

        SEARCHING_BLUE_PUBLIC_CELL,
        SEARCHING_RED_PUBLIC_CELL,
        SEARCHING_BLUE_OPPOSITE_CELL,
        SEARCHING_RED_OPPOSITE_CELL,

        SEARCHING_PUBLIC_SIDE_CENTER,
        SEARCHING_OPPOSITE_SIDE_CENTER,
        SEARCHING_TARGET_POS,
        GOING_TO_ANGLE,

        RESETING_ITSELF,

        ALIGNED_WITH_BLUE_PUBLIC_CELL,
        ALIGNED_WITH_RED_PUBLIC_CELL,
        ALIGNED_WITH_BLUE_OPPOSITE_CELL,
        ALIGNED_WITH_RED_OPPOSITE_CELL,

        ALIGNED_WITH_PUBLIC_SIDE_CENTER,
        ALIGNED_WITH_OPPOSITE_SIDE_CENTER,
        ALIGNED_WITH_TARGET_POS,
        ALIGNED_WITH_ANGLE

    }
    private WantedState wantedState = WantedState.STAND_BY;
    private SystemState systemState = SystemState.IDLE;
    private SystemState previousState = SystemState.IDLE;

    public void setWantedState(WantedState state){wantedState = state;}
    public SystemState getSystemState(){return systemState;}

    public TurretIO getTurretIO() {
        return turretIO;
    }

    public void setTargetPos(Pose2d target){targetPos = target;}

    public void setTargetAngle(double angle){targetAngle = angle;}

    public PidRBL getPidTurret(){return pidTurret;}

    public TurretSubsystem(HardwareMap hmap, miniRobot robot, Supplier<Pose2d> getPos){

        getRobotPos = getPos;

        this.robot = robot;

        turretIO = new TurretIO(hmap);

        pidTurret = new PidRBL(TurretConstants.turretKP,TurretConstants.turretKI , TurretConstants.turretKD);
        pidTurret.reset(turretIO.getHeadingTicks());
        pidTurret.setOutputLimits(-1, 1);
        //pidTurret.setContinuous(true);

        systemState = SystemState.IDLE;
        wantedState = WantedState.STAND_BY;

        timer = new ElapsedTime();
        timer.startTime();
    }

    @Override
    public void periodic(){
        turretIO.update();
        pidTurret.setGains(TurretConstants.turretKP,TurretConstants.turretKI , TurretConstants.turretKD);

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
        //CONDITION DANS LAQUELLE IL FAUT DEMELER LES CABLE  : (turretIO.isHallSensorTrigered() && (turretIO.getHeadingRadiant() > 1.5 * Math.PI || turretIO.getHeadingRadiant() < -1.5 * Math.PI))

        if (isInit)
        {
            RunStateMachine();
            //if ((turretIO.isHallSensorTrigered() && (turretIO.getHeadingRadiant() > 1.5 * Math.PI || turretIO.getHeadingRadiant() < -1.5 * Math.PI)))
            //{
            //    previousState = systemState;
            //    systemState = SystemState.RESETING_ITSELF;
            //}
            switch (systemState)
            {
                case IDLE:
                    turretIO.applyPower(0.0);
                    break;
                case SEARCHING_BLUE_PUBLIC_CELL:
                case ALIGNED_WITH_BLUE_PUBLIC_CELL:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(TurretConstants.bluePublicCellePos) * TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                case SEARCHING_RED_PUBLIC_CELL:
                case ALIGNED_WITH_RED_PUBLIC_CELL:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(TurretConstants.redPublicCellPos)* TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                case SEARCHING_BLUE_OPPOSITE_CELL:
                case ALIGNED_WITH_BLUE_OPPOSITE_CELL:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(TurretConstants.blueOppositeCellPos)* TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                case SEARCHING_RED_OPPOSITE_CELL:
                case ALIGNED_WITH_RED_OPPOSITE_CELL:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(TurretConstants.redOppositeCellPos)* TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                case SEARCHING_PUBLIC_SIDE_CENTER:
                case ALIGNED_WITH_PUBLIC_SIDE_CENTER:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(TurretConstants.publicSideCenter)* TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                case SEARCHING_OPPOSITE_SIDE_CENTER:
                case ALIGNED_WITH_OPPOSITE_SIDE_CENTER:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(TurretConstants.oppositeSideCenter)* TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                case SEARCHING_TARGET_POS:
                case ALIGNED_WITH_TARGET_POS:
                    pidTurret.setSetpoint(
                            calculateTargetRadiantWithPosition(targetPos) * TurretConstants.TICKS_PER_RADIANT
                    );
                    turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    break;
                //case GOING_TO_ANGLE:
                //case ALIGNED_WITH_ANGLE:
                   // pidTurret.setSetpoint(targetAngle * TurretConstants.TICKS_PER_RADIANT);
                    //turretIO.applyPower(pidTurret.calculateWithRealTime(turretIO.getHeadingTicks(), actualTime));
                    //break;
                //case RESETING_ITSELF:
                    //pidTurret.setSetpoint(0.0);
                    //break;
                default:
                    robot.getTelemetry().addLine("turret system state isn't valid");
                    break;
            }

        }

        robot.getTelemetry().addData("TargetPosTicks", pidTurret.getSetpoint());
        robot.getTelemetry().addData("TargetPosRadiant", pidTurret.getSetpoint() / TurretConstants.TICKS_PER_RADIANT);
        robot.getTelemetry().addLine();
        robot.getTelemetry().addData("CurrentHeadingTicks", turretIO.getHeadingTicks());
        robot.getTelemetry().addData("CurrentHeadingRadiant", turretIO.getHeadingRadiant());
        robot.getTelemetry().addData("CurrentHeadingDegrees", turretIO.getHeadingTicks() / TurretConstants.TICKS_PER_DEGREES);
        robot.getTelemetry().addLine();
        robot.getTelemetry().addData("TurretMotorPower", turretIO.getTurretVoltage());
        robot.getTelemetry().addLine();
        robot.getTelemetry().addData("degrees angle from origin to red public cell",Math.toDegrees(calculateTargetRadiantWithPosition(TurretConstants.redPublicCellPos) ));
        robot.getTelemetry().addLine();
        robot.getTelemetry().addData("Turret wanted state", wantedState);
        robot.getTelemetry().addData("Turret system state", systemState);
        //robot.getTelemetry().addData("output of my superbe fonction", calculateTargetRadiantWithPosition(TurretConstants.redPublicCellPos));
        //robot.getTelemetry().addData("pos Robot", robot.getRobotPos());
    }

    private void RunStateMachine(){
        switch (wantedState){
            case STAND_BY:
                if (systemState != SystemState.IDLE)
                    pidTurret.reset(turretIO.getHeadingTicks());

                systemState = SystemState.IDLE;
                break;
            case TRACK_BLUE_PUBLIC_CELL:

                if (systemState != SystemState.ALIGNED_WITH_BLUE_PUBLIC_CELL)
                    systemState = SystemState.SEARCHING_BLUE_PUBLIC_CELL;
                break;

            case TRACK_RED_PUBLIC_CELL:

                if (systemState != SystemState.ALIGNED_WITH_RED_PUBLIC_CELL)
                    systemState = SystemState.SEARCHING_RED_PUBLIC_CELL;
                break;

            case TRACK_BLUE_OPPOSITE_CELL:

                if (systemState != SystemState.ALIGNED_WITH_BLUE_OPPOSITE_CELL)
                    systemState = SystemState.SEARCHING_BLUE_OPPOSITE_CELL;
                break;

            case TRACK_RED_OPPOSITE_CELL:

                if (systemState != SystemState.ALIGNED_WITH_RED_OPPOSITE_CELL)
                    systemState = SystemState.SEARCHING_RED_OPPOSITE_CELL;
                break;

            case TRACK_PUBLIC_SIDE_CENTER:

                if (systemState != SystemState.ALIGNED_WITH_PUBLIC_SIDE_CENTER)
                    systemState = SystemState.SEARCHING_PUBLIC_SIDE_CENTER;
                break;

            case TRACK_OPPOSITE_SIDE_CENTER:

                if (systemState != SystemState.ALIGNED_WITH_OPPOSITE_SIDE_CENTER)
                    systemState = SystemState.SEARCHING_OPPOSITE_SIDE_CENTER;
                break;
            case TRACK_TARGET_POS:
                if (systemState != SystemState.ALIGNED_WITH_TARGET_POS)
                    systemState = SystemState.SEARCHING_TARGET_POS;
                break;

            case GO_TO_ANGLE:
                if (systemState != SystemState.ALIGNED_WITH_ANGLE)
                    systemState = SystemState.GOING_TO_ANGLE;
                break;
            default:
                robot.getTelemetry().addLine("Please enter a valid turret Wanted state");
        }

        boolean alignedWithTarget = UtilsRBL.isInRange(turretIO.getHeadingTicks(), pidTurret.getSetpoint(), TurretConstants.turretTolerance);

        switch (systemState)
        {
            case IDLE:
                break;
            case SEARCHING_BLUE_PUBLIC_CELL:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_BLUE_PUBLIC_CELL;
                break;

            case SEARCHING_RED_PUBLIC_CELL:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_RED_PUBLIC_CELL;
                break;

            case SEARCHING_BLUE_OPPOSITE_CELL:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_BLUE_OPPOSITE_CELL;
                break;

            case SEARCHING_RED_OPPOSITE_CELL:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_RED_OPPOSITE_CELL;
                break;

            case SEARCHING_PUBLIC_SIDE_CENTER:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_PUBLIC_SIDE_CENTER;
                break;

            case SEARCHING_OPPOSITE_SIDE_CENTER:

                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_OPPOSITE_SIDE_CENTER;
                break;
            case SEARCHING_TARGET_POS:
                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_TARGET_POS;
                break;
            case GOING_TO_ANGLE:
                if (alignedWithTarget)
                    systemState = SystemState.ALIGNED_WITH_ANGLE;
                break;
            case RESETING_ITSELF:
                if (alignedWithTarget)
                {
                    systemState = previousState;
                    previousState = SystemState.IDLE; //In case previous state is just shit and it doesn't work
                }
                break;
            case ALIGNED_WITH_BLUE_PUBLIC_CELL:
            case ALIGNED_WITH_RED_PUBLIC_CELL:
            case ALIGNED_WITH_BLUE_OPPOSITE_CELL:
            case ALIGNED_WITH_RED_OPPOSITE_CELL:
            case ALIGNED_WITH_PUBLIC_SIDE_CENTER:
            case ALIGNED_WITH_OPPOSITE_SIDE_CENTER:
            case ALIGNED_WITH_TARGET_POS:
            case ALIGNED_WITH_ANGLE:
                break;
            default:
                robot.getTelemetry().addLine("turret system state isn't valid");
                break;
        }
    }

    private double calculateTargetRadiantWithPosition(Pose2d targetPos1) {
        this.targetPos = targetPos1;
        double xRobot = getRobotPos.get().xMeters;
        double yRobot = getRobotPos.get().yMeters;
        double headingRobot = getRobotPos.get().headingRadians;

        double xTargetRobot = Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos.xMeters - xRobot) + Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos.yMeters - yRobot);
        double yTargetRobot = -Math.sin(headingRobot - TurretConstants.offsetHeading) * (targetPos.xMeters - xRobot) + Math.cos(headingRobot - TurretConstants.offsetHeading) * (targetPos.yMeters - yRobot);

        return Math.atan2(yTargetRobot, xTargetRobot);
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

    //public double calculateTargetTicksFromRadiant(double radiants){
    //    return radiants * TurretConstants.TICKS_PER_RADIANT;
    //}


}
