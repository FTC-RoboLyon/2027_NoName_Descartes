package org.firstinspires.ftc.LyonLib.localization;


import org.firstinspires.ftc.LyonLib.drive.WheelIO;
import org.firstinspires.ftc.LyonLib.kinematics.DrivetrainKinematics;
import org.firstinspires.ftc.LyonLib.kinematics.Pose2d;
import org.firstinspires.ftc.LyonLib.logging.RealtimeLogger;
import org.firstinspires.ftc.LyonLib.utils.UtilsRBL;
import org.firstinspires.ftc.teamcode.subsystems.localization.LocalizationLimelight;

import java.util.ArrayList;
import java.util.Arrays;

public abstract class RobotLocalizer {
    private ArrayList<LocalizationSensor> sensors = new ArrayList<LocalizationSensor>();
    private ArrayList<LocalizationLimelight> MT2cameras = new ArrayList<>(); //store cameras that will use MegaTag2 algorithm to estimate pos

    protected Pose2d EstimatedPos = new Pose2d();
    private double[][] posErrorCovariance = {
            {0.02*0.02, 0,         0},
            {0,         0.02*0.02, 0},
            {0,         0,         (2*Math.PI/180)*(2*Math.PI/180)}
    };

    private boolean isInit;

    protected DrivetrainKinematics drivetrainKinematics;
    protected WheelIO.WheelIOInputs[] wheels;
    private Pose2d modelProcessNoise;

    protected RealtimeLogger logger;
    protected double lastTimestamp;

    public RobotLocalizer() {}

    public RobotLocalizer(RealtimeLogger logger)
    {
        this.logger = logger;
    }

    public void SetDrivetrainParameters(DrivetrainKinematics drivetrainKinematics, WheelIO.WheelIOInputs[] wheels, Pose2d modelProcessNoise)
    {
        this.wheels = wheels;
        this.drivetrainKinematics = drivetrainKinematics;
        this.modelProcessNoise = modelProcessNoise.copy();
    }

    /**
     * Allows to add a localization sensor to the filter.
     * If the localization sensor you want to add is a camera using MegaTag2
     * algorithm to estimate the robot pos see {@link RobotLocalizer#AddMT2Camera(LocalizationLimelight)}
     */
    public void AddSensor(LocalizationSensor sensor)
    {
        sensors.add(sensor);
    }

    /**
     * Allows to add a camera that uses MegaTag2 to locate the robot to the filter.
     * These cameras are separated from the other sensors cause they need a
     * heading value to estimate robot pos.
     * If you simply want to add a camera that doesn't need this heading value
     * you can simply use {@link RobotLocalizer#AddSensor(LocalizationSensor)}
     */
    public void AddMT2Camera(LocalizationLimelight camera)
    {
        MT2cameras.add(camera);
    }
    public void Init(Pose2d startingPos)
    {
        for (int i=0; i < sensors.size(); i++)
        {
            sensors.get(i).Init(startingPos.copy());
        }
        for (int i=0; i < MT2cameras.size(); i++)
        {
            MT2cameras.get(i).Init(new Pose2d());
        }
        isInit = true;
    }

    public void StartLocalizer(double timestamp)
    {
        lastTimestamp = timestamp;
    }

    public int GetNumberOfSensors()
    {
        return sensors.size()+MT2cameras.size();
    }

    public Pose2d Update(double timestamp)
    {
        if (!isInit)
        {
            //TODO
        }
        else
        {
            //1. Update values for each sensor
            for (int i = 0; i < sensors.size(); i++)
            {
                sensors.get(i).Update();
            }

            //2. Calculate estimated pos with the drivetrain model equations
            Pose2d predictedPos = CalculatePosWithModel(timestamp).copy();

            //3. Calculate predicted position error covariance.
            //   Calculations are simplified to be more efficient. If you want to understand where they come from, see the documentation
            double[][] predictedPosErrorCovariance = {
                    {0, 0, 0},
                    {0, 0, 0},
                    {0, 0, 0}
            };

            double dX = predictedPos.xMeters-EstimatedPos.xMeters;
            double dY = predictedPos.yMeters-EstimatedPos.yMeters;

            //Useful results :)
            double A = -Math.sin(EstimatedPos.headingRadians)*dX-Math.cos(EstimatedPos.headingRadians)*dY;
            double B = Math.cos(EstimatedPos.headingRadians)*dX-Math.sin(EstimatedPos.headingRadians)*dY;

            //Line 1 of the matrix
            predictedPosErrorCovariance[0][0] = posErrorCovariance[0][0]+A*(posErrorCovariance[2][0]+posErrorCovariance[0][2]+A*posErrorCovariance[2][2]) + modelProcessNoise.xMeters;
            predictedPosErrorCovariance[0][1] = posErrorCovariance[0][1]+A*posErrorCovariance[2][1]+B*(posErrorCovariance[0][2]+A*posErrorCovariance[2][2]);
            predictedPosErrorCovariance[0][2] = posErrorCovariance[0][2]+A*posErrorCovariance[2][2];

            //Line 2 of the matrix
            predictedPosErrorCovariance[1][0] = posErrorCovariance[1][0]+B*posErrorCovariance[2][0]+A*(posErrorCovariance[1][2]+B*posErrorCovariance[2][2]);
            predictedPosErrorCovariance[1][1] = posErrorCovariance[1][1]+B*(posErrorCovariance[2][1]+posErrorCovariance[1][2]+B*posErrorCovariance[2][2]) + modelProcessNoise.yMeters;
            predictedPosErrorCovariance[1][2] = posErrorCovariance[1][2]+B*posErrorCovariance[2][2];

            //Line 3 of the matrix
            predictedPosErrorCovariance[2][0] = posErrorCovariance[2][0]+A*posErrorCovariance[2][2];
            predictedPosErrorCovariance[2][1] = posErrorCovariance[2][1]+B*posErrorCovariance[2][2];
            predictedPosErrorCovariance[2][2] = posErrorCovariance[2][2] + modelProcessNoise.headingRadians;

            if (logger != null)
            {
                logger.log("RobotLocalizer/ModelEstimatedPos", predictedPos.toArray());
            }

            //4.Apply the Kalman filter for each sensor
            for (int i = 0; i < sensors.size(); i++)
            {
                RunKalmanFilter(predictedPos,predictedPosErrorCovariance, sensors.get(i));
            }

            for (int i = 0; i < MT2cameras.size(); i++)
            {
                MT2cameras.get(i).UpdateWithMT2(predictedPos.headingRadians);
                RunKalmanFilter(predictedPos,predictedPosErrorCovariance, MT2cameras.get(i));
            }

            posErrorCovariance = predictedPosErrorCovariance;
            EstimatedPos = predictedPos.copy();
        }

        if (logger != null)
        {
            logger.log("RobotLocalizer/EstimatedPos", EstimatedPos.toArray());
            logger.log("RobotLocalizer/PosErrorCovariance", Arrays.deepToString(posErrorCovariance));
        }
        return EstimatedPos;
    }

    public Pose2d GetLastEstimatedPos()
    {
        return EstimatedPos.copy();
    }

    public void ResetPos(Pose2d pos)
    {
        EstimatedPos = pos.copy();
        for (LocalizationSensor sensor : sensors)
        {
            sensor.SetPosition(pos.copy());
        }
    }

    private void RunKalmanFilter(Pose2d predictedPos, double[][] predictedPosErrorCovariance, LocalizationSensor sensor)
    {
        //1. Calculate the Kalman gain for the currently used sensor
        //   Calculations are simplified to be more efficient. If you want to understand where they come from, see the documentation
        Pose2d currentSensorAnalysedAxes = sensor.GetAnalysedAxes();
        Pose2d currentSensorStdDev = sensor.GetStdDev();

        //Finding the dividing matrix to invert it
        double[][] innovationCovariance = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };

        //Line 1
        innovationCovariance[0][0] = predictedPosErrorCovariance[0][0]*currentSensorAnalysedAxes.xMeters + currentSensorStdDev.xMeters*currentSensorStdDev.xMeters;
        innovationCovariance[0][1] = predictedPosErrorCovariance[0][1]*currentSensorAnalysedAxes.xMeters*currentSensorAnalysedAxes.yMeters;
        innovationCovariance[0][2] = predictedPosErrorCovariance[0][2]*currentSensorAnalysedAxes.xMeters*currentSensorAnalysedAxes.headingRadians;

        //Line 2
        innovationCovariance[1][0] = predictedPosErrorCovariance[1][0]*currentSensorAnalysedAxes.xMeters*currentSensorAnalysedAxes.yMeters;
        innovationCovariance[1][1] = predictedPosErrorCovariance[1][1]*currentSensorAnalysedAxes.yMeters + currentSensorStdDev.yMeters*currentSensorStdDev.yMeters;
        innovationCovariance[1][2] = predictedPosErrorCovariance[1][2]*currentSensorAnalysedAxes.yMeters*currentSensorAnalysedAxes.headingRadians;

        //Line 3
        innovationCovariance[2][0] = predictedPosErrorCovariance[2][0]*currentSensorAnalysedAxes.xMeters*currentSensorAnalysedAxes.headingRadians;
        innovationCovariance[2][1] = predictedPosErrorCovariance[2][1]*currentSensorAnalysedAxes.yMeters*currentSensorAnalysedAxes.headingRadians;
        innovationCovariance[2][2] = predictedPosErrorCovariance[2][2]*currentSensorAnalysedAxes.headingRadians + currentSensorStdDev.headingRadians*currentSensorStdDev.headingRadians;

        //Invert the dividing matrix using LU decomposition algorithm
        double[][] L = {
                {1, 0, 0},
                {0, 1, 0},
                {0, 0, 1}
        };

        double[][] U = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };
        for (int j = 0; j < 3; j++)
        {
            U[j] = innovationCovariance[j].clone();
        }

        //Row 1 of U & L
        if (U[0][0] != 0.0)
        {
            if (U[1][0] != 0.0)
            {
                L[1][0] = U[1][0]/U[0][0];

                U[1][0] = 0.0;
                U[1][1] -= U[0][1]*L[1][0];
                U[1][2] -= U[0][2]*L[1][0];
            }

            if (U[2][0] != 0.0)
            {
                L[2][0] = U[2][0]/U[0][0];

                U[2][0] = 0.0;
                U[2][1] -= U[0][1]*L[2][0];
                U[2][2] -= U[0][2]*L[2][0];
            }
        }

        //Row 2 of U & L
        if (U[1][1] != 0.0 && U[2][1] != 0.0)
        {
            L[2][1] = U[2][1]/U[1][1];

            U[2][0] -= U[1][0]*L[2][1];
            U[2][1] = 0.0;
            U[2][2] -= U[1][2]*L[2][1];
        }


        //Calculating the inverted matrix with U & L
        double[][] invertedInnovationCovariance = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };

        //Row 1 of invertedInnovationCovariance
        double row2Coef = -L[1][0];
        double row3Coef = -(L[2][0] + L[2][1]*row2Coef);

        double[] intermediateSolution = {1,
                row2Coef,
                row3Coef};

        row3Coef = intermediateSolution[2]/U[2][2];
        row2Coef = (intermediateSolution[1] - row3Coef*U[1][2]) / U[1][1];

        invertedInnovationCovariance[0][0] = (intermediateSolution[0] - row3Coef*U[0][2] - row2Coef*U[0][1]) / U[0][0];
        invertedInnovationCovariance[1][0] = row2Coef;
        invertedInnovationCovariance[2][0] = row3Coef;

        //Row 2 of invertedInnovationCovariance
        intermediateSolution = new double[] {0,
                1,
                -L[2][1]};

        row3Coef = intermediateSolution[2]/U[2][2];
        row2Coef = (intermediateSolution[1] - row3Coef*U[1][2]) / U[1][1];

        invertedInnovationCovariance[0][1] = (intermediateSolution[0] - row3Coef*U[0][2] - row2Coef*U[0][1]) / U[0][0];
        invertedInnovationCovariance[1][1] = row2Coef;
        invertedInnovationCovariance[2][1] = row3Coef;

        //Row 3 of invertedInnovationCovariance
        intermediateSolution = new double[] {0,
                0,
                1};

        row3Coef = intermediateSolution[2]/U[2][2];
        row2Coef = (intermediateSolution[1] - row3Coef*U[1][2]) / U[1][1];

        invertedInnovationCovariance[0][2] = (intermediateSolution[0] - row3Coef*U[0][2] - row2Coef*U[0][1]) / U[0][0];
        invertedInnovationCovariance[1][2] = row2Coef;
        invertedInnovationCovariance[2][2] = row3Coef;

        //Calculate the Kalman gain for the sensor
        double[][] K = {
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };

        for (int j = 0; j < 3; j++)
        {
            for (int k = 0; k < 3; k++)
            {
                K[j][k] = predictedPosErrorCovariance[j][0]*currentSensorAnalysedAxes.xMeters*invertedInnovationCovariance[0][k]
                        + predictedPosErrorCovariance[j][1]*currentSensorAnalysedAxes.yMeters*invertedInnovationCovariance[1][k]
                        + predictedPosErrorCovariance[j][2]*currentSensorAnalysedAxes.headingRadians*invertedInnovationCovariance[2][k];
            }
        }

        //2. Calculate the innovation of the sensor
        Pose2d innovation = sensor.GetLastEstimatedPos().copy();
        innovation.headingRadians = UtilsRBL.wrapNegPiToPi(innovation.headingRadians - predictedPos.headingRadians * currentSensorAnalysedAxes.headingRadians);
        innovation.xMeters -= predictedPos.xMeters * currentSensorAnalysedAxes.xMeters;
        innovation.yMeters -= predictedPos.yMeters * currentSensorAnalysedAxes.yMeters;

        //3. Apply correction
        Pose2d correction = new Pose2d(
                innovation.xMeters*K[0][0]+innovation.yMeters*K[0][1]+innovation.headingRadians*K[0][2],
                innovation.xMeters*K[1][0]+innovation.yMeters*K[1][1]+innovation.headingRadians*K[1][2],
                innovation.xMeters*K[2][0]+innovation.yMeters*K[2][1]+innovation.headingRadians*K[2][2]
        );

        predictedPos.xMeters += correction.xMeters;
        predictedPos.yMeters += correction.yMeters;
        predictedPos.headingRadians += correction.headingRadians;

        //4. Calculate new covariance
        double[][] updatedPosErrorCovariance ={
                {0, 0, 0},
                {0, 0, 0},
                {0, 0, 0}
        };

        for (int j =0; j < 3; j++)
        {
            updatedPosErrorCovariance[0][j] = (1-K[0][0]*currentSensorAnalysedAxes.xMeters)*predictedPosErrorCovariance[0][j] - K[0][1]*currentSensorAnalysedAxes.yMeters*predictedPosErrorCovariance[1][j] - K[0][2]*currentSensorAnalysedAxes.headingRadians*predictedPosErrorCovariance[2][j];
            updatedPosErrorCovariance[1][j] = -K[1][0]*currentSensorAnalysedAxes.xMeters*predictedPosErrorCovariance[0][j] + (1-K[1][1]*currentSensorAnalysedAxes.yMeters)*predictedPosErrorCovariance[1][j] - K[1][2]*currentSensorAnalysedAxes.headingRadians*predictedPosErrorCovariance[2][j];
            updatedPosErrorCovariance[2][j] = -K[2][0]*currentSensorAnalysedAxes.xMeters*predictedPosErrorCovariance[0][j] - K[2][1]*currentSensorAnalysedAxes.yMeters*predictedPosErrorCovariance[1][j] + (1-K[2][2]*currentSensorAnalysedAxes.headingRadians)*predictedPosErrorCovariance[2][j];
        }

        predictedPosErrorCovariance = updatedPosErrorCovariance.clone();

        if (logger != null)
        {
            logger.log("RobotLocalizer/"+ sensor.GetSensorName() + "EstimatedPos", sensor.GetLastEstimatedPos().toArray());
        }
    }

    public boolean[] AreSensorsReady()
    {
        boolean[] result = new boolean[sensors.size()];
        for (int i=0; i < sensors.size(); i++)
        {
            result[i] = sensors.get(i).IsReady();
        }
        return result;
    }

    protected abstract Pose2d CalculatePosWithModel(double timestamp);
}