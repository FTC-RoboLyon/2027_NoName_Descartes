package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(group = "tests")
public class CameraTests extends OpMode {
    private Limelight3A cam_ille;

    @Override
    public void init()
    {
        cam_ille = hardwareMap.get(Limelight3A.class, "cam_ille");
        cam_ille.setPollRateHz(100);
        cam_ille.start();
    }

    @Override
    public void loop()
    {
        LLResult result = cam_ille.getLatestResult();
        if(result != null && result.isValid())
        {
            List<LLResultTypes.FiducialResult> aprilTags = result.getFiducialResults();
            if(!aprilTags.isEmpty())
            {
                for(LLResultTypes.FiducialResult tag : aprilTags)
                {
                    telemetry.addLine("AprilTag " + tag.getFiducialId());
                    telemetry.addData("Skew", tag.getSkew());
                    telemetry.addLine();
                }
            }
            else
            {
                telemetry.addLine("Show me your April Tags and I will tell you their skew");
            }
        }
        else
        {
            telemetry.addLine("can't analyse invalid result");
        }
        telemetry.update();
    }

    @Override
    public void stop()
    {
        cam_ille.stop();
    }
}
