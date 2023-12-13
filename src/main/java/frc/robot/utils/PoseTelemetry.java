package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class PoseTelemetry {
    
    Field2d field = new Field2d();

    Pose2d actPose = new Pose2d();
    Pose2d desPose = new Pose2d();
    Pose2d estPose = new Pose2d();

    public PoseTelemetry() {
        update();
    }

    public void update() {
        field.getObject("DesPose").setPose(desPose);
        field.getObject("ActPose").setPose(actPose);
        field.getObject("Robot").setPose(estPose);
    }

    public void setActualPose(Pose2d in) {
        actPose = in;
    }

    public void setDesiredPose(Pose2d in) {
        desPose = in;
    }

    public void setEstimatedPose(Pose2d in) {
        estPose = in;
    }
}
