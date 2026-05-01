package org.firstinspires.ftc.teamcode.vision;

import static org.apache.commons.math3.util.FastMath.cos;
import static org.apache.commons.math3.util.FastMath.sin;
import static org.apache.commons.math3.util.FastMath.tan;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class Artifact {
    final double ARTIFACT_RADIUS_INCHES = 2.45;
    public final double INTAKING_RAMP_SLOPE = 0.45977011494;
    public final double fx = 1218.145;
    public final double fy = 1219.418;
    public final double cx = 621.829;
    public final double cy = 500.362;
    public final double fxNN = fx / 4;
    public final double fyNN = fy / 4;
    public final double cxNN = cx / 4;
    public final double cyNN = cy / 4;
    public Pose3D cameraPoseOnRobot = new Pose3D(new Position(DistanceUnit.METER, 0.182, 0, 0.2225, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0));
    public double x;
    public double y;
    public double z;
    public String className;
    public double txBottomCenter;
    public double tyBottomCenter;
    public double txCenter;
    public double tyCenter;
    public List<List<Double>> corners;
    public ARTIFACT_TYPE artifactType;
    public double zDiff;
    public enum ARTIFACT_TYPE{
        GROUND,
        CLASSIFIER,
        INVALID
    }
    public Artifact(LLResultTypes.DetectorResult detectorResult, Vision.CAMERA_ORIENTATION cameraOrientation, Pose botPose){
        if (detectorResult == null) {
            artifactType = ARTIFACT_TYPE.INVALID;
            return;
        }

        this.txCenter = detectorResult.getTargetXDegrees();
        this.tyCenter = detectorResult.getTargetYDegrees();

        this.className = detectorResult.getClassName();

        this.corners = detectorResult.getTargetCorners();

        double targetYTopCenter = getTopCenterYPixels(cameraOrientation, corners);

        double yTopOffset = cyNN - targetYTopCenter;

        double tyTopCenter = Math.toDegrees(Math.atan(yTopOffset / fyNN));

        double targetX = getBottomCenterXPixels(cameraOrientation, corners);
        double targetY = getBottomCenterYPixels(cameraOrientation, corners);

        double xOffset = targetX - cxNN;
        double yOffset = cyNN - targetY;

        this.txBottomCenter = Math.toDegrees(Math.atan(xOffset / fxNN));
        this.tyBottomCenter = Math.toDegrees(Math.atan(yOffset / fyNN));

        if (tyTopCenter < 0){
            this.artifactType = ARTIFACT_TYPE.GROUND;

            double verticalAngleDeg = tyCenter + 90 + cameraPoseOnRobot.getOrientation().getPitch(AngleUnit.DEGREES);

            double relativeX = ((cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z - ARTIFACT_RADIUS_INCHES) * Math.tan(Math.toRadians(verticalAngleDeg)));

            double relativeY = -(relativeX * Math.tan(Math.toRadians(txCenter + cameraPoseOnRobot.getOrientation().getYaw(AngleUnit.DEGREES))));

            // relativeY depends on relativeX being relative to camera, not center of robot
            relativeX += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x;
            relativeY += cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y;

            double botPoseX = botPose.getX();
            double botPoseY = botPose.getY();
            double theta = botPose.getHeading();

            double cos = Math.cos(theta);
            double sin = Math.sin(theta);

            this.x = botPoseX + relativeX * cos - (relativeY) * sin;
            this.y = botPoseY + relativeX * sin + (relativeY) * cos;
            this.z = 0;

            if (!(x >= 0 && x <= 144 && y >= 0 && y <= 144)){
                this.artifactType = ARTIFACT_TYPE.INVALID;
            }
        }
        else {
            double horizontalAngle = -txCenter + Math.toDegrees(botPose.getHeading());
            double cameraDist = Math.sqrt(cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x*cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).x+cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y*cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).y);

            double limelightX = botPose.getX()+cos(botPose.getHeading())*cameraDist;
            double limelightY = botPose.getY()+sin(botPose.getHeading())*cameraDist;
            double limelightZ = cameraPoseOnRobot.getPosition().toUnit(DistanceUnit.INCH).z;

            this.x = cos(Math.toRadians(horizontalAngle))>0 ? 144-3.5: 3.5;

            double hyp = (this.x-limelightX)/cos(Math.toRadians(horizontalAngle));
            this.y = hyp*sin(Math.toRadians(horizontalAngle))+limelightY;

            this.z = tan(Math.toRadians(tyCenter)) * hyp + limelightZ;

            double predictedZ = INTAKING_RAMP_SLOPE * (y - 70.5) + 8.75;

            this.zDiff = z - predictedZ;

            if (zDiff < 10){
                artifactType = ARTIFACT_TYPE.CLASSIFIER;
            }
            else{
                artifactType = ARTIFACT_TYPE.INVALID;
            }
        }
    }
    public Double getTopCenterXPixels(Vision.CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetXPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            targetXPixels = (corner1.get(0) + corner0.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetXPixels = (corner2.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            targetXPixels = (corner0.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetXPixels = (corner2.get(0) + corner1.get(0)) / 2;
        }

        return targetXPixels;
    }
    public Double getTopCenterYPixels(Vision.CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetYPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            targetYPixels = corner1.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetYPixels = corner3.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            targetYPixels = corner0.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetYPixels = corner2.get(1);
        }

        return targetYPixels;
    }
    public Double getBottomCenterXPixels(Vision.CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetXPixels = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            targetXPixels = (corner2.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetXPixels = (corner0.get(0) + corner1.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            targetXPixels = (corner1.get(0) + corner2.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetXPixels = (corner3.get(0) + corner0.get(0)) / 2;
        }

        return targetXPixels;
    }

    public Double getBottomCenterYPixels(Vision.CAMERA_ORIENTATION orientation, List<List<Double>> corners) {
        if (corners == null) return null;

        double targetYPixels;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            targetYPixels = corner3.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            targetYPixels = corner1.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            targetYPixels = corner2.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            targetYPixels = corner0.get(1);
        }
        else return null;

        return targetYPixels;
    }
}
