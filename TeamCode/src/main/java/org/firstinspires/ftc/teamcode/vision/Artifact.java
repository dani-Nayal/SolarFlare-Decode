package org.firstinspires.ftc.teamcode.vision;

import java.util.List;

public class Artifact {
    double x;
    double y;
    String className;
    List<List<Double>> corners;
    double bottomCenterXPixels;
    double bottomCenterYPixels;
    double centerXPixels;
    double centerYPixels;
    double topCenterXPixels;
    double topCenterYPixels;
    public Artifact(double x, double y, String className){
        this.x = x;
        this.y = y;
        this.className = className;
    }
    public Artifact(double x, double y, String className, List<List<Double>> corners){
        this.x = x;
        this.y = y;
        this.className = className;
        this.corners = corners;

    }
    public double getX() {return x;}
    public void setX(double x) {this.x = x;}
    public double getY() {return y;}
    public void setY(double y) {this.y = y;}
    public String getClassName() {return className;}
    public void setClassName(String className) {this.className = className;}
    public Double getTopCenterXPixels(Vision.CAMERA_ORIENTATION orientation) {
        double topCenterX = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            topCenterX = (corner1.get(0) + corner0.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            topCenterX = (corner2.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            topCenterX = (corner0.get(0) + corner3.get(0)) / 2;
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            topCenterX = (corner2.get(0) + corner1.get(0)) / 2;
        }

        this.topCenterXPixels = topCenterX;

        return topCenterX;
    }
    public void setTopCenterXPixels(double topCenterXPixels){
        this.topCenterXPixels = topCenterXPixels;
    }
    public Double getTopCenterYPixels(Vision.CAMERA_ORIENTATION orientation) {
        double topCenterY = 0;

        List<Double> corner0 = corners.get(0);
        List<Double> corner1 = corners.get(1);
        List<Double> corner2 = corners.get(2);
        List<Double> corner3 = corners.get(3);

        if (orientation == Vision.CAMERA_ORIENTATION.NORMAL){
            topCenterY = corner1.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.UPSIDE_DOWN){
            topCenterY = corner3.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.CLOCKWISE_90){
            topCenterY = corner0.get(1);
        }
        else if (orientation == Vision.CAMERA_ORIENTATION.COUNTER_CLOCKWISE_90){
            topCenterY = corner2.get(1);
        }

        this.topCenterYPixels = topCenterY;

        return topCenterY;
    }
    public void setTopCenterYPixels(double topCenterYPixels){
        this.topCenterYPixels = topCenterYPixels;
    }

}


