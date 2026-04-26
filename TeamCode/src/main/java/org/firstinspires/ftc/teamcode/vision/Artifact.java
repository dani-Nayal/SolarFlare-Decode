package org.firstinspires.ftc.teamcode.vision;

public class Artifact {
    double x;
    double y;
    String className;
    double bottomCenterXPixels;
    double bottomCenterYPixels;
    double centerXPixels;
    double centerYPixels;
    double topCenterXPixels;
    double topCenterYPixels;
    double txTopCenter;
    double tyTopCenter;
    public Artifact(double x, double y, String className){
        this.x = x;
        this.y = y;
        this.className = className;
    }
    public Artifact(double x, double y, String className, double centerXPixels, double centerYPixels, double bottomCenterXPixels, double bottomCenterYPixels, double topCenterXPixels, double topCenterYPixels, double txTopCenter, double tyTopCenter){
        this.x = x;
        this.y = y;
        this.className = className;
        this.centerXPixels = centerXPixels;
        this.centerYPixels = centerYPixels;
        this.bottomCenterXPixels = bottomCenterXPixels;
        this.bottomCenterYPixels = bottomCenterYPixels;
        this.topCenterXPixels = topCenterXPixels;
        this.topCenterYPixels = topCenterYPixels;
        this.txTopCenter = txTopCenter;
        this.tyTopCenter = tyTopCenter;
    }
    public double getX() {return x;}
    public void setX(double x) {this.x = x;}
    public double getY() {return y;}
    public void setY(double y) {this.y = y;}
    public String getClassName() {return className;}
    public void setClassName(String className) {this.className = className;}
    public double getBottomCenterXPixels() {return bottomCenterXPixels;}
    public double getBottomCenterYPixels() {return bottomCenterYPixels;}
    public void setBottomCenterXPixels(double bottomCenterXPixels) {this.bottomCenterXPixels = bottomCenterXPixels;}
    public void setBottomCenterYPixels(double bottomCenterYPixels) {this.bottomCenterYPixels = bottomCenterYPixels;}
    public double getCenterXPixels() {return centerXPixels;}
    public void setCenterXPixels(double centerXPixels) {this.centerXPixels = centerXPixels;}
    public double getCenterYPixels() {return centerYPixels;}
    public void setCenterYPixels(double centerYPixels) {this.centerYPixels = centerYPixels;}
    public double getTopCenterYPixels() {return topCenterYPixels;}
    public void setTopCenterYPixels(double topCenterYPixels) {this.topCenterYPixels = topCenterYPixels;}
    public double getTopCenterXPixels() {return topCenterXPixels;}
    public void setTopCenterXPixels(double topCenterXPixels) {this.topCenterXPixels = topCenterXPixels;}
    public double getTxTopCenter() {return txTopCenter;}
    public void setTxTopCenter(double txCenter) {this.txTopCenter = txCenter;}
    public double getTyTopCenter() {return tyTopCenter;}
    public void setTyTopCenter(double tyCenter) {this.tyTopCenter = tyCenter;}
}
