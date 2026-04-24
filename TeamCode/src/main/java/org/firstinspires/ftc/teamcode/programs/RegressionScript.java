package org.firstinspires.ftc.teamcode.programs;

import static org.firstinspires.ftc.teamcode.robotconfigs.Inferno.BallPath.LOW;
import static java.lang.Math.sqrt;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.teamcode.robotconfigs.Fisiks;
import org.firstinspires.ftc.teamcode.robotconfigs.Inferno;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;

public class RegressionScript {
    public static void main(String[] args){
        ArrayList<ArrayList<Double>> points = new ArrayList<>();
        double[] targetPoint = new double[]{141.5,141.5,46};
        int count = 0;
        for (double i=24; i<=144; i+=8){
            for (int x = -200; x<=100; x+=50){
                count++;
                double dist = i*sqrt(2);
                double vel = Inferno.VelRegression.regressFormula(i)+x;
                double time = Fisiks.runPhysics(LOW, targetPoint, new Pose(targetPoint[0]-i,targetPoint[1]-i),new Vector(),vel)[2];
                if (!(time==0.7 && !Fisiks.success)) points.add(new ArrayList<>(Arrays.asList(dist,vel,time)));
            }
        }
        System.out.println(count);
        System.out.println(points);
        System.out.println(points.size());
    }
}
