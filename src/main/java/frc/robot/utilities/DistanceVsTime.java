package frc.robot.utilities;
import java.util.ArrayList;

public class DistanceVsTime {
    private ArrayList<Double> distances;
    private ArrayList<Double> times;
    
    
    public  DistanceVsTime(){
        distances = new ArrayList<Double>();
        times = new ArrayList<Double>();
    }

    public void addValue(double val, double time){
        distances.add(val);
        times.add(time);
        if(distances.size() > 5){
            distances.remove(0);
            times.remove(0);
        }
    }

    public double getValue(double time){
        double val;
        double startDistance = distances.get(0);
        double endDistance = distances.get(4);
        double startTime = times.get(0);
        double endTime = times.get(4);
        boolean greater = false;
        if (time < startTime){
            endTime = times.get(1);
            endDistance = distances.get(1);
        }
        else if(time > endTime){
            startTime = times.get(3);
            startDistance = distances.get(3);
        }
        else{
            for (int i = 0; i < times.size(); i++){
                if(times.get(i) < time && times.get(i+1) > time){
                    startTime = times.get(i);
                    endTime = times.get(i+1);
                    startDistance = distances.get(i);
                    endDistance = distances.get(i+1);
                }
            }
        }
        double slope = (endDistance - startDistance)/(endTime - startTime);
        double intercept = endDistance - slope * endTime;
        val = slope * time + intercept;
        System.out.println("\nSlope: " + slope + "\nintercept: " + intercept + "\nStart distance: " + startDistance + "\nEnd Distance: " + endDistance + "\ntime: " + time + "\nbefore: " + startTime + "\nafter: " + endTime + "\nvalue: " + val);
        return val;
    }
}
