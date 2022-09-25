package frc.robot.utilities;
import java.util.ArrayList;

/**
 * Class used to track distances or values at certain times
 */
public class DistanceVsTime {
    private ArrayList<Double> distances;
    private ArrayList<Double> times;
 
    public  DistanceVsTime(){
        distances = new ArrayList<Double>();
        times = new ArrayList<Double>();
    }

    /**
     * adds a value and time to arraylists
     * @param val value at time
     * @param time in milliseconds
     */
    public void addValue(double val, double time){
        distances.add(val);
        times.add(time);
        if(distances.size() > 5){
            distances.remove(0);
            times.remove(0);
        }
    }

    /**
     * 
     * @param time in milliseconds
     * @return Value at time
     */
    public double getValue(double time){
        double val;
        double startDistance = distances.get(0);
        double endDistance = distances.get(4);
        double startTime = times.get(0);
        double endTime = times.get(4);
        if (time < startTime){
            // before first time in arraylist
            endTime = times.get(1);
            endDistance = distances.get(1);
        }
        else if(time > endTime){
            // after last time in arraylist
            startTime = times.get(3);
            startDistance = distances.get(3);
        }
        else{
            // between first and last times in arraylist
            for (int i = 0; i < times.size(); i++){
                if(times.get(i) <= time && times.get(i+1) >= time){
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
        System.out.println("value: " + val);
        return val;
    }
}
