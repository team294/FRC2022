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
        double startDistance = 0;
        double endDistance = distances.get(4);
        double startTime = 0;
        double endTime = times.get(4);
        boolean greater = false;
        for (int i = 0; i < 5; i++){
            if (greater){
                if(time < times.get(i)){
                    endDistance = distances.get(i);
                    endTime = times.get(i);
                }
                else{
                    startDistance = distances.get(i);
                    startTime = times.get(i);
                }
            }
            else if (time > times.get(i)){
                greater = true;
                startDistance = distances.get(i);
                startTime = times.get(i);

            }
        }
        double slope = (endTime - startTime)/(endDistance - startDistance);
        double intercept = endTime - slope * endDistance;
        val = slope * time + intercept;
        return val;
    }
}
