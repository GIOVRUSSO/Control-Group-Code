package ie.ucd.smartrideRT;

import java.util.ArrayList;

public class PlottingData {
    private float recentMTarget;
    private MValuePlottingData recentMActual;
    private ArrayList<Float> inputToCharacteristic;
    private ArrayList<Float> outputOfCharacteristic;
    private float recentHumanPowerAveraged;
    private float motorReferencePower;
    private float motorActualPower;

    public PlottingData(Float recentMTarget,
                        MValuePlottingData recentMActual,
                        ArrayList<Float> inputToCharacteristic,
                        ArrayList<Float> outputOfCharacteristic,
                        float recentHumanPowerAveraged,
                        float motorReferencePower,
                        float motorActualPower){
        this.recentMTarget = recentMTarget;
        this.recentMActual = recentMActual;
        this.inputToCharacteristic = inputToCharacteristic;
        this.outputOfCharacteristic = outputOfCharacteristic;
        this.recentHumanPowerAveraged = recentHumanPowerAveraged;
        this.motorReferencePower = motorReferencePower;
        this.motorActualPower = motorActualPower;
    }

    public float getRecentMTarget(){
        return recentMTarget;
    }

    public MValuePlottingData getRecentMActual(){
        return recentMActual;
    }

    public ArrayList<Float> getInputToCharacteristic(){ return inputToCharacteristic; }

    public ArrayList<Float> getOutputOfCharacteristic(){ return outputOfCharacteristic; }

    public float getRecentHumanPowerAveraged() {
        return recentHumanPowerAveraged;
    }

    public float getMotorReferencePower(){
        return motorReferencePower;
    }

    public float getMotorActualPower(){
        return motorActualPower;
    }
}
