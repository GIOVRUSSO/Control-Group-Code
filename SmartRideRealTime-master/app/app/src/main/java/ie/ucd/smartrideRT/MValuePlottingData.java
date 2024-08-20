package ie.ucd.smartrideRT;

import java.util.Date;

public class MValuePlottingData {
    private Date date;
    private float mValue;

    public MValuePlottingData(Date date, float mValue){
        this.date = date;
        this.mValue = mValue;
    }

    public Date getDate(){
        return date;
    }

    public float getMValue(){
        return mValue;
    }
}
