package ie.ucd.smartrideRT;

/**
 * Created by apple on 25/05/2017.
 */

public class CooperativeBreathingControlData {
    private int _id;
    private String _commandSent;
    private float _mTarget;
    private float _fm;
    private float _samplingPeriod;


    public CooperativeBreathingControlData(String nextYString, float mTarget, float fm, float samplingPeriod){
        this._commandSent = nextYString;
        this._mTarget = mTarget;
        this._samplingPeriod = samplingPeriod;
        this._fm = fm;
    }

    public int get_id() {
        return _id;
    }

    public void set_id(int _id) {
        this._id = _id;
    }

    public String get_commandSent() {
        return _commandSent;
    }

    public void set_commandSent(String _commandSent) {
        this._commandSent = _commandSent;
    }

    public float get_mTarget() {
        return _mTarget;
    }

    public void set_mTarget(float _mTarget) {
        this._mTarget = _mTarget;
    }

    public float get_fm() {
        return _fm;
    }

    public void setFm(float fm) {
        this._fm = fm;
    }

    public float get_samplingPeriod() {
        return _samplingPeriod;
    }

    public void set_samplingPeriod(float _samplingPeriod) {
        this._samplingPeriod = _samplingPeriod;
    }
}
