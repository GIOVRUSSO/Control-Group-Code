package ie.ucd.smartrideRT;

/**
 * Created by apple on 29/03/2017.
 */

public class BreathingControlData {
    private int _id;
    private String _commandSent;
    private float _gainParameter;
    private float _humanPowerAvg;
    private float _motorPowerAvg;
    private float _mTarget;
    private float _mActualAvg;
    private float _error;
    private float _samplingPeriod;

    public BreathingControlData(String requestToSendToBike, float gainParameter, float humanPowerAvg, float motorPowerAvg,
        float mTarget, float mActualAvg, float error, float samplingPeriod){
        this._commandSent = requestToSendToBike;
        this._gainParameter = gainParameter;
        this._humanPowerAvg = humanPowerAvg;
        this._motorPowerAvg = motorPowerAvg;
        this._mTarget = mTarget;
        this._mActualAvg = mActualAvg;
        this._error = error;
        this._samplingPeriod = samplingPeriod;
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

    public float get_gainParameter() {
        return _gainParameter;
    }

    public void set_gainParameter(float _gainParameter) {
        this._gainParameter = _gainParameter;
    }

    public float get_humanPowerAvg() {
        return _humanPowerAvg;
    }

    public void set_humanPowerAvg(float _humanPowerAvg) {
        this._humanPowerAvg = _humanPowerAvg;
    }

    public float get_motorPowerAvg() {
        return _motorPowerAvg;
    }

    public void set_motorPowerAvg(float _motorPowerAvg) {
        this._motorPowerAvg = _motorPowerAvg;
    }

    public float get_mTarget() {
        return _mTarget;
    }

    public void set_mTarget(float _mTarget) {
        this._mTarget = _mTarget;
    }

    public float get_mActualAvg() {
        return _mActualAvg;
    }

    public void set_mActualAvg(float _mActualAvg) {
        this._mActualAvg = _mActualAvg;
    }

    public float get_error() {
        return _error;
    }

    public void set_error(float _error) {
        this._error = _error;
    }

    public float get_samplingPeriod() {
        return _samplingPeriod;
    }

    public void set_samplingPeriod(float _samplingPeriod) {
        this._samplingPeriod = _samplingPeriod;
    }
}
