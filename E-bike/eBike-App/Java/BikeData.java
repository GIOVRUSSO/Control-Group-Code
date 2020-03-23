/*
* Class Name: BikeData.java
* Corresponding layout: No
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
* Description: BikeData is the data type that covers data received from the cycle analyst using getters
* and setters in accordance with the java principle of encapsulation, note that many of the setters are
* not currently used for anything, this class should be used every time it is desired to save data from the
* cycle anlalyst / data from the bike
* */

package ie.ucd.smartrideRT;

public class BikeData {

    private int _id;
    private String _productname;
    private float _batteryEnergy;
    private float _voltage;
    private float _current;
    private float _speed;
    private float _distance;
    private float _temperature;
    private float _RPM;
    private float _humanPower;
    private float _torque;
    private float _throttleIn;
    private float _throttleOut;
    private float _acceleration;
    //note that _flag is defined as a string here because sometimes it can be "1W" (etc.) if there
    //is a warning about a datapoint received from cycle analyst
    private String _flag;



    //constructor for bikeData
    public BikeData(float batteryEnergy, float voltage, float current, float speed, float distance,
                    float temperature, float RPM, float humanPower, float torque, float throttleIn, float throttleOut,
                    float acceleration, String flag){
        this._batteryEnergy = batteryEnergy;
        this._voltage = voltage;
        this._current = current;
        this._speed = speed;
        this._distance = distance;
        this._temperature = temperature;
        this._RPM = RPM;
        this._humanPower = humanPower;
        this._torque = torque;
        this._throttleIn = throttleIn;
        this._throttleOut = throttleOut;
        this._acceleration = acceleration;
        this._flag = flag;
    }

    public void set_id(int _id) {
        this._id = _id;
    }

    public void set_productname(String _productname) {
        this._productname = _productname;
    }

    public void set_batteryEnergy(float _batteryEnergy) {
        this._batteryEnergy = _batteryEnergy;
    }

    public void set_voltage(float _voltage) {
        this._voltage = _voltage;
    }

    public void set_current(float _current) {
        this._current = _current;
    }

    public void set_speed(float _speed) {
        this._speed = _speed;
    }

    public void set_distance(float _distance) {
        this._distance = _distance;
    }

    public void set_temperature(float _temperature) {
        this._temperature = _temperature;
    }

    public void set_RPM(float _RPM) {
        this._RPM = _RPM;
    }

    public void set_humanPower(float _humanPower) {
        this._humanPower = _humanPower;
    }

    public void set_torque(float _torque) {
        this._torque = _torque;
    }

    public void set_throttleIn(float _throttleIn) {
        this._throttleIn = _throttleIn;
    }

    public void set_throttleOut(float _throttleOut) {
        this._throttleOut = _throttleOut;
    }

    public void set_acceleration(float _acceleration) {
        this._acceleration = _acceleration;
    }

    public void set_flag(String _flag) {
        this._flag = _flag;
    }

    public int get_id() {
        return _id;
    }

    public String get_productname() {
        return _productname;
    }

    public float get_batteryEnergy() {
        return _batteryEnergy;
    }

    public float get_voltage() {
        return _voltage;
    }

    public float get_current() {
        return _current;
    }

    public float get_speed() {
        return _speed;
    }

    public float get_distance() {
        return _distance;
    }

    public float get_temperature() {
        return _temperature;
    }

    public float get_RPM() {
        return _RPM;
    }

    public float get_humanPower() {
        return _humanPower;
    }

    public float get_torque() {
        return _torque;
    }

    public float get_throttleIn() {
        return _throttleIn;
    }

    public float get_throttleOut() {
        return _throttleOut;
    }

    public float get_acceleration() {
        return _acceleration;
    }

    public String get_flag() {
        return _flag;
    }
}
