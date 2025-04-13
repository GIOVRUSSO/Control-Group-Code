/*
* Class Name: MicrosoftBandData.java
* Corresponding layout: No
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: MicrosoftBandData is the data type that is used to save data to database when a command has been
* sent to the bike in accordance with the java principle of encapsulation. Note that since the MicrosoftBand broadcasts
* data from each sensor individually (not all together) it is important to ensure the correct variable is processed properly
* which is done below by considering the "category" variable which is set in DatabaseService.java when data is received.
* Note that many of the getters and setters are not used since many of the data available from the sensors are not used.
* */

package ie.ucd.smartrideRT;

public class MicrosoftBandData {

    private int _id;
    private float _accelerationX;
    private float _accelerationY;
    private float _accelerationZ;
    private float _angularVelocity;
    private float _distance; //centimeters
    private float _speed;
    private float _heartRate; //beats per minute
    private float _stepCount; //steps since last factory reset
    private float _skinTemp; //degrees Celsius
    private float _UVRadiationIntensity;
    private float _caloriesBurned;
    private float _skinResistance; //galvanic skin resistance
    private float _RRInterval; //interval between the last two continuous heart beats
    private float _lightIntensity;
    private float _airPressure;
    private float _elevation;
    private String _category;

    public MicrosoftBandData() {

    }


    //method handles different cases for data received from DatabaseService - if it is desired to process
    //other data from MicrosoftBand add another category here
    public MicrosoftBandData(String category, float measurement){
        switch(category){
            case "heartRate":
                this._category = "heartRate";
                this._heartRate = measurement;
                break;
            case "calories":
                this._category = "calories";
                this._caloriesBurned = measurement;
                break;
        }

    }


    //this constructor would be for if all data is received at once - it has no practical use since this is not the case
    public MicrosoftBandData(float _accelerationX, float _accelerationY, float _accelerationZ, float _angularVelocity,
                             float _distance, float _speed, float _heartRate, float _stepCount, float _skinTemp,
                             float _UVRadiationIntensity, float _caloriesBurned, float _skinResistance, float _RRInterval,
                             float _lightIntensity, float _airPressure, float _elevation) {
        this._accelerationX = _accelerationX;
        this._accelerationY = _accelerationY;
        this._accelerationZ = _accelerationZ;
        this._angularVelocity = _angularVelocity;
        this._distance = _distance;
        this._speed = _speed;
        this._heartRate = _heartRate;
        this._stepCount = _stepCount;
        this._skinTemp = _skinTemp;
        this._UVRadiationIntensity = _UVRadiationIntensity;
        this._caloriesBurned = _caloriesBurned;
        this._skinResistance = _skinResistance;
        this._RRInterval = _RRInterval;
        this._lightIntensity = _lightIntensity;
        this._airPressure = _airPressure;
        this._elevation = _elevation;
    }

    public String get_category() {
        return _category;
    }

    public void set_category(String _category) {
        this._category = _category;
    }

    public float get_accelerationX() {
        return _accelerationX;
    }

    public void set_accelerationX(float _accelerationX) {
        this._accelerationX = _accelerationX;
    }

    public float get_accelerationY() {
        return _accelerationY;
    }

    public void set_accelerationY(float _accelerationY) {
        this._accelerationY = _accelerationY;
    }

    public float get_accelerationZ() {
        return _accelerationZ;
    }

    public void set_accelerationZ(float _accelerationZ) {
        this._accelerationZ = _accelerationZ;
    }

    public float get_angularVelocity() {
        return _angularVelocity;
    }

    public void set_angularVelocity(float _angularVelocity) {
        this._angularVelocity = _angularVelocity;
    }

    public float get_distance() {
        return _distance;
    }

    public void set_distance(float _distance) {
        this._distance = _distance;
    }

    public float get_speed() {
        return _speed;
    }

    public void set_speed(float _speed) {
        this._speed = _speed;
    }

    public float get_heartRate() {
        return _heartRate;
    }

    public void set_heartRate(float _heartRate) {
        this._heartRate = _heartRate;
    }

    public float get_stepCount() {
        return _stepCount;
    }

    public void set_stepCount(float _stepCount) {
        this._stepCount = _stepCount;
    }

    public float get_skinTemp() {
        return _skinTemp;
    }

    public void set_skinTemp(float _skinTemp) {
        this._skinTemp = _skinTemp;
    }

    public float get_UVRadiationIntensity() {
        return _UVRadiationIntensity;
    }

    public void set_UVRadiationIntensity(float _UVRadiationIntensity) {
        this._UVRadiationIntensity = _UVRadiationIntensity;
    }

    public float get_caloriesBurned() {
        return _caloriesBurned;
    }

    public void set_caloriesBurned(float _caloriesBurned) {
        this._caloriesBurned = _caloriesBurned;
    }

    public float get_skinResistance() {
        return _skinResistance;
    }

    public void set_skinResistance(float _skinResistance) {
        this._skinResistance = _skinResistance;
    }

    public float get_RRInterval() {
        return _RRInterval;
    }

    public void set_RRInterval(float _RRInterval) {
        this._RRInterval = _RRInterval;
    }

    public float get_lightIntensity() {
        return _lightIntensity;
    }

    public void set_lightIntensity(float _lightIntensity) {
        this._lightIntensity = _lightIntensity;
    }

    public float get_airPressure() {
        return _airPressure;
    }

    public void set_airPressure(float _airPressure) {
        this._airPressure = _airPressure;
    }

    public float get_elevation() {
        return _elevation;
    }

    public void set_elevation(float _elevation) {
        this._elevation = _elevation;
    }
}
