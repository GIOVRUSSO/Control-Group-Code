/*
* Class Name: CaloriesControlData.java
* Corresponding layout: No
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: CaloriesControlData is the data type that is used to save data used in calories control application
* (errors, targets, parameters etc.) using getters and setters in accordance with the java principle of encapsulation
* */

package ie.ucd.smartrideRT;

public class CaloriesControlData {
    private int _id;
    private float _commandSent;
    private float _actualCaloriesBurnedRate;
    private float _targetCaloriesRate;
    private float _errorCalories;
    private float _gainParameter;

    public CaloriesControlData(float commandSent){
        this._commandSent = commandSent;
    }

    public CaloriesControlData(float requestToSendToBike, float actualCaloriesBurnedRate, float targetCaloriesRate,
    float errorCalories, float gainParameter){
        this._commandSent = requestToSendToBike;
        this._actualCaloriesBurnedRate = actualCaloriesBurnedRate;
        this._targetCaloriesRate = targetCaloriesRate;
        this._errorCalories = errorCalories;
        this._gainParameter = gainParameter;

    }

    public float get_commandSent() {
        return _commandSent;
    }

    public void set_commandSent(float _commandSent) {
        this._commandSent = _commandSent;
    }

    public float get_actualCaloriesBurnedRate() {
        return _actualCaloriesBurnedRate;
    }

    public void set_actualCaloriesBurnedRate(float _actualCaloriesBurnedRate) {
        this._actualCaloriesBurnedRate = _actualCaloriesBurnedRate;
    }

    public float get_targetCaloriesRate() {
        return _targetCaloriesRate;
    }

    public void set_targetCaloriesRate(float _targetCaloriesRate) {
        this._targetCaloriesRate = _targetCaloriesRate;
    }

    public float get_errorCalories() {
        return _errorCalories;
    }

    public void set_errorCalories(float _errorCalories) {
        this._errorCalories = _errorCalories;
    }

    public float get_gainParameter() {
        return _gainParameter;
    }

    public void set_gainParameter(float _gainParameter) {
        this._gainParameter = _gainParameter;
    }
}
