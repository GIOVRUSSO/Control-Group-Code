/*
* Class Name: ProactivePollutionControl.java
* Corresponding layout: activity_proactive_pollution_control.xml
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: The only (current) purpose of ProactivePollutionControl is to have an activity
* through which to launch SumoService but it may be used in the future for feedback pollution control
* */

package ie.ucd.smartrideRT;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.Handler;
import android.os.IBinder;
import android.os.Message;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;

import java.lang.Math;

import java.util.ArrayList;
import java.util.List;
import java.util.Timer;
import java.util.TimerTask;


public class ProactivePollutionControl extends AppCompatActivity {
    private static final String tag = "debugging";
    private int timeStartDown = 240000;
    private int timeStartUp = 480000;
    SumoService sumoService;
    BluetoothService bluetoothService;
    DatabaseService databaseService;
    boolean isBound;
    MyDBHandler dbHandler;
    public static final int BREATHING_DATA = 1;
    private Timer controlTimer;
    private Timer dropMTimer;
    private Timer increaseMTimer;
    private Timer cooperativeControlTimer;
    Boolean ping = true;
    //DoControlTask doControlTask;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_proactive_pollution_control);

        //start service for bluetooth connection to use BluetoothService methods
        Intent i = new Intent(this, BluetoothService.class);
        bindService(i, bluetoothServiceConnection, Context.BIND_AUTO_CREATE);

        dbHandler = new MyDBHandler(this, null, null, 1);
    }

    //onClick method to launch SUMO service for simulated pollution environment
    public void startSumoService(View view){
        Log.i(tag, "Launching SUMO service");
        Intent i = new Intent(this, SumoService.class);
        bindService(i, sumoServiceConnection, Context.BIND_AUTO_CREATE);
    }

    //onClick method to close sumoService
    public void endSumoService(View view){
        sumoService.disconnectSumo();
    }

    public void startBreathingControl(View view) {
        float samplingPeriod=1000;
        String message="";
        float mTargetUnpolluted=0.9f;
        float mTargetPolluted = 0.3f;
        float tolerance = 0.01f;
        int transitionTime = 120; //in seconds
        float changeM = (mTargetUnpolluted-mTargetPolluted)/transitionTime;
        boolean startingIter = true;



//        //get user inputted target values for calories to lose and duration
//        final EditText caloriesInputReference = (EditText) findViewById(R.id.caloriesInput);
//        //targetCalories = Float.parseFloat(caloriesInputReference.getText().toString());
//
//        final EditText durationInputReference = (EditText) findViewById(R.id.timeInput);
//        //duration = Float.parseFloat(durationInputReference.getText().toString());

        // Start thread to start syncing data from bike
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);

        Log.i(tag, "about to add mtargetrow");
        //add to database so that there is an initial m value
        dbHandler.addMTargetRow(mTargetUnpolluted);
        Log.i(tag, "mtargetrow added");

        //add initial value to write to bike
        write("0!");

        Log.i(tag, "About to start control task");
        //start closed loop control with feedback
        controlTimer = new Timer();
        controlTimer.schedule(new DoControlTask(mTargetUnpolluted, startingIter, samplingPeriod), 100, (int)samplingPeriod);

        //timers to update the value of m to transition between m is 0.9 and 0.3 as a ramp
        dropMTimer = new Timer();
        dropMTimer.schedule(new DecreaseM(changeM, mTargetPolluted, tolerance), timeStartDown, (int)samplingPeriod);

        increaseMTimer = new Timer();
        increaseMTimer.schedule(new IncreaseM(changeM, mTargetUnpolluted, tolerance), timeStartUp, (int)samplingPeriod);
    }

    public void startCooperativeControl(View view){
        float mTarget = 0.5f;
        Boolean startingIter = true;
        float samplingPeriod=6;

        //start thread to start syncing data from bike
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);

        //add an initial value to database to show control is about to start - this serves as a filer for which values to send to the motor
        dbHandler.addMotorFilterRow(0);


        cooperativeControlTimer = new Timer();
        cooperativeControlTimer.schedule(new DoCooperativeControlTask(mTarget, startingIter, samplingPeriod), 1000, (int)samplingPeriod);

    }

    //Thread to carry out control algorithm
    class DoCooperativeControlTask extends TimerTask{
        private float mTarget;
        boolean startingIteration = true;
        private int maxRequestToBike = 165;
        private int minRequestToBike = 90;
        private float samplingPeriod;
        private float motorEfficiency = 0.8f;
        private float torqueBias = 44.5f;
        private float cranksetEfficiency = 0.9f;
        private float humanPowerFactor = 1.5f;
        private int windowSize = 25;
        private int numMotorAverages = 5;
        private float mActualAvg;
        float[] humanPowerArray;
        float[] motorPowerArray;


        public DoCooperativeControlTask(float mTarget, Boolean startingIter, float samplingPeriod){
            this.mTarget = mTarget;
            this.startingIteration = startingIter;
            this.samplingPeriod = samplingPeriod;
        }

        @Override
        public void run() {
            //Log.i(tag, "in cooperative run method");

            float lastY;
            float nextY;
            float fm;
            int count;

            List<float[]> bikeDataList = new ArrayList<>();

            //Log.i(tag, "about to get bike data list");
            //get most recent data from bike
            bikeDataList = dbHandler.getRecentBikeData(motorEfficiency, torqueBias, cranksetEfficiency, humanPowerFactor, windowSize);
            // Log.i(tag, "bike data list got");
            motorPowerArray = bikeDataList.get(0);
            humanPowerArray = bikeDataList.get(1);

            //Log.i(tag, "motor and human power arrays got");

            //after establishing data is syncing - get the most recent data
            //filtering
            //control starts when cycling starts - get from db when pedals are (consistently) moving?
            //get rid of spurious measurements?


            //calculate PHuman, PMotor, PTotal
            float humanPowerAvg = calculateHumanFloatArrayAverage(humanPowerArray, humanPowerArray.length);
            float motorPowerAvg = calculateFloatArrayAverage(motorPowerArray, numMotorAverages);
//            //calculateFloatArrayAverage(motorPowerArray);
            float pTotal = humanPowerAvg + motorPowerAvg;

            //Log.i(tag, "humanPowerAverage: "+humanPowerAvg);
            //Log.i(tag, "motorPowerAverage "+motorPowerAvg);

            //calculate m - what if both human power and motor power are 0?
            if(humanPowerAvg==0.0f && motorPowerAvg ==0.0f){
                mActualAvg= 0.0f;
            }else
            {
                mActualAvg = humanPowerAvg / pTotal;
            }

            //calculate error by comparing mNow to mTarget
//            float mError = mTarget - mActualAvg;
//            float gamma = 12;


            //for testing purposes lastY corresponds to an initial condition for y and can be set as such
            lastY=165;
            //lastY = dbHandler.getLastRequestToBike();

            //do something to get last value of m
//            lastM = dbHandler.getLatestMActual();


            //define fm, the value of fm determines the value of y that will be settled at
            fm= -75*mActualAvg + 165;

            nextY = lastY - ((samplingPeriod/1000)*lastY)*(lastY - fm);
            int nextYInt = Math.round(nextY);

//
//            //set initial condition
//            if(startingIteration==true){
//                Log.i(tag, "startingIter is true");
//                requestToSendToBike = 110;
//                startingIteration = false;
//            }
          //Log.i(tag, "FM: "+fm);
            //Log.i(tag, "nextYInt: "+nextYInt);

            if(nextYInt>maxRequestToBike){
                nextYInt=maxRequestToBike;
            }else if(nextYInt<minRequestToBike){
                nextYInt=minRequestToBike;
            }
            //Log.i(tag, "nextYInt final: "+nextYInt);


            count = dbHandler.getLatestMotorCount();
            Log.i(tag, "Count: "+count);
            //send to bike
            if(count == 100) {
                count=0;
                handler.obtainMessage(BREATHING_DATA, nextYInt).sendToTarget();
                dbHandler.addMotorFilterRow(count);
            }
            //update the value of count in the database
            dbHandler.addMotorFilterRow(count+1);

            String nextYString = ""+nextYInt;
            //  Log.i(tag, "request to bike: "+nextYString);
            //Log.i(tag, "mActual: "+mActualAvg);


            CooperativeBreathingControlData cooperativeBreathingControlData = new CooperativeBreathingControlData(nextYString,
                    mTarget, fm, samplingPeriod);
            dbHandler.addCooperativeControlData(cooperativeBreathingControlData);
           //\ Log.i(tag, "at end of cooperative run method");

        }

    }

    //write method is used to send message to BluetoothService for it to be sent by bluetooth
    private void write(String message) {
        if (message.length() > 0) {
            byte[] send = message.getBytes();
            bluetoothService.write(send);
        }
    }

    //Update UI based on results of thread here
    Handler handler = new Handler(){
        Integer requestToBikeInt;
        String requestToSendToBike= "";

        @Override
        public void handleMessage(Message msg) {
            switch(msg.what){
                //CALORIES_DATA case is for when an updated assistance level should be sent to the bike
                //based on feedback
                case BREATHING_DATA:
                    requestToBikeInt = (Integer) msg.obj;
                    requestToSendToBike = requestToBikeInt + "!";
                    //Log.i(tag, "Request to send to bike is: " + requestToSendToBike);
                    write(requestToSendToBike);
                    break;
                default:
                    Log.i(tag, "PPC: default case");
                    break;

            }
        }
    };

    //Thread to decrease m as a ramp
    class DecreaseM extends TimerTask{
        private float latestM;
        private float newM;
        private float changeM;
        private float pollutedM = 0.9f;
        private float tolerance;

        public DecreaseM(float changeM, float pollutedM, float tolerance){
            this.changeM = changeM;
            this.pollutedM = pollutedM;
            this.tolerance = tolerance;
        }

        @Override
        public void run() {
            Log.i(tag, "in decrease m");
            latestM = dbHandler.getLatestMTarget();
            if(Math.abs(latestM - pollutedM) <= tolerance){
                Log.i(tag, "in if in decrease m");
                dbHandler.addMTargetRow(pollutedM);
                Log.i(tag, "passed dbHandler add row");
                dropMTimer.cancel();
                Log.i(tag, "dropM Timer cancelled");
            }else {
                //Log.i(tag, "in else");
                newM = latestM - changeM;
                dbHandler.addMTargetRow(newM);
            }
        }


    }

    //Thread to increase m as a ramp
    class IncreaseM extends TimerTask{
        private float latestM;
        private float newM;
        private float changeM;
        private float tolerance;
        private float unpollutedM;

        public IncreaseM(float changeM, float UnpollutedM, float tolerance){
            this.changeM = changeM;
            this.tolerance = tolerance;
            this.unpollutedM = UnpollutedM;
        }

        @Override
        public void run() {
            Log.i(tag, "in increase m");
            latestM = dbHandler.getLatestMTarget();
            if(Math.abs(latestM - unpollutedM) <= tolerance){
                Log.i(tag, "in if in increase m");
                dbHandler.addMTargetRow(unpollutedM);
                Log.i(tag, "passed dbHandler add row");
                increaseMTimer.cancel();
                Log.i(tag, "increaseM Timer cancelled");
            }else {
                newM = latestM + changeM;
                dbHandler.addMTargetRow(newM);
            }
        }


    }


    //Thread to carry out control algorithm
    class DoControlTask extends TimerTask{
        private float motorEfficiency = 0.8f;
        private float torqueBias = 44.5f;
        private float cranksetEfficiency = 0.9f;
        private float humanPowerFactor = 1.5f;
        private int windowSize = 25;
        private int numMotorAverages = 5;
        private float mTarget;
        private float requestToSendToBike;
        private float mActualAvg;
//        private long duration;
//        private float timePassed;
//        private long endTime;
//        private long startTime;
//        private long futureTime;
//        private long samplingPeriod;
        private int maxRequestToBike = 165;
        private int minRequestToBike = 90;
        boolean startingIteration = true;
        private float mInit;
        float[] humanPowerArray;
        float[] motorPowerArray;
        float samplingPeriod;


        public DoControlTask(float mTarget, Boolean startingIter, float samplingPeriod){
            this.samplingPeriod = samplingPeriod;
            this.mTarget = mTarget;
            this.startingIteration = startingIter;
        }

        @Override
        public void run() {
           // Log.i(tag, "in control task");
            mTarget = dbHandler.getLatestMTarget();
            Log.i(tag, "mTarget: "+mTarget);

//            startTime = System.currentTimeMillis();
//            endTime = System.currentTimeMillis() + duration;


            List<float[]> bikeDataList = new ArrayList<>();

            //Log.i(tag, "about to get bike data list");
            //get most recent data from bike
            bikeDataList = dbHandler.getRecentBikeData(motorEfficiency, torqueBias, cranksetEfficiency, humanPowerFactor, windowSize);
           // Log.i(tag, "bike data list got");
            motorPowerArray = bikeDataList.get(0);
            humanPowerArray = bikeDataList.get(1);

            //Log.i(tag, "motor and human power arrays got");

            //after establishing data is syncing - get the most recent data
            //filtering
            //control starts when cycling starts - get from db when pedals are (consistently) moving?
            //get rid of spurious measurements?


            //calculate PHuman, PMotor, PTotal
            float humanPowerAvg = calculateHumanFloatArrayAverage(humanPowerArray, humanPowerArray.length);
            float motorPowerAvg = calculateFloatArrayAverage(motorPowerArray, numMotorAverages);
            //calculateFloatArrayAverage(motorPowerArray);
            float pTotal = humanPowerAvg + motorPowerAvg;

            //Log.i(tag, "humanPowerAverage: "+humanPowerAvg);
            //Log.i(tag, "motorPowerAverage "+motorPowerAvg);

            //calculate m - what if both human power and motor power are 0?
            if(humanPowerAvg==0.0f && motorPowerAvg ==0.0f){
                mActualAvg= 0.0f;
            }else
            {
                mActualAvg = humanPowerAvg / pTotal;
            }

            //calculate error by comparing mNow to mTarget
            float mError = mTarget - mActualAvg;
            float gamma = 12;

            //update request to send to bike - must get previous value from database
            requestToSendToBike = dbHandler.getLastRequestToBike();

            //set initial condition
            if(startingIteration==true){
                Log.i(tag, "startingIter is true");
                requestToSendToBike = 110;
                startingIteration = false;
            }

            //update request to send to bike
            int requestToSendToBikeInt = Math.round(requestToSendToBike - gamma * mError);

            if(requestToSendToBikeInt>maxRequestToBike){
                requestToSendToBikeInt=maxRequestToBike;
            }else if(requestToSendToBikeInt<minRequestToBike){
                requestToSendToBikeInt=minRequestToBike;
            }

            //send to bike
            handler.obtainMessage(BREATHING_DATA, requestToSendToBikeInt).sendToTarget();


            String bikeRequestString = ""+requestToSendToBikeInt;
          //  Log.i(tag, "request to bike: "+bikeRequestString);
            Log.i(tag, "mActual: "+mActualAvg);

            //save data in database
            BreathingControlData breathingControlData = new BreathingControlData(bikeRequestString, gamma,
                    humanPowerAvg, motorPowerAvg, mTarget, mActualAvg, mError, samplingPeriod);
            dbHandler.addBreathingControlData(breathingControlData);

        }

    }

    //calculate average values of data got from database that is saved in a float[]
//    public float calculateFloatArrayAverage(float[] floatArray){
//        float average;
//        float sum=0;
//
//        for(int i=0; i < floatArray.length ; i++){
//            sum = sum + floatArray[i];
//        }
//        //calculate average value
//        average = sum / floatArray.length;
//        return average;
//    }

    //calculate power average
    public float calculateFloatArrayAverage(float[] floatArray, int numAverages){
        float average;
        float sum=0;

        for(int i=0; i < numAverages; i++){
            sum = sum + floatArray[i];
           // Log.i(tag, "motorPower:" + floatArray[i]);
        }

        //calculate average value
        average = sum / numAverages;
        return average;
    }

    //calculate power average
    public float calculateHumanFloatArrayAverage(float[] floatArray, int numAverages){
        float average;
        float sum=0;

        for(int i=0; i < numAverages; i++){
            sum = sum + floatArray[i];
            //Log.i(tag, "humanPower:" + floatArray[i]);
        }

        //calculate average value
        average = sum / numAverages;
        return average;
    }

    //sumoServiceConnection is needed to allow SUMO processing to work in the background
    private ServiceConnection sumoServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            SumoService.SumoServiceMyLocalBinder binder  = (SumoService.SumoServiceMyLocalBinder) service;
            sumoService = binder.getService();
            isBound = true;
        }


        @Override
        public void onServiceDisconnected(ComponentName name){
            isBound = false;
        }

    };

    //bluetoothServiceConnection is needed to use BluetoothService methods
    private ServiceConnection bluetoothServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            BluetoothService.BluetoothMyLocalBinder binder  = (BluetoothService.BluetoothMyLocalBinder) service;
            bluetoothService = binder.getService();
            isBound = true;
        }


        @Override
        public void onServiceDisconnected(ComponentName name){
            isBound = false;
        }

    };

    //databaseServiceConnection is needed to use DatabaseService methods
    private ServiceConnection databaseServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            DatabaseService.DatabaseMyLocalBinder binder  = (DatabaseService.DatabaseMyLocalBinder) service;
            databaseService = binder.getService();
        }

        @Override
        public void onServiceDisconnected(ComponentName name){
            
        }
    };

}
