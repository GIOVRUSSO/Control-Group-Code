/*
* Class Name: MicrosoftBand.java
* Corresponding layout: activity_microsoft_band.xml
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: MicrosoftBand is used to:
* (i) connect to Microsoft Band - consent is possibly required for some data (heart rate)
* (ii) start syncing data to database, both data from cycle analyst and microsoft band is synced if available
* (iii) closed loop feedback to enable a user to set a target rate of calories that they wish to lose
* and bike varies level of assistance that it gives to keep user on track
* */


package ie.ucd.smartrideRT;

import android.app.Activity;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.ServiceConnection;
import android.os.AsyncTask;
import android.os.IBinder;
import android.os.Message;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.EditText;
import android.os.Handler;

import com.microsoft.band.BandClient;
import com.microsoft.band.BandClientManager;
import com.microsoft.band.BandException;
import com.microsoft.band.BandInfo;
import com.microsoft.band.ConnectionState;
import com.microsoft.band.sensors.HeartRateConsentListener;

import java.lang.ref.WeakReference;
import java.util.Date;
import java.util.Map;
import java.util.TreeMap;

public class MicrosoftBand extends AppCompatActivity {

    BluetoothService bluetoothService;
    boolean isBound=false;
    private static final String tag = "debugging";
    DatabaseService databaseService;
    MyDBHandler dbHandler;
    public static final int CALORIES_DATA = 1;
    private BandClient client = null;
    private Button btnConsent;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(tag, "In onCreate for calories");

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_microsoft_band);


        //start service for bluetooth connection to use BluetoothService methods
        Intent i = new Intent(this, BluetoothService.class);
        bindService(i, bluetoothServiceConnection, Context.BIND_AUTO_CREATE);


        dbHandler = new MyDBHandler(this, null, null, 1);
        btnConsent = (Button) findViewById(R.id.btnConsent);

    }

    public void startDataSync(View view) {

        /*(ii) Code to start data syncing to database*/
        //Start service for database to sync data
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);



        /*(iii) Code to get user input of target number of calories to lose and cycling duration*/
        String message="";
        float targetCalories;
        float duration;

        //get user inputted target values for calories to lose and duration
        final EditText caloriesInputReference = (EditText) findViewById(R.id.caloriesInput);
        targetCalories = Float.parseFloat(caloriesInputReference.getText().toString());
        //set manually below for testing for now
        targetCalories = 100;

        final EditText durationInputReference = (EditText) findViewById(R.id.timeInput);
        duration = Float.parseFloat(durationInputReference.getText().toString());
        //set manually below for testing for now
        duration = 300000;

        //uncomment this line when it is desired to do closed loop control
        //startCaloriesControl();


    }

    //user has clicked button to give their consent for microsoft band data to sync
    public void btnConsentClicked(View view){
        final WeakReference<Activity> reference = new WeakReference<Activity>(this);
        new HeartRateConsentTask().execute(reference);
    }



    //onClick button indicating user wishes to send the data they have inputted by bluetooth to bike
    public void manualCommand(View view){
        final EditText enterCommand = (EditText) findViewById(R.id.enterCommand);
        String bikeCommand = enterCommand.getText().toString();
        write(bikeCommand);
    }


    //onClick method to go back to MainActivity
    public void onBackClick(View view){
        Intent i = new Intent(this, MainActivity.class);
        startActivity(i);
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
                case CALORIES_DATA:
                    requestToBikeInt = (Integer) msg.obj;
                    requestToSendToBike = requestToBikeInt + "!";
                    //Log.i(tag, "Request to send to bike is: " + requestToSendToBike);
                    write(requestToSendToBike);
                    break;
                default:
                    Log.i(tag, "MB: default case");
                    break;

            }
        }
    };


    //Need to get user consent for some Microsoft band data e.g. heart rate data
    private class HeartRateConsentTask extends AsyncTask<WeakReference<Activity>, Void, Void> {

        @Override
        protected Void doInBackground(WeakReference<Activity>... params){

            try {
                if(getConnectedHeartBandClient()){
                    if(params[0].get() != null){
                        client.getSensorManager().requestHeartRateConsent(params[0].get(), new HeartRateConsentListener() {
                            @Override
                            public void userAccepted(boolean b) {

                            }
                        });
                    }
                }else{
                    // appendToUI("Band isn't connected, please make sure bluetooth is on and the band is in range");
                }

            } catch (Exception e) {
                e.printStackTrace();
            }
            return null;
        }
    }



    //Get connection to Microsoft band
    private Boolean getConnectedHeartBandClient() throws InterruptedException, BandException {
        Log.i(tag, "in get connected band client heart");

        if (client ==null){
            //Find paired bands
            BandInfo[] pairedBands = BandClientManager.getInstance().getPairedBands();

            if (pairedBands.length == 0){
                return false;
            }

            client = BandClientManager.getInstance().create(getBaseContext(), pairedBands[0]);
            Log.i(tag, "client found heart " + client);
            return true;

        } else if(ConnectionState.CONNECTED == client.getConnectionState()){
//            BandInfo[] pairedBands = BandClientManager.getInstance().getPairedBands();
//            client = BandClientManager.getInstance().create(getBaseContext(), pairedBands[0]);
            Log.i(tag, "in else if heart");
            return true;
        }

        return ConnectionState.CONNECTED == client.connect().await();
    }


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

//to add a message to UI maybe use this method
//    private void appendToUI(final String string) {
//        this.runOnUiThread(new Runnable() {
//            @Override
//            public void run() {
//                TEXTVIEW.setText(string);
//            }
//        });
//
//    }











/*
*----------------------------------------------------------------
* Code below here is code for closed loop control with feedback which has not been
* fully tested or implemented but is useful
* ----------------------------------------------------------------
* */

    public void startCaloriesControl() {
        String message="";
        long targetCalories;
        long duration;

        //get user inputted target values for calories to lose and duration
        final EditText caloriesInputReference = (EditText) findViewById(R.id.caloriesInput);
        //targetCalories = Float.parseFloat(caloriesInputReference.getText().toString());
        targetCalories = 100;

        final EditText durationInputReference = (EditText) findViewById(R.id.timeInput);
        //duration = Float.parseFloat(durationInputReference.getText().toString());
        duration = 300000;

        //start service for database
        Log.i(tag, "Calories control button clicked");


        //Is this intent needed?? To sync data -> yes
        Intent j = new Intent(this, DatabaseService.class);
        bindService(j, databaseServiceConnection, Context.BIND_AUTO_CREATE);

        //Start to sync data
        //databaseService.registerCaloriesEventListener();

        //UI - "please enter your height and weight"

        //Start thread to analyse data - final command to bike will be sent in handler from this thread
        Log.i(tag, "about to start caloriescalc thread");


//        Leave these lines commented for now when we are carrying out modelling - uncomment for control
//        CaloriesCalcThread caloriesCalcThread = new CaloriesCalcThread(targetCalories, duration);
//        caloriesCalcThread.start();

    }


    //Thread to carry out control algorithm
    private class CaloriesCalcThread extends Thread{
        private long targetCalories;
        private long duration;
        private int requestToSendToBike;
        private float caloriesBurnedNow;
        private float timePassed;
        private long endTime;
        private long startTime;
        private long futureTime;
        private long samplingPeriod;
        boolean startingIteration = true;
        private float startingCalories;

        public CaloriesCalcThread(long targetCalories, long duration){
            //specify units for calories and time later - maybe give set range of options?
            //for now - assume calories is kcal, and time is milliseconds, more naturally should be minutes
            this.targetCalories = targetCalories;
            this.duration = duration;
        }

        @Override
        public void run() {
            Log.i(tag, "in calories calc thread");
            //duration is set here for testing purposes but in practise the duration value
            //should be set based on user input in the MicrosoftBand.java activity
            duration = 300000;

            startTime = System.currentTimeMillis();
            endTime = System.currentTimeMillis()+duration;


            //set sampling period - longer sampling period = bigger changes, shorter sampling periods = smaller changes?
            //smaller changes - affect on filtering?
            //arbitrarily set to 5 seconds for now
            samplingPeriod = 5000;

            //inital target calories rate should be this
            float targetCaloriesRate = targetCalories / duration;

            //initial request  starting assistance to give to bike - based on calculation of calories?
            float lastRequestSentToBike = 0;
            int count = 0;

            //we initialise futureTime to be the time now so that for the logical test below the
            //first iteration will always pass
            futureTime = System.currentTimeMillis();

            //initial request to send to bike should be some function of target calories and duration - for now it is fixed
            requestToSendToBike = 100;

            while (endTime - System.currentTimeMillis()>0) {

                if(futureTime < System.currentTimeMillis()) {
                    futureTime = System.currentTimeMillis() + samplingPeriod;

                    //Later - think about desired curve for target calories - include some start up and cool down time?
                    //targetCaloriesRate should account for accumulated error (integral)

                    Map<Date, Float> caloriesData = new TreeMap<Date, Float>();
                    Date time;
                    float calories = 0f;

                    //caloriesData is a treemap with the recent readings of calories burned
                    caloriesData = dbHandler.getRecentCaloriesData();


                    //what about if time user has set, isn't divisive by sampling rate? should there be one large loop?


                    //after establishing data is syncing - get the most recent data
                    //make sure data is syncing
                    for (Map.Entry<Date, Float> entry : caloriesData.entrySet()) {
                        //get time values
                        Date key = entry.getKey();
                        //get calories values
                        Float value = entry.getValue();

                        //if we need to use time for anything
                        //time = key;

                        //get the most recent value of calories burned
                        caloriesBurnedNow = value;

                        //get starting calories - to be completed (make sure data is syncing - not old data)
                        if (startingIteration == true) {
                            startingCalories = caloriesBurnedNow;

                            //we only want to do this on the first iteration of the loop
                            startingIteration = false;
                        }

                    }


                    //filtering
                    //control starts when cycling starts - get from db when pedals are (consistently) moving?
                    //get rid of spurious measurements?
                    //Think about which measured value of calories do we use? last one or average? (in case of errors..)
                    //maybe do some translation between calories burned at rest and calories burned due to exercise


                    //Compare calories burned (total) now to the start time to get calories burned
                    float timePassed = System.currentTimeMillis() - startTime;
                    float actualCaloriesBurnedRate = (caloriesBurnedNow-startingCalories) / (timePassed/1000);

                    //target calories rate should update on each iteraction to account for accumulated error
                    float timeRemaining = endTime - System.currentTimeMillis();
                    targetCaloriesRate = (targetCalories - (caloriesBurnedNow-startingCalories)) / (timeRemaining/1000);

                    //Compare calories burned now to target rate of calories
                    //Store error somewhere - db? can it be recreated after?
                    //should we also store targetCaloriesRate and actualCaloriesBurnedRate? yes probably
                    float errorCalories = targetCaloriesRate - actualCaloriesBurnedRate;
                    Log.i(tag, "Target calories rate is: "+targetCaloriesRate);
                    Log.i(tag, "Actual calories rate is: "+actualCaloriesBurnedRate);


                    //Translate calories burned now to electrical power output
                    //get last request sent to bike from database
                    //maybe add try catch here
                    //lastRequestSentToBike = dbHandler.getLastRequestToBike();

                    //Add the ability to set the gain parameter by user? (for testing) Store gain parameter somewhere - db?
                    //have a positive and negative gain parameter?
                    float gainParameter = 5;


                    if (errorCalories > 0) {
                        //if error is positive - we want to send a request to the bike for a smaller amount than
                        //was previously sent (the person should work harder)
                        Log.i(tag, "We are behind schedule with error: "+errorCalories);
                        requestToSendToBike = Math.round(requestToSendToBike + gainParameter * errorCalories);
                        // Log.i(tag,"Request to send to bike would be: "+requestToSendToBike);

                    } else if (errorCalories < 0) {
                        Log.i(tag, "We are ahead of schedule with error: "+errorCalories);
                        //if the error is negative - we want to send a request to the bike for a larger amount
                        //than was previously sent (the person should work less hard) ~~ maybe?? or just let
                        //them stay on track...

                        requestToSendToBike = Math.round(requestToSendToBike - gainParameter * errorCalories);

                    } else {
                        //else they are on track
                        //is this the only other case? exceptions?
                        requestToSendToBike = Math.round(requestToSendToBike);
                        Log.i(tag, "We are on schedule with zero error");
                    }


                    //Add some error checking here to ensure that silly values aren't sent to bike
                    //N.B. make sure to respect the limitations of the bike - dont send a request for less than
                    //~90 or greater than ~180, motor will not be able to handle larger values


                    if (true) {
                        //update UI / UX
                        //when request to bike changes - message should pop up on UI
                        handler.obtainMessage(CALORIES_DATA, requestToSendToBike).sendToTarget();
                    }


                    //what if user wants to increase total calories to lose / duration during the control?
                    //print to screen?
                    CaloriesControlData caloriesControlData = new CaloriesControlData(requestToSendToBike, actualCaloriesBurnedRate, targetCaloriesRate,
                            errorCalories, gainParameter);
                    dbHandler.addCaloriesControlData(caloriesControlData);

//                    count = 1;
                } else{

                }

            }

            //do something after duration is over?
            //"your workout has ended" - you burned this many calories? / were on track?etc
            //show countdown on UI

        }

    }



}
