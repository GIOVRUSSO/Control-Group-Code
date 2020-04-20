/*
* Class Name: DatabaseService.java
* Corresponding layout: No
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: DatabaseService processes data received from the bike by Bluetooth and passes it to
* MyDBHandler to save it to the database. For convenience methods relating to getting data from the
* MicrosoftBand (heart rate, calories burned) are also implemented here. If desired these could be moved
* into their own service and transferred directly to the MyDBHandler
* */

/*
* This class (line 89 - 214) has been modified by Jingyi Hu - jingyi.hu@ucdconnect.ie in 2019.
* An Upload thread has been added for supporting upload the database file onto the Dropbox
* Excaption might occurs if the CA3 settings changed
*/
package ie.ucd.smartrideRT;

import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.AsyncTask;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;

import com.microsoft.band.BandClient;
import com.microsoft.band.BandClientManager;
import com.microsoft.band.BandException;
import com.microsoft.band.BandInfo;
import com.microsoft.band.ConnectionState;
import com.microsoft.band.InvalidBandVersionException;
import com.microsoft.band.UserConsent;
import com.microsoft.band.sensors.BandCaloriesEvent;
import com.microsoft.band.sensors.BandCaloriesEventListener;
import com.microsoft.band.sensors.BandHeartRateEvent;
import com.microsoft.band.sensors.BandHeartRateEventListener;

import java.util.Calendar;

public class DatabaseService extends Service {

    private final IBinder databaseBinder = new DatabaseMyLocalBinder();
    IntentFilter filter;
    private static final String tag = "debuggingDBS";
    MyDBHandler dbHandler;
    private BandClient client = null;

    @Override
    public IBinder onBind(Intent intent) {
        Log.d(tag,"Call reg db");
        registerDatabaseReceiver();
        dbHandler = new MyDBHandler(this, null, null, 1);

        //methods to register listeners for
//        startMBandHeartDataSync();
//        startMBandCaloriesDataSync();

        return databaseBinder;
    }

    public void startMBandCaloriesDataSync(){
        new CaloriesSubscriptionTask().execute();
        new CaloriesSubscriptionTask().execute();
        Log.i(tag, "DS: cals subscription task should be registered");
    }

    public void startMBandHeartDataSync(){
        new HeartRateSubscriptionTask().execute();
        new HeartRateSubscriptionTask().execute();
        Log.i(tag, "DS: heart rate subscription task should be registered");
    }


    public class DatabaseMyLocalBinder extends Binder {
        DatabaseService getService(){
            return DatabaseService.this;
        }

    }

    private void registerDatabaseReceiver() {
        Log.d(tag, "db reg");
        filter = new IntentFilter("ie.ucd.smartrideRT.database");
        registerReceiver(MyReceiver, filter);

    }
    
    int minute = -1;
    private final BroadcastReceiver MyReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {
            Log.d(tag, "db onreceive");
            String bikeDataReceived = intent.getStringExtra("database");

            //comment this log line for now but it is very useful for checking data is being received
            Log.i(tag, "DS: Bike data: " + bikeDataReceived);

            // Must determine which table the data is intended for
            String[] splitBikeDataReceived = bikeDataReceived.split(" ");

            Calendar c = Calendar.getInstance();

            int currentMinute = c.get(Calendar.MINUTE);
            if (minute == -1)
                minute = currentMinute;
            if (currentMinute != minute) {
                minute = currentMinute;
                UploadBikeDataThread uploadBikeDaraThread = new UploadBikeDataThread();
                uploadBikeDaraThread.start();
                }

            else{
                if(splitBikeDataReceived[0].equals("Signal")){
                ProcessTrafficLightDataThread processTrafficLightDataThread = new ProcessTrafficLightDataThread(bikeDataReceived);
                processTrafficLightDataThread.start();
                }
                else {
                        //If it is desired to save the frequency with which data is saved from cycle analyst this can be
                        //implemented here with a count variable as is done in mHeartRateEventListener below
                        ProcessBikeDataThread processBikeDataThread = new ProcessBikeDataThread(bikeDataReceived);
                        processBikeDataThread.start();
                    }
                }
        }
    };


    private class UploadBikeDataThread extends Thread{
        public void run(){
            try {
                Upload.upload("EBike");
                Log.d(tag,"Success Call Upload here"+ minute);
            } catch (Exception e) {
                System.out.println("Call upload exception");
                return;
            }
        }
    }

    //Save data from the cycle analyst in a separate thread
    private class ProcessBikeDataThread extends Thread{
        private final String bikeDataString;

        private final Object o = new Object();

        public ProcessBikeDataThread(String string){
            this.bikeDataString = string;
            // this.count = count;
        }

        public void run(){
            String[] strings;
            String tempFlag="";
            Float[] floats = new Float[13];

            strings = bikeDataString.split("\\t");


            //Cycle analyst outputs 13 variables
            for(int i=0;i<13;i++){
                //The last variable "flag" is sometimes a string if there is a warning e.g. it could be "1W"
                //as such must be processed as a string, see cycle analyst guide
                if(i==12){
                    try{
                        tempFlag = strings[i];
                    } catch (ArrayIndexOutOfBoundsException e){
                        tempFlag = "1M";
                    }
                }
                else{
                    try{
                        floats[i] = Float.valueOf(strings[i]);
                    } catch (NumberFormatException e){
                        e.printStackTrace();
                        return;
                    }

                }
            }

            float batteryEnergy = floats[0];
            float voltage = floats[1];
            float current = floats[2];
            float speed = floats[3];
            float distance = floats[4];
            float temperature = floats[5];
            float RPM = floats[6];
            float humanPower = floats[7];
            float torque = floats[8];
            float throttleIn = floats[9];
            float throttleOut = floats[10];
            float acceleration = floats[11];
            String flag = tempFlag;

            BikeData bikedata = new BikeData(batteryEnergy, voltage, current, speed, distance,temperature,
                    RPM, humanPower, torque, throttleIn, throttleOut, acceleration, flag);

            /*
            synchronized (o) {
                dbHandler.addBikeDataRow(bikedata);
            }
*/
            try{
            dbHandler.addBikeDataRow(bikedata);
            Log.d(tag,"ROWWWWWW");
            } catch(Exception e){
                Log.d(tag,"SQLiteConnectionPool Error");
                return;
            }
        }
    }

    //Make checks and if all is OK register listener to listen for data from Microsoft Band
    private class HeartRateSubscriptionTask extends AsyncTask<Void, Void, Void> {
        @Override
        protected Void doInBackground(Void... params) {
            try {
                if(getConnectedHeartBandClient()){
                    Log.i(tag, "In if connected band client heart");
                    if(client.getSensorManager().getCurrentHeartRateConsent() == UserConsent.GRANTED){
                        Log.i(tag, "In heart rate consent granted");
                        client.getSensorManager().registerHeartRateEventListener(mHeartRateEventListener);
                        Log.i(tag, "Heart rate event listener should be registered");
                    } else{
                   //     appendToUI("You have not given consent to this application to access heart rate data yet." +
                     //           "Please press the Heart rate consent button. \n");
                    }
                } else{
                //    appendToUI("Band isn't connected, please make sure bluetooth is on and the band is in range");
                }
            } catch (BandException e) {
                String exceptionMessage = "";
                switch (e.getErrorType()) {
                    case UNSUPPORTED_SDK_VERSION_ERROR:
                        exceptionMessage = "Microsoft Health BandService doesn't support your SDK Version. Please update to latest SDK.\n";
                        break;
                    case SERVICE_ERROR:
                        exceptionMessage = "Microsoft Health BandService is not available. Please make sure Microsoft Health is installed and that you have the correct permissions.\n";
                        break;
                    default:
                        exceptionMessage = "Unknown error occured: " + e.getMessage() + "\n";
                        break;
                }
             //   appendToUI(exceptionMessage);
            }catch (Exception e) {
            //    appendToUI(e.getMessage());

            }
            return null;
        }
    }


    //Start connection for listening for calories data from Microsoft Band
    public class CaloriesSubscriptionTask extends AsyncTask<Void, Void, Void> {
        boolean connection = false;
        @Override
        protected Void doInBackground(Void... params) {
            Log.i(tag, "in doInBackground");
            try {
                //it seems we have to make two requests to getConnectedCaloriesBandClient() for it to register connection - a Microsoft bug?
                getConnectedCaloriesBandClient();
                connection = getConnectedCaloriesBandClient();
                if(connection){
                        Log.i(tag, "about to register calories event listener");
                        client.getSensorManager().registerCaloriesEventListener(mCaloriesEventListener);
                        Log.i(tag, "listener should be registered");
                } else{
                    //    appendToUI("Band isn't connected, please make sure bluetooth is on and the band is in range");
                }
            } catch (BandException e) {
                Log.i(tag, "exception");
                String exceptionMessage = "";
                switch (e.getErrorType()) {
                    case UNSUPPORTED_SDK_VERSION_ERROR:
                        exceptionMessage = "Microsoft Health BandService doesn't support your SDK Version. Please update to latest SDK.\n";
                        Log.i(tag, exceptionMessage);
                        break;
                    case SERVICE_ERROR:
                        exceptionMessage = "Microsoft Health BandService is not available. Please make sure Microsoft Health is installed and that you have the correct permissions.\n";
                        Log.i(tag, exceptionMessage);
                        break;
                    default:
                        exceptionMessage = "Unknown error occured: " + e.getMessage() + "\n";
                        Log.i(tag, exceptionMessage);
                        break;
                }
                //   appendToUI(exceptionMessage);
            }catch (Exception e) {
                Log.i(tag, "another exception");
                //    appendToUI(e.getMessage());

            }
            return null;
        }
    }


    //create Microsoft band heart rate event listener to listen for data broadcasts from Band
    private BandHeartRateEventListener mHeartRateEventListener = new BandHeartRateEventListener() {
        float heartRate;
        String category = "heartRate";
        private int count =0;

        @Override
        public void onBandHeartRateChanged(BandHeartRateEvent bandHeartRateEvent) {
           Log.i(tag, "in heart rate event change");
            if (bandHeartRateEvent !=null){
                heartRate = bandHeartRateEvent.getHeartRate();



                //To ensure this method only gets called e.g. once every 1 second
                count+=1;
                //Log.i(tag, "Count in listener is " + count);
                if(count==5){
                    count = 0;
                }

                SaveMBandDataThread saveMBandData = new SaveMBandDataThread(category, heartRate, count);
                saveMBandData.start();

            }

        }
    };


    //Get connection to Microsoft band
    private Boolean getConnectedHeartBandClient() throws InterruptedException, BandException {
        Log.i(tag, "in get connected band client heart");

        if (client ==null){
            //Find paired bands
            BandInfo[] pairedBands = BandClientManager.getInstance().getPairedBands();

            if (pairedBands.length == 0){
                return false;
            }

            //need to set client if there are devices
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


    //Get connection to Microsoft band
    private Boolean getConnectedCaloriesBandClient() throws InterruptedException, BandException {
        Log.i(tag, "in get connected band client calories");

        if (client ==null){
            //Find paired bands
            BandInfo[] pairedBands = BandClientManager.getInstance().getPairedBands();

            if (pairedBands.length == 0){
                //appendToUI("No paired bands");
                return false;
            }

            //need to set client if there are devices
            // client = BandClientManager.getInstance().create(getBaseContext(), pairedBands[0]);
            return true;

        } else if(ConnectionState.CONNECTED == client.getConnectionState()){
           // BandInfo[] pairedBands = BandClientManager.getInstance().getPairedBands();
           // client = BandClientManager.getInstance().create(getBaseContext(), pairedBands[0]);
            Log.i(tag, "in else if calories");
            return true;
        }

        return ConnectionState.CONNECTED == client.connect().await();
    }

    //Code for listener for calories burned count - create Microsoft band heart rate event listener
    private BandCaloriesEventListener mCaloriesEventListener = new BandCaloriesEventListener() {
        private final String category = "calories";
        private int count=0;
        float calories;
        float caloriesToday;


        @Override
        public void onBandCaloriesChanged(BandCaloriesEvent bandCaloriesEvent) {
            if (bandCaloriesEvent !=null){

                calories = bandCaloriesEvent.getCalories();
                try {
                    caloriesToday = bandCaloriesEvent.getCaloriesToday();
                }catch (InvalidBandVersionException e) {
                    e.printStackTrace();

                }


                //Start thread to save microsoft band data to database - the count variable is used in the thread
                //and can be used to change the frequency with which data is saved to the database. i.e. it can
                //be used to ignore some data readings so that they don't get saved
                count+=1;

                if(count==5){
                    count = 0;
                }

                SaveMBandDataThread saveMBandData = new SaveMBandDataThread(category, calories, count);
                saveMBandData.start();

            }

        }
    };


    private class ProcessTrafficLightDataThread extends Thread {
        private final String trafficLightData;

        public ProcessTrafficLightDataThread(String trafficLightData){
            this.trafficLightData = trafficLightData;
        }

        public void run(){
            String[] subStrings = trafficLightData.split(" ");
            TrafficLightStatus tlStatus = TrafficLightStatus.Red;
            double latitude;
            double longitude;

            // TrafficLightSignal
            String trafficLightSignal = subStrings[1];
            switch(trafficLightSignal){
                case "Red":
                    tlStatus = TrafficLightStatus.Red;
                    break;
                case "Amber":
                    tlStatus = TrafficLightStatus.Amber;
                    break;
                case "Green":
                    tlStatus = TrafficLightStatus.Green;
                    break;
                default:
                    break;
            }

            // Get latitude and longitude doubles
            latitude = Double.valueOf(subStrings[3]);
            longitude = Double.valueOf(subStrings[5]);

            TrafficLightData trafficLightData = new TrafficLightData(tlStatus, (float)latitude, (float)longitude);

            dbHandler.addTrafficLightDataRow(trafficLightData);
        }
    }

    //Save data from the Microsoft band to the database in a separate thread
    private class SaveMBandDataThread extends Thread{
        private final MicrosoftBandData mBandData;
        int count;

        public SaveMBandDataThread(String category, float value, int count){
            mBandData = new MicrosoftBandData(category, value);
            this.count = count;
        }

        public void run(){
            //Uncomment the if statement here if you only want to save data based on value of count
           // if (count == 0) {
                dbHandler.addMBandRow(mBandData);
          //  }
        }
    }
}
