/*
* Class Name: DatabaseService.java
* Corresponding layout: No
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
* Description: DatabaseService processes data received from the bike by Bluetooth and passes it to
* MyDBHandler to save it to the database. For convenience methods relating to getting data from the
* MicrosoftBand (heart rate, calories burned) are also implemented here. If desired these could be moved
* into their own service and transferred directly to the MyDBHandler
* */

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
            int currentSecond = c.get(Calendar.SECOND);
            if(currentSecond==0 || currentSecond%5==0){
                DownloadALthread downloadALthread = new DownloadALthread();
                downloadALthread.start();
            }


            if (minute == -1)
                minute = currentMinute;
            if (currentMinute != minute) {
                minute = currentMinute;
                UploadBikeDataThread uploadBikeDataThread = new UploadBikeDataThread();
                uploadBikeDataThread.start();
                // Insert delay here
                // Thread.sleep(200);
                }

            else{
                if(splitBikeDataReceived[0].equals("Signal")){
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

    private class DownloadALthread extends Thread{
        public void run(){
            try {
                Download.download();
            } catch (Exception E){
                System.out.println("Call download exception");
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

//                SaveMBandDataThread saveMBandData = new SaveMBandDataThread(category, heartRate, count);
//                saveMBandData.start();

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




}
