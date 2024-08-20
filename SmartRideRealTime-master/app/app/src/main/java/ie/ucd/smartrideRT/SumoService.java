/*
* Class Name: SumoService.java
* Corresponding layout: No
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: SumoService is used for open loop proactive pollution example. It is invoked in
* ProactivePollutionControl.java. It is used to transfer the most recent value of speed from
* the bike to SUMO service which is running on a server on the same IP (host) and port which is specified
* in both this .java file and the .py script which starts the SUMO server. It is also used to transfer the
* most recent heart rate value (from Microsoft Band) to the heartRate.py script which should also be
* running. As such the code to connect to the Microsoft Band is also included here.
* */

package ie.ucd.smartrideRT;

import android.app.Activity;
import android.app.Service;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.AsyncTask;
import android.os.Binder;
import android.os.IBinder;
import android.util.Log;
import android.os.Handler;

import com.microsoft.band.BandClient;
import com.microsoft.band.BandClientManager;
import com.microsoft.band.BandException;
import com.microsoft.band.BandInfo;
import com.microsoft.band.ConnectionState;
import com.microsoft.band.UserConsent;
import com.microsoft.band.sensors.BandHeartRateEvent;
import com.microsoft.band.sensors.BandHeartRateEventListener;
import com.microsoft.band.sensors.HeartRateConsentListener;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.lang.ref.WeakReference;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.Timer;
import java.util.TimerTask;


public class SumoService extends Service {

    private final IBinder sumoBinder = new SumoService.SumoServiceMyLocalBinder();
    private static final String tag = "debugging";
    BluetoothService bluetoothService;
    IntentFilter filter;
    private BandClient client = null;
    MyDBHandler dbHandler;

    //sumo sockets
    private Socket clientSocket = null;
    private BufferedWriter out = null;
    private BufferedReader in = null;

    //heart rate sockets
    private Socket HRclientSocket = null;
    private BufferedReader HRin = null;
    private BufferedWriter HRout = null;

    private Boolean HRping = false;
    private Boolean ping = false;
    private String incoming;
    private Timer listenTimer;
    private Timer reduceMTimer;
    private Timer increaseMTimer;


    public class SumoServiceMyLocalBinder extends Binder {
        SumoService getService(){
            return SumoService.this;
        }
    }


    @Override
    public IBinder onBind(Intent intent) {

        //constructor for MyDBHandler
        dbHandler = new MyDBHandler(this, null, null, 1);

        //start service for bluetooth connection
        Intent i = new Intent(this, BluetoothService.class);
        bindService(i, bluetoothServiceConnection, Context.BIND_AUTO_CREATE);



        //register receiver to listen for updates from bike
        registerSendDataReceiver();
        startListeningForSumo();

        //code for registering heart rate events
//        new HeartRateConsentTask().execute();
//        startHeartRateSubscriptionTask();
//        startHeartRateTransfer();


        return sumoBinder;
    }


    //register receiver to listen to data from BluetoothService.java
    private void registerSendDataReceiver() {
        filter = new IntentFilter("info.shaunsweeney.smartrideRT.sumo");
        registerReceiver(MyReceiver, filter);
    }


    //start heart rate subscription task
    public void startHeartRateSubscriptionTask(){
//        new HeartRateSubscriptionTask().execute();
//        new HeartRateSubscriptionTask().execute();
        Log.i(tag, "SS: heart rate subscription task should be registered");
    }


    //Broadcast Receiver to start thread to process data received from BluetoothService.java
    private final BroadcastReceiver MyReceiver = new BroadcastReceiver() {

        @Override
        public void onReceive(Context context, Intent intent) {

            String bikeDataReceived = intent.getStringExtra("sumo");
            //Log.i(tag, "SS: Bike data: "+bikeDataReceived);

            ProcessBikeDataThread processBikeDataThread = new ProcessBikeDataThread(bikeDataReceived);
            processBikeDataThread.start();
        }
    };


    //Thread to send data to heartRate.py script to plot graph
    private class SendHRDataThread extends Thread{
        private final String HRString;

        public SendHRDataThread(String string){
            this.HRString = string;
        }

        public void run(){

            if (HRping == true) {
                try {
                    HRout.write(HRString);
                    HRout.flush();
                    Log.i(tag, "HR is: "+HRString);
                }
                catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }

    }



    public void startHeartRateTransfer(){
        Log.i(tag, "started Listening for HR");
        new HeartRateConnectTask().execute();

    }


    //start listen task for data received from SUMO
    public void startListeningForSumo(){
        Log.i(tag, "started Listening for SUMO");
        new ConnectTask().execute();


        final Handler handler = new Handler();
//        listenTimer = new Timer();
        incoming = null;


        listenTimer = new Timer();
        listenTimer.schedule(new ListenTask(), 2000, 200);

//        TimerTask doAsynchronousTask = new TimerTask() {
//            @Override
//            public void run() {
//                handler.post(new Runnable() {
//                    public void run() {
//                        try {
//                            ListenTask performBackgroundTask = new ListenTask();
//                            performBackgroundTask.execute(ping);
//                        } catch (Exception e) {
//                        }
//                    }
//                });
//            }
//        };
//        listenTimer.schedule(doAsynchronousTask, 100, 300);
    }


    //disconnect from the service
    public void disconnectSumo(){
        new DisconnectTask().execute();
    }



    // Start connection to transfer heart rate data to heartRate.py script where it is plotted as a graph
    private class HeartRateConnectTask extends AsyncTask<Void, Void, Boolean> {
        protected Boolean doInBackground(Void... params) {
            if (HRping == false) {
                Log.i(tag, "HRping is false - connecting");
                try {
                    HRclientSocket = new Socket("192.168.43.177", 50112);
                    HRout = new BufferedWriter(new OutputStreamWriter(HRclientSocket.getOutputStream()));
                    HRin = new BufferedReader(new InputStreamReader(HRclientSocket.getInputStream()));
                    HRping = true;
                } catch (UnknownHostException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
            return HRping;
        }
        protected void onPostExecute(Boolean result) {
        }
    }



    // Start connection to SUMO based on provided IP and port
    private class ConnectTask extends AsyncTask<Void, Void, Boolean> {
        protected Boolean doInBackground(Void... params) {
            if (ping == false) {
                Log.i(tag, "ping is false - connecting");
                try {
                    //below change the socket IP (first parameter) and port (second parameter) for specific connection
                    clientSocket = new Socket("192.168.43.177", 50100);
                    out = new BufferedWriter(new OutputStreamWriter(clientSocket.getOutputStream()));
                    in = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
                    ping = true;
                } catch (UnknownHostException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
            return ping;
        }
        protected void onPostExecute(Boolean result) {
        }
    }

    //Thread to carry out control algorithm
    class ListenTask extends TimerTask{
        float mNow;

        public ListenTask(){

        }

        @Override
        public void run() {
            //Log.i(tag, "ping is: "+ ping);
            if (ping == true) {
                try {
                    incoming = in.readLine();
                    incoming = in.readLine();
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            } else if (ping == false) {
                Log.i(tag, "ping is false");
                listenTimer.cancel();
                incoming = "No Advice";
            }


            //int adviceFlag = Integer.parseInt(incoming);
            //Log.i(tag, "advice flag:"+adviceFlag);
            if (incoming.compareTo("A")==0) {
                float mTargetUnpolluted = 0.45f;
                dbHandler.addMTargetRow(mTargetUnpolluted);
                Log.i(tag, "saved: "+ mTargetUnpolluted);
//                String message="0!";
//                writeToBike(message);
                //  advised_speed.setText("Manual (Pedal)");
         //       mNow = dbHandler.getLatestMTarget();
//                if(Math.abs(mNow - mTargetUnpolluted) > 0.05f){
//                    Log.i(tag, "in if unpolluted");
//                    increaseMTimer = new Timer();
//                    increaseMTimer.schedule(new IncreaseMGraduallyTask(mTargetUnpolluted), 200, 1000);
//                }
            }
            if (incoming.compareTo("B")==0) {
                float mTargetUnpolluted = 0.6f;
                dbHandler.addMTargetRow(mTargetUnpolluted);
                Log.i(tag, "saved: "+ mTargetUnpolluted);
            }
            if (incoming.compareTo("C")==0) {
                float mTargetUnpolluted = 0.75f;
                dbHandler.addMTargetRow(mTargetUnpolluted);
                Log.i(tag, "saved: "+ mTargetUnpolluted);
            }
            if (incoming.compareTo("D")==0) {
                float mTargetUnpolluted = 0.9f;
                dbHandler.addMTargetRow(mTargetUnpolluted);
                Log.i(tag, "saved: "+ mTargetUnpolluted);
            }
            if (incoming.compareTo("1")==0) {
                float mTargetPolluted = 0.75f;
//                //Log.i(tag, "advice flag 150 is: "+adviceFlag);
                dbHandler.addMTargetRow(mTargetPolluted);
            //    mNow = dbHandler.getLatestMTarget();
//                if(Math.abs(mNow - mTargetPolluted) > 0.05f) {
//                    Log.i(tag, "in if polluted");
//                    reduceMTimer = new Timer();
//                    reduceMTimer.schedule(new ReduceMGraduallyTask(mTargetPolluted), 200, 1000);
//                }
               // String message="150!";
               // writeToBike(message);
                //   advised_speed.setText("Electric");
                //Log.i(tag, "saved: "+ mTargetPolluted);
            }
            if (incoming.compareTo("2")==0) {
                float mTargetPolluted = 0.6f;
                dbHandler.addMTargetRow(mTargetPolluted);
            }
            if (incoming.compareTo("3")==0) {
                float mTargetPolluted = 0.45f;
                dbHandler.addMTargetRow(mTargetPolluted);
            }
            if (incoming.compareTo("4")==0) {
                float mTargetPolluted = 0.3f;
                dbHandler.addMTargetRow(mTargetPolluted);
            }
        }

    }


    //Timer task to gradually reduce m
    class ReduceMGraduallyTask extends TimerTask{
        float mNow;
        float mTargetFinal;
        float mDecrease = 0.1f;
        float mNext;

        public ReduceMGraduallyTask(float mTargetFinal){
            this.mTargetFinal = mTargetFinal;
        }

        @Override
        public void run() {
        Log.i(tag, "in run reduce");
            //        float mNow;
        mNow = dbHandler.getLatestMTarget();

        if(mNow > mTargetFinal){
            mNext = mNow - mDecrease;
        }

        if (mNext < mTargetFinal){
            mNext = mTargetFinal;
            reduceMTimer.cancel();
        }

        dbHandler.addMTargetRow(mNext);
        Log.i(tag, "end of run reduce");
        }

    }


    class IncreaseMGraduallyTask extends TimerTask{
        float mNow;
        float mTargetFinal;
        float mIncrease = 0.1f;
        float mNext;

        public IncreaseMGraduallyTask(float mTargetFinal){
            this.mTargetFinal = mTargetFinal;
        }

        @Override
        public void run() {
            Log.i(tag, "in run increase");
            //        float mNow;
            mNow = dbHandler.getLatestMTarget();
            Log.i(tag, "mNow increase"+mNow);

            if(mNow < mTargetFinal){
                mNext = mNow + mIncrease;
            }

            if (mNext > mTargetFinal){
                mNext = mTargetFinal;
                increaseMTimer.cancel();
            }

            dbHandler.addMTargetRow(mNext);
            Log.i(tag, "mNext"+mNext);
            Log.i(tag, "at end of run increase");
        }

    }

//    public void reduceMGradually(float mTargetFinal){
//        float mNow;
//        float mTemp;
//        mNow = dbHandler.getLatestMTarget();
//
//        mTargetFinal - mNow;
//
//
//        dbHandler.addMTargetRow(mTemp);
//    }



    //method to send command to bike via bluetooth
    public void writeToBike(String message){
        byte[] send = message.getBytes();
        Log.i(tag, "Bytes are " + send);
        bluetoothService.write(send);
    }


    // Disconnects from SUMO.
    private class DisconnectTask extends AsyncTask<Void, Void, Boolean> {
        protected Boolean doInBackground(Void... params) {
            if (ping == true) {
                try {
                    ping = false;
                    out.write("quit");
                    out.close(); // close the writer connected to the socket
                    in.close(); // close the reader connected to the socket
                    clientSocket.close(); // close the socket
                } catch (IOException e) {
                    // TODO Auto-generated catch block
                    e.printStackTrace();
                }
            }
            return ping;
        }
        protected void onPostExecute(Boolean result) {
        }
    }

    //Kick off the heart rate reading
    private class HeartRateSubscriptionTask extends AsyncTask<Void, Void, Void> {
        @Override
        protected Void doInBackground(Void... params) {
            try {
                if(getConnectedHeartBandClient()){
                    if(client.getSensorManager().getCurrentHeartRateConsent() == UserConsent.GRANTED){
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

    //Need to get user consent for some Microsoft band data e.g. heart rate
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

            }
            catch (Exception e) {
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



    //create Microsoft Band heart rate event listener for data received from Microsoft Band
    private BandHeartRateEventListener mHeartRateEventListener = new BandHeartRateEventListener() {
        float heartRate;
        String category = "heartRate";
        private int count =0;

        @Override
        public void onBandHeartRateChanged(BandHeartRateEvent bandHeartRateEvent) {
            if (bandHeartRateEvent !=null){
                heartRate = bandHeartRateEvent.getHeartRate();

                String heartRateString = ""+heartRate;
                SendHRDataThread sendHRDataThread = new SendHRDataThread(heartRateString);
                sendHRDataThread.start();
            }
        }
    };



    private ServiceConnection bluetoothServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            BluetoothService.BluetoothMyLocalBinder binder  = (BluetoothService.BluetoothMyLocalBinder) service;
            bluetoothService = binder.getService();
        }

        @Override
        public void onServiceDisconnected(ComponentName name){

        }

    };

    //Process data received from the bike in a thread and send to SUMO
    private class ProcessBikeDataThread extends Thread{
        private final String bikeDataString;

        public ProcessBikeDataThread(String string){
            this.bikeDataString = string;
        }

        public void run(){
            String[] strings;
            Float[] floats = new Float[13];

            strings = bikeDataString.split("\\t");
            for(int i=0;i<5;i++){
                floats[i] = Float.valueOf(strings[i]);
            }
            //The only value that we care about for SumoService is the speed
            int speed =  Math.round(floats[3]);


            String current_speed_string = "";
            current_speed_string = Float.toString(speed);


            if (ping == true) {
                try {
                    //Log.i(tag, "current speed: "+current_speed_string);
                    //out.write(""+15);
                    //Log.i(tag, "current speed "+current_speed_string);
                    out.write(""+speed);
                    //out.write(current_speed_string);
                    out.flush();
                } catch (IOException e) {
                    e.printStackTrace();
                }
            }
        }
    }




}
