/*
 * Class Name: BluetoothService.java
 * Corresponding layout: No
 * Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
 * Date: March 2017
 * Description: BluetoothService connects to the bluetooth device (RN-41 module), and provides methods
 * to listen for incoming data from the bike and to send commands to the bike. It also passes the
 * incoming data from the bike to the Database by means of a Broadcast Receiver. Note - ensure that
 * bluetooth is turned on on phone and you have already paired with the device that you want to connect to.
 * */

package ie.ucd.smartrideRT;

import android.app.Service;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Handler;
import android.os.IBinder;
import android.os.Binder;
import android.os.Message;
import android.util.Log;
import android.bluetooth.BluetoothAdapter;
import android.bluetooth.BluetoothDevice;
import android.bluetooth.BluetoothSocket;
import android.content.BroadcastReceiver;
import android.widget.Toast;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.ObjectInputStream;
import java.io.OutputStream;
import java.io.UnsupportedEncodingException;
import java.util.ArrayList;
import java.util.Set;
import java.util.UUID;

import bikemessaging.TrafficLightMessage;

public class BluetoothService extends Service {
    private static final String tag = "debuggingBTS";

    private BluetoothTrafficLightConnectThread tlConnect;
    private BluetoothBikeConnectThread connect;

    private SmartRideConnectedThread connectedThread;
    private TrafficLightEmulatorConnectedThread connectedTrafficLightThread;

    BluetoothAdapter btAdapter;
    Set<BluetoothDevice> devicesArray;
    ArrayList<String> pairedDevices;
    //HashMap<String, BluetoothDevice> pairedDevices;
    //HashMap<String, BluetoothDevice> visibleDevices;
    ArrayList<BluetoothDevice> devices;
    MyDBHandler dbHandler;
    public static final UUID MY_UUID = UUID.fromString("00001101-0000-1000-8000-00805F9B34FB");

    protected static final int MESSAGE_READ = 1;
    protected static final int PROCESSING_DATA = 2;
    protected static final int SEND_TO_SUMO = 3;
    protected static final int SEND_HR_DATA = 4;
    protected static final int SUCCESS_CONNECT_BIKE = 5;
    protected static final int SUCCESS_CONNECT_TRAFFIC_LIGHT = 6;
    protected static final int TRAFFIC_LIGHT_DATA = 7;
    protected static final int FAIL_CONNECT_BIKE = 8;

    public boolean bikeIsConnected;
    private boolean trafficLightIsConnected;

    IntentFilter filter;

    private final String SMARTRIDE_BLUETOOTH_ADDRESS = "00:66:66:6E:DC:85";
    private final String PHONE_BLUETOOTH_ADDRESS = "5C:51:81:BC:11:B6";

    // Bluetooth is set up as a service which allows it to run in the background across multiple activities
    private final IBinder bluetoothBinder = new BluetoothMyLocalBinder();
    public class BluetoothMyLocalBinder extends Binder{
        BluetoothService getService(){
            return BluetoothService.this;
        }
    }

    @Override
    public IBinder onBind(Intent intent) {
        bluetoothStart();

        //Get ability to pull data from database
        dbHandler = new MyDBHandler(this, null, null, 1);

        return bluetoothBinder;
    }

    public void bluetoothStart(){
        btAdapter = BluetoothAdapter.getDefaultAdapter();
        pairedDevices = new ArrayList<>();
        //visibleDevices = new HashMap<>();
        devices = new ArrayList<BluetoothDevice>();
        bikeIsConnected = false;
        trafficLightIsConnected = false;

        if (btAdapter == null) {
            Toast.makeText(getApplicationContext(), "No bluetooth detected", Toast.LENGTH_SHORT).show();
        }
        else {
            if (!btAdapter.isEnabled()) {
                turnOnBT();
            }
            getPairedDevices();
            startDiscovery();
            callFilters();
        }
    }

    // Turn on Bluetooth if it is not already on
    private void turnOnBT() {
        Intent btIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
        btIntent.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        startActivity(btIntent);
    }

    // Look for new bluetooth devices
    private void startDiscovery() {
        btAdapter.cancelDiscovery();
        btAdapter.startDiscovery();
    }

    //get paired devices
    private void getPairedDevices() {
        String sendString;
        Intent device_intent = new Intent();
        devicesArray = btAdapter.getBondedDevices();
        if (devicesArray.size() > 0) {
            for (BluetoothDevice device : devicesArray) {
                pairedDevices.add(device.getName());
                devices.add(device);
                String s = "";
                for (int a = 0; a < pairedDevices.size(); a++) {
                    if (device.getName().equals(pairedDevices.get(a))) {
                        s = "Paired";
                        break;
                    }
                }

                //send Broadcast intent of device as string to appear on UI in MainActivity
                sendString = device.getName() + " " + s + " " + "\n" + device.getAddress();
                device_intent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
                device_intent.putExtra("device", sendString);
                device_intent.setAction("ie.ucd.smartrideRT");
                sendBroadcast(device_intent);

            }
        }
    }

    // Register receivers to listen for changes in status of bluetooth connection
    public void callFilters(){
        filter = new IntentFilter(BluetoothDevice.ACTION_FOUND);
        registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_STARTED);
        registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_DISCOVERY_FINISHED);
        registerReceiver(receiver, filter);
        filter = new IntentFilter(BluetoothAdapter.ACTION_STATE_CHANGED);
        registerReceiver(receiver, filter);
    }

    //checkifPaired is used in MainActivity to see if user is paired with device they have selected
    public void checkifPaired(int arg2){
        BluetoothDevice selectedDevice = devices.get(arg2);
        Log.i(tag, "selectedDevice name is " + selectedDevice.getName());
        BluetoothService.BluetoothBikeConnectThread connect = new BluetoothService.BluetoothBikeConnectThread(selectedDevice);
        connect.start();
    }

    // connectToPairedDevice is used in MainActivity to see if user is paired with device they have selected
//    public void connectToPairedDevice(String address){
//        BluetoothDevice selectedDevice = pairedDevices.get(address);
//
//        if(selectedDevice.getAddress().equals(PHONE_BLUETOOTH_ADDRESS)){
//            if(tlConnect != null){
//                tlConnect.cancel();
//            }
//            tlConnect = new BluetoothService.BluetoothTrafficLightConnectThread(selectedDevice);
//            tlConnect.start();
//        }
//        else if(selectedDevice.getAddress().equals(SMARTRIDE_BLUETOOTH_ADDRESS)) {
//            if(connect != null){
//                connect.cancel();
//            }
//            connect = new BluetoothService.BluetoothBikeConnectThread(selectedDevice);
//            connect.start();
//        }
//    }

    //BroadcastReceiver is to take action depending on state of bluetooth and to manage if Bluetooth
    //gets turned off when app is running
    // BroadcastReceiver is to take action depending on state of bluetooth and to manage if Bluetooth
    // gets turned off when app is running
    private final BroadcastReceiver receiver = new BroadcastReceiver() {
        public static final String BROADCAST_ACTION = "ie.ucd.smartrideRT.displayevent";
        @Override
        public void onReceive(Context context, Intent intent) {
            String action = intent.getAction();
            Log.d("debuggg", "onreceive: " + action);
            if (BluetoothDevice.ACTION_FOUND.equals(action)) {
                //Toast.makeText(getApplicationContext(), "ACTION_FOUND", Toast.LENGTH_SHORT).show();
                BluetoothDevice device = intent.getParcelableExtra(BluetoothDevice.EXTRA_DEVICE);
                devices.add(device);
                String s = "";
//                Log.i(tag, "is device null?" + (device == null));
//                for (int a = 0; a < pairedDevices.size(); a++) {
//                    if (device.getName().equals(pairedDevices.get(a))) {
//                        //append
//                        s = "Paired";
//                        break;
//                    }
//                }
            } else if (BluetoothAdapter.ACTION_DISCOVERY_FINISHED.equals(action)) {


            } else if (BluetoothAdapter.ACTION_STATE_CHANGED.equals(action)) {
                if (btAdapter.getState() == btAdapter.STATE_OFF) {
                    turnOnBT();
                    getPairedDevices();
                    startDiscovery();
                    callFilters();
                }
                else if(btAdapter.getState() == btAdapter.STATE_ON) {
                    getPairedDevices();
                    startDiscovery();
                    callFilters();
                }
            }
        }
    };

    /**
     * Write to the ConnectedThread in an unsynchronized manner
     *
     * @param out The bytes to write
     * @see SmartRideConnectedThread#write(byte[])
     */
    public void write(byte[] out) {
        // Create temporary object
        SmartRideConnectedThread r;
        // Synchronize a copy of the ConnectedThread
        synchronized (this) {
            //if (mState != STATE_CONNECTED) return;
            r = connectedThread;
        }
        // Perform the write unsynchronized
        r.write(out);
    }
//客户端获取BluetoothSocket， 获取线程
    public class BluetoothBikeConnectThread extends Thread {
        private BluetoothDevice bluetoothDevice;
        private BluetoothSocket bluetoothSocket;
        private boolean rejectedDevice = false;

        public BluetoothBikeConnectThread(BluetoothDevice bluetoothDevice){
            BluetoothSocket tmp = null;

            this.bluetoothDevice = bluetoothDevice;

            Log.i(tag, "Constructing BluetoothBikeConnectThread");

            try{
                //MY_UUID is the app's UUID string, also used by the server code
                tmp = bluetoothDevice.createRfcommSocketToServiceRecord(MY_UUID);
            }
            catch(IOException e){
                Log.i(tag, "Get socket failed in BluetoothBikeConnectThread");
            }

            bluetoothSocket = tmp;
        }

        public void run() {
            try {
                // Connect the device through the socket. This will block
                // until it succeeds or throws an exception
                bluetoothSocket.connect();
                Log.i(tag, "connect - succeeded");
            }
            catch (IOException connectException) {
                Log.i(tag, "connect failed");
                try {
                    bluetoothSocket.close();
                    mHandler.obtainMessage(FAIL_CONNECT_BIKE, bluetoothSocket).sendToTarget();
                }
                catch (IOException closeException) {
                }
                return;
            }

            // Do work to manage the connection (in a separate thread)
            mHandler.obtainMessage(SUCCESS_CONNECT_BIKE, bluetoothSocket).sendToTarget();
        }

        public boolean wasDeviceRejected(){
            return rejectedDevice;
        }

        public void cancel(){
            try {
                bluetoothSocket.close();
            }
            catch (IOException e) {}
        }
    }

    public class BluetoothTrafficLightConnectThread extends Thread {
        private BluetoothDevice bluetoothDevice;
        private final BluetoothSocket bluetoothSocket;

        public BluetoothTrafficLightConnectThread(BluetoothDevice btDevice){
            BluetoothSocket tmp = null;
            bluetoothDevice = btDevice;
            btAdapter.cancelDiscovery();

            try{
                //MY_UUID is the app's UUID string, also used by the server code
                tmp = bluetoothDevice.createRfcommSocketToServiceRecord(MY_UUID);
            }
            catch(IOException e){
                Log.i(tag, "Get socket failed in BluetoothTrafficLightConnectThread");
            }

            bluetoothSocket = tmp;
        }

        public void run(){
            try {
                // Connect the device through the socket. This will block
                // until it succeeds or throws an exception
                Log.i(tag, "TrafficLightConnect trying to connect...");
                bluetoothSocket.connect();
                Log.i(tag, "TrafficLightConnect succeeded.");

                OutputStream outputStream = bluetoothSocket.getOutputStream();
                Log.i(tag, "OutputStream is " + outputStream.toString());

                byte[] b = {(byte) 1, (byte) 2};
                outputStream.write(b);
                Log.i(tag, "Wrote bytes to the output stream");
            }
            catch (IOException connectException) {
                Log.i(tag, "TrafficLightConnect failed.\n" + connectException.toString());

                try {
                    bluetoothSocket.close();
                    Log.i(tag, "Closed the socket.");
                }
                catch (IOException closeException) {
                    Log.i(tag, "Failed to close socket after connection unsuccessful." + closeException);
                }
                return;
            }

            Log.i(tag, "Handler taking the socket.");
            // Do work to manage the connection (in a separate thread)
            mHandler.obtainMessage(SUCCESS_CONNECT_TRAFFIC_LIGHT, bluetoothSocket).sendToTarget();
        }

        public void cancel(){
            try {
                bluetoothSocket.close();
            }
            catch (IOException e) {}
        }
    }

    // Once the device is connected by Bluetooth set up methods to enable us to read and write to the socket
    //数据传输， 蓝牙数据处理线程
    public class SmartRideConnectedThread extends Thread {
        private BufferedReader bufferedReader;
        private final OutputStream mmOutStream;
        private final BluetoothSocket mmSocket;

        public SmartRideConnectedThread(BluetoothSocket socket, int count) {
            mmSocket = socket;
            OutputStream tmpOut = null;
            InputStream tmpIn = null;

            try {
                tmpOut = mmSocket.getOutputStream();
                tmpIn = mmSocket.getInputStream();
            }
            catch (IOException e) {
                try{
                    mmSocket.close();
                } catch (IOException e1){
                    Log.i(tag, "Could not close the socket in SmartRideConnectThread.");
                }
            }

            try{
                bufferedReader = new BufferedReader(new InputStreamReader(tmpIn, "UTF-8"));
            }
            catch(UnsupportedEncodingException e1){
                e1.printStackTrace();
            }

            mmOutStream = tmpOut;
            bikeIsConnected = true;
        }

        @Override
        public void run() {
            int numBytes = 0; // bytes returned from read()
            int begin = 0;
            Log.i(tag, "in the SmartRideConnectedThread run method");
            String s;

            while(true) {
                try {
                    if (bufferedReader.ready()) {
                        s = bufferedReader.readLine();

                        // Line below case could be either PROCESSING_DATA or SEND_TO_SUMO depending on use case
                        mHandler.obtainMessage(PROCESSING_DATA, s).sendToTarget();
                        mHandler.obtainMessage(SEND_TO_SUMO, s).sendToTarget();
                    }
                    s = "";
                } catch (IOException e) {
                    //回调断开连接
                    bikeIsConnected = false;
                    e.printStackTrace();
                    Log.i(tag, "some kind of exception");
                }
            }
        }
//发送数据
        public void write(byte[] bytes) {
            String commandSent;
            try {
                mmOutStream.write(bytes);
            } catch (IOException e) {

            }
            commandSent = new String(bytes);

            // Need to remove the '!' character from this string before entering in database
            commandSent = commandSent.replace("!", "");
            CommandSentData commandSentData = new CommandSentData(commandSent);
            dbHandler.addCommandSentRow(commandSentData);
        }

        /* Call this from the main activity to shutdown the connection */
        //关闭连接
        public void cancel() {
            bikeIsConnected = false;
            try {
                mmSocket.close();
            } catch (IOException e) {

            }
        }
    }

    public class TrafficLightEmulatorConnectedThread extends Thread {
        private final BluetoothSocket mmSocket;
        private InputStream inputStream;
        private ObjectInputStream ois;
        private OutputStream outputStream;

        public TrafficLightEmulatorConnectedThread(BluetoothSocket socket, int count){
            mmSocket = socket;

            try{
                inputStream = mmSocket.getInputStream();
                ois = new ObjectInputStream(inputStream);
                Log.i(tag, "In connected thread");
            }
            catch(IOException e){
                Log.i(tag, "Could not initialise InputStream or ObjectInputStream\n" + e.toString());
                cancel();
            }

            try{
                Log.i(tag, "Initialising outputStream");
                outputStream = mmSocket.getOutputStream();
            }
            catch(IOException e){
                Log.i(tag, "Failed to initialise outputStream.");
                cancel();
            }

            trafficLightIsConnected = true;
        }

        public void run() {
            int numBytes = 0; // bytes returned from read()
            int begin = 0;
            TrafficLightMessage trafficLightMessage;
            Log.i(tag, "In the TrafficLightEmulatorConnectedThread run method");

            while(true) {
                try {
                    Log.i(tag, "Trying to read in an object in the TLConnectedThread run method.");
                    trafficLightMessage = (TrafficLightMessage) ois.readObject();
                    Log.i(tag, "TrafficLightMessage is null? " + (trafficLightMessage == null));


                    Log.i(tag, "Lat: " + trafficLightMessage.getSimpleLocation().getLatitude() + "\n" +
                            "Long: " + trafficLightMessage.getSimpleLocation().getLongitude() + "\n" +
                            "Status: " + trafficLightMessage.getTrafficLightStatus());

                    // Must send this data to the ui thread
                    Message tlMessage = new Message();
                    String tlMessageString = trafficLightMessage.toString();
                    mHandler.obtainMessage(TRAFFIC_LIGHT_DATA, tlMessageString).sendToTarget();

                    // Want to write this data to the database

                    // Want to write the location to the database from the UI thread also
                } catch (Exception e) {
                    trafficLightIsConnected = false;
                    e.printStackTrace();
                    Log.i(tag, "some kind of exception");

                    try{
                        mmSocket.close();
                    }
                    catch(IOException e1){
                        Log.i(tag, "Could not close the socket.");
                    }
                    // Connection has most likely been lost or
                    // terminated, so end this thread, and start up a new
                    // TrafficLightConnect thread for the next one.
                    break;
                }
            }
        }

        public void write(byte[] array){
            try{
                outputStream.write(array);
            }
            catch(IOException e){
                Log.i(tag, "Could not write to outputStream.");
            }
        }

        public void cancel(){
            try{
                inputStream.close();
                outputStream.close();
                ois.close();
                mmSocket.close();
                trafficLightIsConnected = false;
            }
            catch(IOException e){
                Log.i(tag, "Failed to close streams and/or bluetooth socket.");
            }
        }
    }

    // Handler to manage messages to send to the UI thread
    Handler mHandler = new Handler(new Handler.Callback(){
        @Override
        public boolean handleMessage(Message msg){
            int count = 0;
            switch (msg.what) {
                case SUCCESS_CONNECT_BIKE:
                    Log.i(tag, "Connected to bike");
                    Toast.makeText(getApplicationContext(), "Successfully connected to bike!", Toast.LENGTH_SHORT).show();
                    connectedThread = new SmartRideConnectedThread((BluetoothSocket) msg.obj, count);
                    connectedThread.start();
                    break;
                case FAIL_CONNECT_BIKE:
                    Toast.makeText(getApplicationContext(), "Failed to connect - please try again.", Toast.LENGTH_SHORT).show();
                    break;
                case SUCCESS_CONNECT_TRAFFIC_LIGHT:
                    Log.i(tag, "Have a successful connection to hand off - is socket connected? " + ((BluetoothSocket) msg.obj).isConnected());
                    connectedTrafficLightThread = new TrafficLightEmulatorConnectedThread((BluetoothSocket) msg.obj, count);
                    connectedTrafficLightThread.start();
                    break;
                // Send data to be saved in the database - see DatabaseService
                case PROCESSING_DATA:
                    String databaseEntry = (String) msg.obj;
                    Intent database_intent = new Intent();
                    database_intent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
                    database_intent.putExtra("database", databaseEntry);
                    database_intent.setAction("ie.ucd.smartrideRT.database");
                    sendBroadcast(database_intent);
                    break;
                case TRAFFIC_LIGHT_DATA:
                    String tlMessage = (String) msg.obj;
                    String[] tlMessageStrings = tlMessage.split(" ");

                    TrafficLightStatus tlStatus = TrafficLightStatus.valueOf(tlMessageStrings[0]);
                    double latitude = Double.parseDouble(tlMessageStrings[1]);
                    double longitude = Double.parseDouble(tlMessageStrings[2]);

                    String tlDatabaseEntry = "Signal " + tlStatus.toString() + " Latitude " + latitude + " Longitude " + longitude;
                    Intent tlDatabaseIntent = new Intent();
                    tlDatabaseIntent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
                    tlDatabaseIntent.putExtra("database", tlDatabaseEntry);
                    tlDatabaseIntent.setAction("ie.ucd.smartrideRT.database");
                    sendBroadcast(tlDatabaseIntent);
                    break;
                case SEND_HR_DATA:
                    String hrData = (String) msg.obj;
                    Intent hr_intent = new Intent();
                    hr_intent.addFlags(Intent.FLAG_INCLUDE_STOPPED_PACKAGES);
                    hr_intent.putExtra("sumo", hrData);
                    hr_intent.setAction("info.shaunsweeney.smartrideRT.heartrate");
                    sendBroadcast(hr_intent);
                    break;
            }
            return true;
        }
    });
}
