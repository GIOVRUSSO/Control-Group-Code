/*
* Class Name: SendCommand.java
* Corresponding layout: activity_send_command.xml
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: SendCommand enables user to manually send a command by bluetooth to the bike to activate
* the motor. Commands that are sent should be of the form "90!" or "150!", i.e. a number with
* an exclamation mark at the end. Values <90 are too small to turn on the motor and values >150 will
* have no greater noticeable affect (motor / arduino voltage limitation)
* */

package ie.ucd.smartrideRT;

import android.content.ComponentName;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.os.IBinder;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.EditText;


public class SendCommand extends AppCompatActivity {
    private BLEService mBLEService;
    boolean isBound=false;
    private static final String tag = "SendCommandDebugging";
    IntentFilter filter;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_send_command);

        //start service for bluetooth connection
        Intent i = new Intent(this, BLEService.class);
//        bindService(i, BLEServiceConnection, Context.BIND_AUTO_CREATE);

    }

    //onClick method indicates user wants to send the command they have entered to the bike
    public void onClick(View view){
        final EditText enterCommand = (EditText) findViewById(R.id.enterCommand);
        String bikeCommand = enterCommand.getText().toString();
        write(bikeCommand);
    }

    //onBackClick indicates user wants to go back to the main menu
    public void onBackClick(View view){
        Log.i(tag, "Back to main");
        Intent i = new Intent(this, MainActivity.class);
        startActivity(i);
    }


    //write method is used to send the message the user has inputted to the bike
    private void write(String message) {
        if (message.length() > 0) {
            byte[] send = message.getBytes();
            Log.i(tag, "Bytes are " + send);
//            BLEService.wirteToBLE(send);
        }
    }


    //bluetoothServiceConnection is needed to use BluetoothService methods including writing
    private ServiceConnection bluetoothServiceConnection = new ServiceConnection(){
        @Override
        public void onServiceConnected(ComponentName name, IBinder service){
            //BluetoothService.BluetoothMyLocalBinder binder  = (BluetoothService.BluetoothMyLocalBinder) service;
            BLEService.LocalBinder binder = (BLEService.LocalBinder) service;
            isBound = true;
        }


        @Override
        public void onServiceDisconnected(ComponentName name){
            mBLEService = null;
            isBound = false;
        }

    };
}
