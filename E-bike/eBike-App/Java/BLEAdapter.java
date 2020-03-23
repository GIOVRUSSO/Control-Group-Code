/*
 * Class Name: BLEAdapter.java
 * Corresponding layout: activity_view_data.xml
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
 * Description: Implemented in BLEService, methods used in connecting to BLE device
 * */

package ie.ucd.smartrideRT;

import java.util.ArrayList;


import android.annotation.SuppressLint;
import android.bluetooth.BluetoothDevice;
import android.content.Context;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.BaseAdapter;
import android.widget.TextView;



@SuppressLint("InflateParams")
public class BLEAdapter extends BaseAdapter {
    private ArrayList<BluetoothDevice> mLeDevices;
    private LayoutInflater mInflater;


    public BLEAdapter(Context context) {
        super();
        mLeDevices = new ArrayList<BluetoothDevice>();
        mInflater = LayoutInflater.from(context);
    }


    @Override
    public int getCount() {
// TODO Auto-generated method stub
        return mLeDevices.size();
    }


    @Override
    public Object getItem(int arg0) {
// TODO Auto-generated method stub
        if (mLeDevices.size() <= arg0) {
            return null;
        }
        return mLeDevices.get(arg0);
    }


    @Override
    public long getItemId(int arg0) {
// TODO Auto-generated method stub
        return arg0;
    }


    @Override
    public View getView(int position, View view, ViewGroup arg2) {
// TODO Auto-generated method stub
        ViewHolder viewHolder;


        if (null == view) {
            viewHolder = new ViewHolder();
            view = mInflater.inflate(R.layout.listitem_device, null);
            viewHolder.deviceName = (TextView) view
                    .findViewById(R.id.device_name);
            view.setTag(viewHolder);
        } else {
            viewHolder = (ViewHolder) view.getTag();
        }


        BluetoothDevice device = mLeDevices.get(position);


        final String deviceName = device.getName();
        if (deviceName != null && deviceName.length() > 0)
            viewHolder.deviceName.setText(deviceName);
        else
            viewHolder.deviceName.setText("Unknown device");


        return view;
    }

    private class ViewHolder {
        public TextView deviceName;
    }

    public void setData(ArrayList<BluetoothDevice> mDevices) {
        this.mLeDevices = mDevices;
    }


/*
* Add Devices
*/
 
    public void addDevice(BluetoothDevice mDevice) {
        if (!mLeDevices.contains(mDevice)) {
            mLeDevices.add(mDevice);
        }
    }

/*
* List Devices
*/
    public BluetoothDevice getDevice(int position) {
        if (mLeDevices.size() <= position) {
            return null;
        }
        return mLeDevices.get(position);
    }


    /**
     * Clear Devices
     */
    public void clear() {
        mLeDevices.clear();
    }


}
