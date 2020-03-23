/*
 * Class Name: MyDBHandler.java
 * Corresponding layout: No
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
 * Description: MyDBHandler extends SQLiteOpenHelper and is used to set up a database on the android
 * phone. It is used below to create different SQL tables. The structure of the tables (table names,
 * column names, data types for each column etc.) are all specified here. Methods for writing to the
 * database or pulling data from the database are all also implemented here.
 * N.B. If it is desired to add a new table / modify an existing table / clear all saved data in the
 * database this can be achieved by changing the variable "private static final int DATABASE_VERSION",
 * it should be incremented by 1, then the next time the database is bound to (onBind), the tables
 * will be recreated.
 * */

package ie.ucd.smartrideRT;

import android.database.sqlite.SQLiteDatabase;
import android.database.sqlite.SQLiteOpenHelper;
import android.database.Cursor;
import android.content.Context;
import android.content.ContentValues;
import android.util.Log;

import java.text.ParseException;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

import bikemessaging.SimpleLocation;

public class MyDBHandler extends SQLiteOpenHelper {

    private static final String tag = "debugging";
    private static final int DATABASE_VERSION = 124;
    private static final String DATABASE_NAME = "data.db";
    public static final String TABLE_BIKEDATA = "bikeData";
    public static final String TABLE_COMMANDSENT = "commandSent";
    public static final String TABLE_HEARTRATE = "heartRate";
    public static final String TABLE_BIKE_LOCATION = "bikeLocation";

    //Common column names
    public static final String COLUMN_ID = "_id";
    public static final String COLUMN_TIME = "time";

    //Column names for bike data table
    private static final String COLUMN_BATTERYENERGY = "batteryEnergy";
    private static final String COLUMN_VOLTAGE = "voltage";
    private static final String COLUMN_CURRENT = "current";
    private static final String COLUMN_SPEED = "speed";
    private static final String COLUMN_DISTANCE = "distance";
    private static final String COLUMN_TEMPERATURE = "temperature";
    private static final String COLUMN_RPM = "RPM";
    private static final String COLUMN_HUMANPOWER = "humanPower";
    private static final String COLUMN_TORQUE = "torque";
    private static final String COLUMN_THROTTLEIN = "throttleIn";
    private static final String COLUMN_THROTTLEOUT = "throttleOut";
    private static final String COLUMN_ACCELERATION = "acceleration";
    private static final String COLUMN_FLAG = "flag";

    //Column names for Microsoft band data table
    private static final String COLUMN_HEARTRATE = "heartRate";

    //Column names for command sent table
    private static final String COLUMN_COMMANDSENT = "commandSent";
    private static final String COLUMN_GAINPARAMETER = "gainParameter";

    //column names for motor filter table
    private static final String COLUMN_COUNT = "count";

    // Column names for Bike location data
    private static final String COLUMN_BIKE_LOCATION_LATITUDE = "latitude";
    private static final String COLUMN_BIKE_LOCATION_LONGITUDE = "longitude";

    public MyDBHandler(Context context, String name, SQLiteDatabase.CursorFactory factory, int version) {
        super(context, DATABASE_NAME, factory, DATABASE_VERSION);
    }

    //first time table is created
    @Override
    public void onCreate(SQLiteDatabase db) {
        //String with query to create table for cycle analyst data (invoked at end of this method)
        String bikeDataQuery = "CREATE TABLE " + TABLE_BIKEDATA + "("
                + COLUMN_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COLUMN_TIME + " TIMESTAMP DEFAULT CURRENT_TIMESTAMP, " +
                COLUMN_BATTERYENERGY + " FLOAT, " +
                COLUMN_VOLTAGE + " FLOAT, " +
                COLUMN_CURRENT + " FLOAT, " +
                COLUMN_SPEED + " FLOAT, " +
                COLUMN_DISTANCE + " FLOAT, " +
                COLUMN_TEMPERATURE + " FLOAT, " +
                COLUMN_RPM + " FLOAT, " +
                COLUMN_HUMANPOWER + " FLOAT, " +
                COLUMN_TORQUE + " FLOAT, " +
                COLUMN_THROTTLEIN + " FLOAT, " +
                COLUMN_THROTTLEOUT + " FLOAT, " +
                COLUMN_ACCELERATION + " FLOAT, " +
                COLUMN_FLAG + " STRING " +
                ");";
        Log.i(tag, bikeDataQuery);


        //String with query to create table for heart rate from Microsoft Band
        String heartRateQuery = "CREATE TABLE " + TABLE_HEARTRATE + "("
                + COLUMN_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COLUMN_TIME + " TIMESTAMP DEFAULT CURRENT_TIMESTAMP, " +
                COLUMN_HEARTRATE + " FLOAT " +
                ");";
        Log.i(tag, heartRateQuery);

        //String with query to create table to save all commands sent to the bike
        String commandSentQuery = "CREATE TABLE " + TABLE_COMMANDSENT + "("
                + COLUMN_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COLUMN_TIME + " TIMESTAMP DEFAULT CURRENT_TIMESTAMP, " +
                COLUMN_COMMANDSENT + " STRING " +
                ");";
        Log.i(tag, commandSentQuery);




        String bikeLocationQuery = "CREATE TABLE " + TABLE_BIKE_LOCATION + "(" +
                COLUMN_ID + " INTEGER PRIMARY KEY AUTOINCREMENT, " +
                COLUMN_BIKE_LOCATION_LATITUDE + " FLOAT, " +
                COLUMN_BIKE_LOCATION_LONGITUDE + " FLOAT " +
                ");";
        Log.i(tag, bikeLocationQuery);

        //how to execute SQL queries on android
        db.execSQL(bikeDataQuery);
        Log.i(tag, "bikeData table should be created");
        db.execSQL(heartRateQuery);
        Log.i(tag, "heartRate table should be created");
        db.execSQL(commandSentQuery);
        Log.i(tag, "commandSent table should be created");
        db.execSQL(bikeLocationQuery);
        Log.i(tag, "bike location table should be created");
    }

    //this is the method that is called if the DATABASE_VERSION variable gets incremented
    @Override
    public void onUpgrade(SQLiteDatabase db, int oldVersion, int newVersion) {
        String bikeDataQuery = "DROP TABLE IF EXISTS " + TABLE_BIKEDATA + ";";
        db.execSQL(bikeDataQuery);

        String heartRateQuery = "DROP TABLE IF EXISTS " + TABLE_HEARTRATE + ";";
        db.execSQL(heartRateQuery);

        String commandSentQuery = "DROP TABLE IF EXISTS " + TABLE_COMMANDSENT + ";";
        db.execSQL(commandSentQuery);

        String bikeLocationQuery = "DROP TABLE IF EXISTS " + TABLE_BIKE_LOCATION + ";";
        db.execSQL(bikeLocationQuery);

        onCreate(db);
    }

    private String getDateTime() {
        //There is a problem with using Android SimpleDateFormat - android wants latest API which seems to be a bug
        java.text.SimpleDateFormat dateFormat = new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS");
        Date date = new Date();
        return dateFormat.format(date);
    }

    //Add a new row to the bikedata table
    public void addBikeDataRow(BikeData bikeData) {
        //Use ContentValues for Bikedata table
        ContentValues bikeDataValues = new ContentValues();
        bikeDataValues.put(COLUMN_TIME, getDateTime());
        bikeDataValues.put(COLUMN_BATTERYENERGY, bikeData.get_batteryEnergy());
        bikeDataValues.put(COLUMN_VOLTAGE, bikeData.get_voltage());
        bikeDataValues.put(COLUMN_CURRENT, bikeData.get_current());
        bikeDataValues.put(COLUMN_SPEED, bikeData.get_speed());
        bikeDataValues.put(COLUMN_DISTANCE, bikeData.get_distance());
        bikeDataValues.put(COLUMN_TEMPERATURE, bikeData.get_temperature());
        bikeDataValues.put(COLUMN_RPM, bikeData.get_RPM());
        bikeDataValues.put(COLUMN_HUMANPOWER, bikeData.get_humanPower());
        bikeDataValues.put(COLUMN_TORQUE, bikeData.get_torque());
        bikeDataValues.put(COLUMN_THROTTLEIN, bikeData.get_throttleIn());
        bikeDataValues.put(COLUMN_THROTTLEOUT, bikeData.get_throttleOut());
        bikeDataValues.put(COLUMN_ACCELERATION, bikeData.get_acceleration());
        bikeDataValues.put(COLUMN_FLAG, bikeData.get_flag());


        //db equals database we are going to write to
        SQLiteDatabase db = getWritableDatabase();

        //insert new row database
        long errorCheck = 0;
        db.insert(TABLE_BIKEDATA, null, bikeDataValues);

        // Use this to keep the DB table size quite small
        //db.delete(TABLE_BIKEDATA, "WHERE ? IN (SELECT ? FROM ? ORDER BY ? ASC LIMIT -1 OFFSET 100)", new String[]{"id", "id", TABLE_BIKEDATA, "id"});
        //db.execSQL("DELETE FROM " + TABLE_BIKEDATA + " WHERE _id IN (SELECT _id FROM " + TABLE_BIKEDATA + " ORDER BY _id DESC LIMIT -1 OFFSET 100)"); //TODO:
        //close database
        db.close();
    }



    public void addBikeLocation(float latitude, float longitude) {
        ContentValues bikeLocationDataValues = new ContentValues();
        bikeLocationDataValues.put(COLUMN_BIKE_LOCATION_LATITUDE, latitude);

        SQLiteDatabase db = getWritableDatabase();

        db.insert(TABLE_BIKE_LOCATION, null, bikeLocationDataValues);

        //You should not close the DB since it will be used again in the next call.
        //(BLE only tested bike data method)
         db.close();
    }

    //Add a new row to the database


    //Add a new row to the database which stores variables from closed loop feedback control - targets, errors etc.


    public void addCommandSentRow(CommandSentData commandSentData) {
        ContentValues commandSentValues = new ContentValues();
        commandSentValues.put(COLUMN_TIME, getDateTime());
        commandSentValues.put(COLUMN_COMMANDSENT, commandSentData.get_commandSent());

        //db equals database we are going to write to
        SQLiteDatabase db = getWritableDatabase();

        //insert new row database
        db.insert(TABLE_COMMANDSENT, null, commandSentValues);

        //close database
        db.close();
    }



    //Print out certain columns of bike data table as string
    public String getBikeData() {
        SQLiteDatabase db = getWritableDatabase();
        String dbString = "";
        String query = "SELECT * FROM " + TABLE_BIKEDATA + " WHERE 1;";

        //cursor points to a location in results
        Cursor c = db.rawQuery(query, null);

        //Move to first row in results
        c.moveToFirst();
        //Log.i(tag, "Bikedata cursor position is " + c.getPosition());
        //Log.i(tag, "Bikedata cursor count is " + c.getCount());

        //make sure there are still some results to go
        while (!c.isAfterLast()) {
            //if(c.getString(c.getColumnIndex("productname"))!=null){
            if (c.getString(c.getColumnIndex("time")) != null) {
                dbString += "Time: " + c.getString(c.getColumnIndex("time"));
                dbString += " \n";
                Log.i(tag, dbString);
            }
            c.moveToNext();
        }
        //Log.i(tag, "Cursor position is " + c.getPosition());
        db.close();

        return dbString;
    }





    //this method returns a treemap containing recent values of data saved to database - the reason why
    //it is formatted as a treemap is in case the control method needs several recent values of data e.g.
    //the 5 most recent values of heart rate to be used in the control loop, so it must be returned in
    //a structured format
    public List<float[]> getRecentBikeData(float motorEfficiency, float torqueBias, float cranksetEfficiency, float humanPowerFactor, int windowSize) {
        int count = 0;
        int numIterations;
        float voltage; //Volts
        float current; //Amps
        float humanPower; //Watts
        float torque;
        float rpm;
        float lastRequestToBike;
        float[] motorPowerArray;
        float[] humanPowerArray;
        float pi = (float) Math.PI;
        float gamma1 = 0.1640f;
        float gamma2 = -12.8473f;
        List<float[]> bikeDataList = null;
        String dateString;

        //Create TreeMap to store data retrieved from database
        // Map<Date, Float[]> m = new TreeMap<Date, Float[]>();
        java.text.SimpleDateFormat dateFormat = new java.text.SimpleDateFormat("yyyy-MM-dd HH:mm:ss");

        //Check if data has been synced recently - to be implemented

        //Get most recent data
        SQLiteDatabase db = getWritableDatabase();
        String query = "SELECT * FROM " + TABLE_BIKEDATA + " WHERE 1;";

        //cursor points to a location in results
        Cursor c = db.rawQuery(query, null);

        //Get most recent results
        boolean moveToLastSucceeded = c.moveToLast();

        // Set numIterations above to choose how many results are needed
        numIterations = windowSize;
        motorPowerArray = new float[numIterations];
        humanPowerArray = new float[numIterations];

        //need most recent command sent to bike also
        lastRequestToBike = getLastRequestToBike();

        // Check recent data has synced / we are not analysing old data (based on datetime)
        for (int i = 0; i < numIterations; i++) {
            if (c.getString(c.getColumnIndex("voltage")) != null) {
                // Calculate motor power
                voltage = Float.parseFloat(c.getString(c.getColumnIndex("voltage")));
                current = Float.parseFloat(c.getString(c.getColumnIndex("current")));
                float motorPower = Math.max(voltage * current * motorEfficiency - (lastRequestToBike * gamma1 + gamma2), 0);

                // Calculate human power
                torque = Float.parseFloat(c.getString(c.getColumnIndex("torque")));
                rpm = Float.parseFloat(c.getString(c.getColumnIndex("RPM")));
                //Log.i(tag, "humanCalculated is " + humanPowerFactor * cranksetEfficiency * (torque - torqueBias) * rpm * 2 * pi / 60);
                //humanPower = Math.max(humanPowerFactor * cranksetEfficiency * (torque - torqueBias) * rpm * 2 * pi / 60, 0);
                humanPower = Math.max(humanPowerFactor * cranksetEfficiency * (torque - 0) * rpm * 2 * pi / 60, 0);

                motorPowerArray[i] = motorPower;
                humanPowerArray[i] = humanPower;

                //format string to datetime
//                Date date = new Date();
//                dateString = c.getString(c.getColumnIndex("time"));
//                try {
//                    date = dateFormat.parse(dateString);
//                    m.put(date, new Float[] {motorPower, humanPower});
//                } catch (ParseException e) {
//                    Log.i(tag, "catch error");
//                    e.printStackTrace();
//                }
//                Log.i(tag, "end of if");
            }
            c.moveToPrevious();
            //count +=1;

        }
        //   Log.i(tag, "outside for");

        bikeDataList = new ArrayList<>();
        bikeDataList.add(motorPowerArray);
        bikeDataList.add(humanPowerArray);

        db.close();

        //return the HashMap object
        return bikeDataList;
    }

    //method to get most recent speed from database - not used for anything currently but could be used as speedometer
    public float getLatestSpeed() {
        float latestSpeed = 0;

        //Get most recent speed data
        SQLiteDatabase db = getWritableDatabase();
        String query = "SELECT * FROM " + TABLE_BIKEDATA + " WHERE 1;";

        //cursor points to a location in results
        Cursor c = db.rawQuery(query, null);

        //Get most recent result
        c.moveToLast();

        //set numIterations above to choose how many results are needed
        if (c.getString(c.getColumnIndex("speed")) != null) {
            latestSpeed = Float.parseFloat(c.getString(c.getColumnIndex("speed")));
        }

        db.close();

        //return the latest speed
        return latestSpeed;
    }



    public SimpleLocation getLatestBikeLocation() {
        SQLiteDatabase db = getWritableDatabase();
        String query = "SELECT * FROM " + TABLE_BIKE_LOCATION + " WHERE 1;";
        float latitude;
        float longitude;
        // Cursor points to a location in results
        Cursor c = db.rawQuery(query, null);

        c.moveToLast();

        if (c.getString(c.getColumnIndex(COLUMN_BIKE_LOCATION_LATITUDE)) != null) {
            latitude = Float.parseFloat(c.getString(c.getColumnIndex(COLUMN_BIKE_LOCATION_LATITUDE)));
        } else {
            return null;
        }

        if (c.getString(c.getColumnIndex(COLUMN_BIKE_LOCATION_LONGITUDE)) != null) {
            longitude = Float.parseFloat(c.getString(c.getColumnIndex(COLUMN_BIKE_LOCATION_LONGITUDE)));
        } else {
            return null;
        }

        SimpleLocation currentLocation = new SimpleLocation(latitude, longitude);

        return currentLocation;
    }


    //get last request sent to bike for control implementation
    public float getLastRequestToBike() {
        float lastRequestToBike = 0;
        int count = 0;

        //Get most recent data
        SQLiteDatabase db = getWritableDatabase();
        String query = "SELECT * FROM " + TABLE_COMMANDSENT + " WHERE 1;";

        //change query to pull back only required data when it is working
        //String query = "SELECT COLUMN_COMMANDSENT FROM " + TABLE_COMMANDSENT + " WHERE 1;";

        //cursor points to a location in results
        Cursor c = db.rawQuery(query, null);

        //Get most recent results
        boolean moveToLastSucceeded = c.moveToLast();


        if (moveToLastSucceeded) {
            //Log.i(tag, "Move to last succeeded");
            while (count < 1) {
                if (c.getString(c.getColumnIndex("commandSent")) != null) {
                    lastRequestToBike = Float.parseFloat(c.getString(c.getColumnIndex("commandSent")));
                }
                count = 1;
            }
        }

        db.close();

        return lastRequestToBike;
    }


}
