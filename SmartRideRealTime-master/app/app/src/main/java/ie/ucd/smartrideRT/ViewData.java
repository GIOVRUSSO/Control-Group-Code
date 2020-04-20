/*
* Class Name: ViewData.java
* Corresponding layout: activity_view_data.xml
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: ViewData prints data from cycle analyst that is saved in database to screen
* */

package ie.ucd.smartrideRT;
import androidx.appcompat.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.widget.EditText;
import android.widget.TextView;



public class ViewData extends AppCompatActivity {

    private static final String tag = "debugging";
    EditText userInput;
    TextView displayData;
    MyDBHandler dbHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        Log.i(tag, "ViewData OnCreate");
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_view_data);

        displayData = (TextView) findViewById(R.id.displayData);
        dbHandler = new MyDBHandler(this, null, null, 1);

        //note that if the database is empty the activity will crash if it tries to print - easily fixed
        printDatabase();

    }

    public void viewButtonClicked(View view){
        printDatabase();
    }


    // Print cycle analyst data to screen - it prints badly, can be easily edited in dbHandler
    public void printDatabase(){
        String dbString = dbHandler.getBikeData();
        displayData.setText(dbString);
        userInput.setText("");
    }


}
