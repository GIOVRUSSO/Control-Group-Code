/*
* Class Name: CommandSentData.java
* Corresponding layout: No
 * Version: 3.0
 * Author: Tom Stanton (Version 2.0: Jingyi Hu, Version 1.0: Shaun Sweeney)
 * Date: January 2020
* Description: CommandSentData is the data type that is used to save data to database when a command has been
* sent to the bike in accordance with the java principle of encapsulation
* */

package ie.ucd.smartrideRT;


public class CommandSentData {
    //private int _id;
    private int _commandSent;


    public CommandSentData(int commandSent){
        this._commandSent = commandSent;
    }

    public int get_commandSent() {
        return _commandSent;
    }

    public void set_commandSent(int _commandSent) {
        this._commandSent = _commandSent;
    }
}
