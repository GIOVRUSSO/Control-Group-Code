/*
* Class Name: CommandSentData.java
* Corresponding layout: No
* Author: Shaun Sweeney - shaun.sweeney@ucdconnect.ie // shaunsweeney12@gmail.com
* Date: March 2017
* Description: CommandSentData is the data type that is used to save data to database when a command has been
* sent to the bike in accordance with the java principle of encapsulation
* */

package ie.ucd.smartrideRT;


public class CommandSentData {
    private int _id;
    private String _commandSent;


    public CommandSentData(String commandSent){
        this._commandSent = commandSent;
    }

    public String get_commandSent() {
        return _commandSent;
    }

    public void set_commandSent(String _commandSent) {
        this._commandSent = _commandSent;
    }
}
