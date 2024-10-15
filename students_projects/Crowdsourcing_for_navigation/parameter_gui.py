# Module for gathering all scripts related to GUI for parameter setting

import sys
from tkinter import *
import tkinter as tk

# Checkbar class for gathering multiple checkboxes
class Checkbar(Frame):
   
   # method for binding different checkboxes
   def disable_enable_binding(self,event):
      if not self.vars[0].get():
         self.chks[1].config(state=tk.ACTIVE)
      else:
         self.chks[1].config(state=tk.DISABLED)

   def __init__(self, parent=None, picks=[], side=LEFT, anchor=W, binding=False):
      Frame.__init__(self, parent)
      self.vars = []
      self.chks = []
      for pick in picks:
         var = IntVar()
         chk = Checkbutton(self, text=pick, variable=var)
         if pick.__contains__('different paths'):
            chk.config(state=tk.DISABLED)
         self.chks.append(chk)
         chk.pack(side=side, anchor=anchor, expand=YES)
         self.vars.append(var)
         if pick.__contains__('selected path'): # enable 'different paths' checkbox only if 'selected path' checkbox is enabled, otherwise disable it
            chk.bind('<Button-1>',self.disable_enable_binding)
            chk.bind('<space>',self.disable_enable_binding)
            chk.bind('<Return>',self.disable_enable_binding)
   
   # method for returning state values of the different checkboxes
   def state(self):
      return map((lambda var: var.get()), self.vars)
   
# Inputbar class for gathering labels and text fields
class Inputbar(Frame):
    def __init__(self, parent=None, picks=[], side=LEFT, anchor=W):
        Frame.__init__(self, parent)
        self.vars = []
        lbl1 = Label(self,text='Number of cars')
        lng = Text(self, height = 1,
                        width = 5,
                        bg = "light yellow")
        lbl2 = Label(self,text='Number of simulation repetitions')
        lng2 = Text(self, height = 1,
                        width = 5,
                        bg = "light yellow")
        lbl1.pack(side=side,anchor=anchor)
        lng.pack(side=side,anchor=anchor)
        lbl2.pack(side=side,anchor=anchor)
        lng2.pack(side=side,anchor=anchor)
        self.vars.append(lng)
        self.vars.append(lng2)
    
    # method for returning state values of the different text fields
    def state(self):
      return map((lambda var: var.get()), self.vars)

# method for writing down the parameter values on a file, inputs are:
# - number of cars, taken from its respective text field,
# - number of times a simulation must be repeated, taken from its respective text field,
# - value collection from the colors checkbox bar,
# - value collection from the audio checkbox
def write_params_on_file(number_of_cars,number_of_repeats,v,audio):
    colorsel = v[0]==1
    colorpaths = v[1]==1
    colorreward = v[2]==1
    useaudio = audio[0]==1    
    f = open('sim_config.txt','w')
    f.write(str(number_of_cars)+';'+str(number_of_repeats)+';'+str(colorsel)+';'+str(colorpaths)+';'+str(colorreward)+';'+str(useaudio))
    f.close()

# method for building the GUI for parameters and displaying it
def gui_create_and_run():
   root = Tk()
   simdet = LabelFrame(root,text='Simulation details') # create GUI window
   simdet.grid(row=0,column=0,padx=0,pady=0)
   # prepare labels and text fields
   l1 = Label(simdet,text='Number of cars')
   l1.grid(row=0,column=0,padx=2,pady=10)
   t1 = Entry(simdet,width=20)
   t1.grid(row=0,column=1,padx=2,pady=10)
   l2 = Label(simdet,text='Number of simulation repeats')
   l2.grid(row=1,column=0,padx=2,pady=10)
   t2 = Entry(simdet,width=20)
   t2.grid(row=1,column=1,padx=2,pady=10)
   # prepare checkboxes for colors and audio
   tgl = Checkbar(simdet, ['Show colour of selected path (suggested for HiL)','Show colours for different paths (suggested for paths analysis)','Show cost map colours (suggested for reward analysis)'],side=TOP,binding=True)
   tgl.grid(row=2)
   tgl2 = Checkbar(simdet, ['Use audio guide'],side=TOP)
   tgl2.grid(row=3)
   # method for writing down the parameters as soon as 'Start Simulation' is pressed
   def allstates(): 
      v = list(tgl.state())
      number_of_cars = int(t1.get())
      number_of_repeats = int(t2.get())
      audio = list(tgl2.state())
      write_params_on_file(number_of_cars,number_of_repeats,v,audio)
      root.destroy()
   # method for quitting application as soon as the 'Quit' button is pressed
   def allbreak():
       root.destroy()
       sys.exit(0)
   # prepare buttons
   Button(simdet, text='Start Simulations', command=allstates).grid(row=4,column=0)
   Button(simdet, text='Quit', command=allbreak).grid(row=4,column=1)
   root.mainloop() # display everything
   
if __name__ == '__main__':
    gui_create_and_run()