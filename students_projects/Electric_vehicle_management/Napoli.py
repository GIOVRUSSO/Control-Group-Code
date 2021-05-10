from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import numpy as np
import matplotlib.pyplot as plt
import random
from pulp import*
from gekko import GEKKO

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

#Funzione che risolve il problema di Ottimizzazione non intero
def Ottimizzazione():

    prob=LpProblem("Ottimizzazione",LpMaximize)
    x1=LpVariable("x1",lowBound=0)
    x2=LpVariable("x2",lowBound=0)
    x3=LpVariable("x3",lowBound=0)
    x4=LpVariable("x4",lowBound=0)
    x5=LpVariable("x5",lowBound=0)
    x6=LpVariable("x6",lowBound=0)
    x7=LpVariable("x7",lowBound=0)
    x8=LpVariable("x8",lowBound=0)
    x9=LpVariable("x9",lowBound=0)
    x10=LpVariable("x10",lowBound=0)
    x11=LpVariable("x11",lowBound=0)
    x12=LpVariable("x12",lowBound=0)
    x13=LpVariable("x13",lowBound=0)
    x14=LpVariable("x14",lowBound=0)

    prob += 0.1*230*x1 + 0.1*178*x2 + 0.1*157*x3 + 0.3*68*x4 + 0.4*54*x5 + 0.5*277*x6 + 0.9*70*x7 + 79*x8 + 22*x9 + 9*x10 + 27*x11 + 38*x12 + 0.3*714*x13 + 0.1*192*x14
    prob += 230*x1 + 178*x2 + 157*x3 + 68*x4 + 54*x5 + 277*x6 + 70*x7 + 79*x8 + 22*x8 + 9*x10 + 27*x11 + 38*x12 + 714*x13 + 192*x14 <= 1200
    prob+= x1 <= 1
    prob+= x2 <= 1
    prob+= x3 <= 1
    prob+= x4 <= 1
    prob+= x5 <= 1
    prob+= x6 <= 1
    prob+= x7 <= 1
    prob+= x8 <= 1
    prob+= x9 <= 1
    prob+= x10 <= 1
    prob+= x11 <= 1
    prob+= x12 <= 1
    prob+= x13 <= 1
    prob+= x14 <= 1

    print(prob)

    status = prob.solve()
    LpStatus[status]
    return(value(x1),value(x2),value(x3),value(x4),value(x5),value(x6),value(x7),value(x8),value(x9),value(x10),value(x11),value(x12),value(x13),value(x14))

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO
def Ottimizzazione_Gekko_per_Veicolo_Elettrico():
    m=GEKKO()

    m.options.SOLVER=1

    x1 = m.Var(value=0, lb=0, ub=1, integer=True)
    x2 = m.Var(value=0, lb=0, ub=1, integer=True)
    x3 = m.Var(value=0, lb=0, ub=1, integer=True)
    x4 = m.Var(value=0, lb=0, ub=1, integer=True)
    x5 = m.Var(value=0, lb=0, ub=1, integer=True)
    x6 = m.Var(value=0, lb=0, ub=1, integer=True)
    x7 = m.Var(value=0, lb=0, ub=1, integer=True)
    x8 = m.Var(value=0, lb=0, ub=1, integer=True)
    x9 = m.Var(value=0, lb=0, ub=1, integer=True)
    x10 = m.Var(value=0, lb=0, ub=1, integer=True)
    x11 = m.Var(value=0, lb=0, ub=1, integer=True)
    x12 = m.Var(value=0, lb=0, ub=1, integer=True)
    x13 = m.Var(value=0, lb=0, ub=1, integer=True)
    x14 = m.Var(value=0, lb=0, ub=1, integer=True)
    # Equations
    #m.Equation(230*x1 + 178*x2 + 157*x3 + 68*x4 + 54*x5 + 277*x6 + 70*x7 + 79*x8 + 22*x9 + 9*x10 + 27*x11 + 38*x12 + 714*x13 + 192*x14 <= 1200)
    m.Equation(100-(230/150*10*x1)-(178/150*10*x2)-(157/150*10*x3) - (68/150*x4*10) - (54/150*10*x5) - (277/150*10*x6) - (70/150*10*x7) - (79/150*10*x8) - (22/150*x9*10) - (9/150*x10*10) - (27/150*x11*10) - (38/150*x12*10) - (714/150*x13*10) - (192/150*x14*10) >= 35)
    #Objective
    m.Obj(-(0.1*230*x1 + 0.1*178*x2 + 0.1*157*x3 + 0.3*68*x4 + 0.4*54*x5 + 0.5*277*x6 + 0.9*70*x7 + 79*x8 + 22*x9 + 9*x10 + 27*x11 + 38*x12 + 0.3*714*x13 + 0.1*192*x14)) # Objective
    m.solve() # Solve
    #print("x1: " + str(x1.value))
    #print("x2: " + str(x2.value))
    #print("x3: " + str(x3.value))
    #print("x4: " + str(x4.value))
    #print("x5: " + str(x5.value))
    #print("x6: " + str(x6.value))
    #print("x7: " + str(x7.value))
    #print("x8: " + str(x8.value))
    #print("x9: " + str(x9.value))
    #print("x10: " + str(x10.value))
    #print("x11: " + str(x11.value))
    #print("x12: " + str(x12.value))
    #print("x13: " + str(x13.value))
    #print("x14: " + str(x14.value))
    #print("Objective: " + str(m.options.objfcnval))
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value,x13.value,x14.value)
    return Variabili


#Funzione che simula l'evoluzione dello stato di carica della Batteria di un veicolo del flusso 1
def Evoluzione_Batteria_1(Stato_Batteria_1,Distanza_Percorsa_1):
    distanza=Distanza_Percorsa_1[len(Distanza_Percorsa_1)-1]-Distanza_Percorsa_1[len(Distanza_Percorsa_1)-2]
    Stato_Batteria_1 = Stato_Batteria_1 - ((distanza/150)*10)
    return Stato_Batteria_1

#Funzione che simula l'evoluzione dello stato di carica della Batteria
def Evoluzione_Batteria(Stato_Batteria,Distanza_Percorsa):
    distanza=Distanza_Percorsa[len(Distanza_Percorsa)-1]-Distanza_Percorsa[len(Distanza_Percorsa)-2]
    Stato_Batteria = Stato_Batteria - ((distanza/150)*10)
    return Stato_Batteria

#Funzione per controllare i veicoli del flusso 12
def Controllo_Flusso_12(Strade_Elettriche_Flusso_12,Strade_Non_Elettriche_Flusso_12,Flusso_12,step,partenze12,arrivi12):
    i=0
    for i in range(0,30):
        if step>=partenze12[i] and step < arrivi12[i]:
            if traci.vehicle.getRoadID(Flusso_12[i]) in Strade_Elettriche_Flusso_12:
                traci.vehicle.setColor(Flusso_12[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_12[i]) in Strade_Non_Elettriche_Flusso_12:
                traci.vehicle.setColor(Flusso_12[i],(255,0,0))

#Funzione per controllare i veicoli del flusso 10
def Controllo_Flusso_10(Strade_Elettriche_Flusso_10,Strade_Non_Elettriche_Flusso_10,Flusso_10,step,partenze10,arrivi10):
    i=0
    for i in range(0,400):
        if step>=partenze10[i] and step < arrivi10[i]:
            if traci.vehicle.getRoadID(Flusso_10[i]) in Strade_Elettriche_Flusso_10:
                traci.vehicle.setColor(Flusso_10[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_10[i]) in Strade_Non_Elettriche_Flusso_10:
                traci.vehicle.setColor(Flusso_10[i],(255,0,0))

#Funzione per controllare i veicoli del flusso 11
def Controllo_Flusso_11(Strade_Elettriche_Flusso_11,Strade_Non_Elettriche_Flusso_11,Flusso_11,step,partenze11,arrivi11):
    i=0
    for i in range(0,30):
        if step>=partenze11[i] and step < arrivi11[i]:
            if traci.vehicle.getRoadID(Flusso_11[i]) in Strade_Elettriche_Flusso_11:
                traci.vehicle.setColor(Flusso_11[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_11[i]) in Strade_Non_Elettriche_Flusso_11:
                traci.vehicle.setColor(Flusso_11[i],(255,0,0))

#Funzione per controllare i veicoli del flusso 13
def Controllo_Flusso_13(Strade_Elettriche_Flusso_13,Strade_Non_Elettriche_Flusso_13,Flusso_13,step,partenze13,arrivi13):
    i=0
    for i in range(0,90):
        if step>=partenze13[i] and step < arrivi13[i]:
            if traci.vehicle.getRoadID(Flusso_13[i]) in Strade_Elettriche_Flusso_13:
                traci.vehicle.setColor(Flusso_13[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_13[i]) in Strade_Non_Elettriche_Flusso_13:
                traci.vehicle.setColor(Flusso_13[i],(255,0,0))

#Funzione per controllare i veicoli del flusso 4
def Controllo_Flusso_4(Strade_Elettriche_Flusso_4,Strade_Non_Elettriche_Flusso_4,Flusso_4,step,partenze4,arrivi4):
    i=0
    for i in range(0,306):
        if step>=partenze4[i] and step < arrivi4[i]:
            if traci.vehicle.getRoadID(Flusso_4[i]) in Strade_Elettriche_Flusso_4:
                traci.vehicle.setColor(Flusso_4[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_4[i]) in Strade_Non_Elettriche_Flusso_4:
                traci.vehicle.setColor(Flusso_4[i],(255,0,0))
                
#Funzione per controllare i veicoli del flusso 3
def Controllo_Flusso_3(Strade_Elettriche_Flusso_3,Strade_Non_Elettriche_Flusso_3,Flusso_3,step,partenze3,arrivi3):
    i=0
    for i in range(0,255):
        if step>=partenze3[i] and step < arrivi3[i]:
            if traci.vehicle.getRoadID(Flusso_3[i]) in Strade_Elettriche_Flusso_3:
                traci.vehicle.setColor(Flusso_3[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_3[i]) in Strade_Non_Elettriche_Flusso_3:
                traci.vehicle.setColor(Flusso_3[i],(255,0,0))

#Funzione per controllare i veicoli del flusso 2
def Controllo_Flusso_2(Strade_Elettriche_Flusso_2,Strade_Non_Elettriche_Flusso_2,Flusso_2,step,partenze2,arrivi2):
    i=0
    for i in range(0,265):
        if step>=partenze2[i] and step < arrivi2[i]:
            if traci.vehicle.getRoadID(Flusso_2[i]) in Strade_Elettriche_Flusso_2:
                traci.vehicle.setColor(Flusso_2[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_2[i]) in Strade_Non_Elettriche_Flusso_2:
                traci.vehicle.setColor(Flusso_2[i],(255,0,0))

#Funzione per controllare i veicoli del flusso 1
def Controllo_Flusso_1(Strade_Elettriche_Flusso_1,Strade_Non_Elettriche_Flusso_1,Flusso_1,step,partenze1,arrivi1):
    i=0
    for i in range(0,300):
        if step>=partenze1[i] and step < arrivi1[i]:
            if traci.vehicle.getRoadID(Flusso_1[i]) in Strade_Elettriche_Flusso_1:
                traci.vehicle.setColor(Flusso_1[i],(0,255,0))
            if traci.vehicle.getRoadID(Flusso_1[i]) in Strade_Non_Elettriche_Flusso_1:
                traci.vehicle.setColor(Flusso_1[i],(255,0,0))                    
            
def run():
    """execute the TraCI control loop"""
    step = 0
    
    #Definizione delle strade in cui va inserita la modalità elettrica ottenuta dall'ottimizzazione offline
    Strade_Elettriche_Flusso_1=["Via_Foria_6","Via_Foria_7","Rotonda_1","Rotonda_2","Rotonda_3","Calata_Capodichino_1","SantAntonio_AbateTerzo","gneE32","gneE31"]
    Strade_Non_Elettriche_Flusso_1=["Via_Foria_5","Via_Foria_0","Via_Foria_1","Via_Foria_2","Via_Foria_3","Via_Foria_4","Calata_Capodichino"]
    Strade_Elettriche_Flusso_2=["Via_Veterinaia_1","Via_Veterinaia_2","Via_Foria_3","Via_Foria_4","Via_Foria_5","Via_Foria_6","Via_Foria_7","Rotonda_1","Rotonda_2","Rotonda_3","Rotonda_4","Rotonda_5","gneE129","gneE181"]
    Strade_Non_Elettriche_Flusso_2=["Vico_SantaMaria","Via_Foria_0","Via_Foria_1","Via_Foria_2","-gneE226","-gneE182","-gneE231","gneE184","gneE228","Via_Miano","gneE159"]
    Strade_Elettriche_Flusso_3=["SantAntonio_AbateTerzo","gneE32","gneE31","Via_Foria_5","Via_Foria_6","Via_Foria_7","Rotonda_1","Rotonda_2","Rotonda_4","Via_Nuova_Poggioreale_5","Via_Nuova_Poggioreale_4"]
    Strade_Non_Elettriche_Flusso_3=["Via_Foria_0","Via_Foria_1","Via_Foria_2","Via_Foria_3","Via_Foria_4","Rotonda_3","Via_Nuova_Poggioreale_3","Via_Nuova_Poggioreale_2","Via_Nuova_Poggioreale_1","Via_Nuova_Poggioreale"]
    Strade_Elettriche_Flusso_4=["-Corso_Meridionale_1","-Via_Taddeo_Sessa_1","-Via_Taddeo_Sessa"]
    Strade_Non_Elettriche_Flusso_4=["gneE8","Salita_Pontenuovo_1","Salita_Pontenuovo_2","gneE210","gneE2","Via_Foria_0","Via_Carbonara","-Corso_Meridionale","-via_Cesare_Rosaroli_10","-via_Cesare_Rosaroli","-gneE211","Vico_Tutti_i_Santi","Corso_Garibaldi_10","Corso_Garibaldi_20"]
    #Strade_Elettriche_Flusso_5=["-gneE2","Corso_Meridionale_1","Via_Taddeo_Sessa_1","Via_Taddeo_Sessa"]
    #Strade_Non_Elettriche_Flusso_5=["Via_Foria_00","-Via_Carbonara","Corso_Meridionale"]
    #Strade_Elettriche_Flusso_6=["-gneE83","-Via_Nuova_Poggioreale_4"]
    #Strade_Non_Elettriche_Flusso_6=["Via_Taddeo_Sessa","-Via_Nuova_Poggioreale_1","-Via_Nuova_Poggioreale_2","-Via_Nuova_Poggioreale_3","-gneE144","-gneE130","-gneE227","Via_Santa_Maria_ai_monti","Calata_Capodichino"]
    #Strade_Elettriche_Flusso_7=["gneE2","gneE96","Rotondone_3","Rotondone_4","gneE234"]
    #Strade_Non_Elettriche_Flusso_7=["Corso_Arnaldo_Lucci","-Corso_Meridionale_1","-Corso_Meridionale","Via_Nuova_Poggioreale_1","Via_Nuova_Poggioreale"]
    #Strade_Elettriche_Flusso_8=["Corso_Garibaldi_2","Corso_Garibaldi_1","Corso_Garibaldi","gneE193","Rotonda_3","Rotonda_4","gneE2"]
    #Strade_Non_Elettriche_Flusso_8=["Corso_Arnaldo_Lucci","Calata_Capodichino_1","Calata_Capodichino"]
    #Strade_Elettriche_Flusso_9=["Corso_Garibaldi_1","Corso_Garibaldi","gneE193","Rotonda_3","Rotonda_4","Rotonda_5","gneE129","gneE181","Via_Miano"]
    #Strade_Non_Elettriche_Flusso_9=["Corso_Arnaldo_Lucci","gneE2","Corso_Garibaldi_2","-gneE226","gneE159"]
    Strade_Elettriche_Flusso_10=["Rotonda_3","Rotonda_4","-gneE33","Rotonda_5","Rotonda_6","Rotonda_7","Rotonda_1","-gneE31","-gneE32","Rotondone_6","Rotondone_7","-gneE65","Rotondone_3","Rotondone_4","Rotondone_5","Via_Taddeo_Sessa","Via_Taddeo_Sessa_1","Corso_Meridionale_1"]
    Strade_Non_Elettriche_Flusso_10=["gneE210","Corso_Garibaldi_10","-gneE83","-Via_Nuova_Poggioreale_5","-Via_Nuova_Poggioreale_4","Corso_Garibaldi_2","-Via_Nuova_Poggioreale_3","gneE101","-Via_Nuova_Poggioreale_2","gneE230","416695585","-Via_Nuova_Poggioreale_1","gneE102","gneE7","gneE96","Rotondone_1","gneE100","Corso_Meridionale","-Corso_Arnaldo_Lucci","-gneE2"]
    Strade_Elettriche_Flusso_11=["Via_Foria_40","Via_Foria_30","-gneE62","-gneE181","-gneE129","Rotonda_6","Rotonda_7","Via_Foria_70","Via_Foria_60","Via_Foria_50"]
    Strade_Non_Elettriche_Flusso_11=["-Via_Santa_Maria_ai_monti","Via_Foria_20","Via_Foria_10","Via_Foria_00","-Calata_Capodichino","gneE227"]
    Strade_Elettriche_Flusso_12=["Via_Nuova_Poggioreale_4","-gneE118","-Via_Taddeo_Sessa_1","gneE144","gneE233","Via_Nuova_Poggioreale_3","Via_Nuova_Poggioreale_1","-Via_Taddeo_Sessa"]
    Strade_Non_Elettriche_Flusso_12=["-Calata_Capodichino","gneE83","gneE65","-gneE103","-Via_Santa_Maria_ai_monti","gneE227","gneE130","Via_Nuova_Poggioreale_2","-Corso_Meridionale"]
    Strade_Elettriche_Flusso_13=["-Via_Nuova_Poggioreale","-Via_Nuova_Poggioreale_2","-Via_Nuova_Poggioreale_3","-Via_Nuova_Poggioreale_4","-Via_Nuova_Poggioreale_5","-gneE144","-gneE227","-gneE130","-gneE233"]
    Strade_Non_Elettriche_Flusso_13=["-Via_Nuova_Poggioreale_1","Calata_Capodichino","Calata_Capodichino_1","Via_Santa_Maria_ai_monti",]
    #Strade_Elettriche_Flusso_14=["-Via_Nuova_Poggioreale","-Via_Nuova_Poggioreale_4","-gneE144","-gneE130","gneE197","Via_Miano"]
    #Strade_Non_Elettriche_Flusso_14=["-Via_Nuova_Poggioreale_1","-Via_Nuova_Poggioreale_2","-Via_Nuova_Poggioreale_3","-gneE227"]

    #Vettori per l'ottimizzazione di "Veicolo Elettrico"
    valori=[]
    Strada_Prevista=[]
    Distanza_Percorsa=[]
    
    #Definizione dei flussi per passarli poi nel ciclo for 
    Flusso_1=["flusso1.0","flusso1.1","flusso1.2","flusso1.3","flusso1.4","flusso1.5","flusso1.6","flusso1.7","flusso1.8","flusso1.9","flusso1.10","flusso1.11","flusso1.12","flusso1.13","flusso1.14","flusso1.15","flusso1.16","flusso1.17","flusso1.18","flusso1.19","flusso1.20","flusso1.21","flusso1.22","flusso1.23","flusso1.24","flusso1.25","flusso1.26","flusso1.27","flusso1.28","flusso1.29","flusso1.30","flusso1.31","flusso1.32","flusso1.33","flusso1.34","flusso1.35","flusso1.36","flusso1.37","flusso1.38","flusso1.39","flusso1.40","flusso1.41","flusso1.42","flusso1.43","flusso1.44","flusso1.45","flusso1.46","flusso1.47","flusso1.48","flusso1.49","flusso1.50","flusso1.51","flusso1.52","flusso1.53","flusso1.54","flusso1.55","flusso1.56","flusso1.57","flusso1.58","flusso1.59","flusso1.60","flusso1.61","flusso1.62","flusso1.63","flusso1.64","flusso1.65","flusso1.66","flusso1.67","flusso1.68","flusso1.69","flusso1.70","flusso1.71","flusso1.72","flusso1.73","flusso1.74","flusso1.75","flusso1.76","flusso1.77","flusso1.78","flusso1.79","flusso1.80","flusso1.81","flusso1.82","flusso1.83","flusso1.84","flusso1.85","flusso1.86","flusso1.87","flusso1.88","flusso1.89","flusso1.90","flusso1.91","flusso1.92","flusso1.93","flusso1.94","flusso1.95","flusso1.96","flusso1.97","flusso1.98","flusso1.99","flusso1.100","flusso1.101","flusso1.102","flusso1.103","flusso1.104","flusso1.105","flusso1.106","flusso1.107","flusso1.108","flusso1.109","flusso1.110","flusso1.111","flusso1.112","flusso1.113","flusso1.114","flusso1.115","flusso1.116","flusso1.117","flusso1.118","flusso1.119","flusso1.120","flusso1.121","flusso1.122","flusso1.123","flusso1.124","flusso1.125","flusso1.126","flusso1.127","flusso1.128","flusso1.129","flusso1.130","flusso1.131","flusso1.132","flusso1.133","flusso1.134","flusso1.135","flusso1.136","flusso1.137","flusso1.138","flusso1.139","flusso1.140","flusso1.141","flusso1.142","flusso1.143","flusso1.144","flusso1.145","flusso1.146","flusso1.147","flusso1.148","flusso1.149","flusso1.150","flusso1.151","flusso1.152","flusso1.153","flusso1.154","flusso1.155","flusso1.156","flusso1.157","flusso1.158","flusso1.159","flusso1.160","flusso1.161","flusso1.162","flusso1.163","flusso1.164","flusso1.165","flusso1.166","flusso1.167","flusso1.168","flusso1.169","flusso1.170","flusso1.171","flusso1.172","flusso1.173","flusso1.174","flusso1.175","flusso1.176","flusso1.177","flusso1.178","flusso1.179","flusso1.180","flusso1.181","flusso1.182","flusso1.183","flusso1.184","flusso1.185","flusso1.186","flusso1.187","flusso1.188","flusso1.189","flusso1.190","flusso1.191","flusso1.192","flusso1.193","flusso1.194","flusso1.195","flusso1.196","flusso1.197","flusso1.198","flusso1.199","flusso1.200","flusso1.201","flusso1.202","flusso1.203","flusso1.204","flusso1.205","flusso1.206","flusso1.207","flusso1.208","flusso1.209","flusso1.210","flusso1.211","flusso1.212","flusso1.213","flusso1.214","flusso1.215","flusso1.216","flusso1.217","flusso1.218","flusso1.219","flusso1.220","flusso1.221","flusso1.222","flusso1.223","flusso1.224","flusso1.225","flusso1.226","flusso1.227","flusso1.228","flusso1.229","flusso1.230","flusso1.231","flusso1.232","flusso1.233","flusso1.234","flusso1.235","flusso1.236","flusso1.237","flusso1.238","flusso1.239","flusso1.240","flusso1.241","flusso1.242","flusso1.243","flusso1.244","flusso1.245","flusso1.246","flusso1.247","flusso1.248","flusso1.249","flusso1.250","flusso1.251","flusso1.252","flusso1.253","flusso1.254","flusso1.255","flusso1.256","flusso1.257","flusso1.258","flusso1.259","flusso1.260","flusso1.261","flusso1.262","flusso1.263","flusso1.264","flusso1.265","flusso1.266","flusso1.267","flusso1.268","flusso1.269","flusso1.270","flusso1.271","flusso1.272","flusso1.273","flusso1.274","flusso1.275","flusso1.276","flusso1.277","flusso1.278","flusso1.279","flusso1.280","flusso1.281","flusso1.282","flusso1.283","flusso1.284","flusso1.285","flusso1.286","flusso1.287","flusso1.288","flusso1.289","flusso1.290","flusso1.291","flusso1.292","flusso1.293","flusso1.294","flusso1.295","flusso1.296","flusso1.297","flusso1.298","flusso1.299"]
    Flusso_2=["flusso2.0","flusso2.1","flusso2.2","flusso2.3","flusso2.4","flusso2.5","flusso2.6","flusso2.7","flusso2.8","flusso2.9","flusso2.10","flusso2.11","flusso2.12","flusso2.13","flusso2.14","flusso2.15","flusso2.16","flusso2.17","flusso2.18","flusso2.19","flusso2.20","flusso2.21","flusso2.22","flusso2.23","flusso2.24","flusso2.25","flusso2.26","flusso2.27","flusso2.28","flusso2.29","flusso2.30","flusso2.31","flusso2.32","flusso2.33","flusso2.34","flusso2.35","flusso2.36","flusso2.37","flusso2.38","flusso2.39","flusso2.40","flusso2.41","flusso2.42","flusso2.43","flusso2.44","flusso2.45","flusso2.46","flusso2.47","flusso2.48","flusso2.49","flusso2.50","flusso2.51","flusso2.52","flusso2.53","flusso2.54","flusso2.55","flusso2.56","flusso2.57","flusso2.58","flusso2.59","flusso2.60","flusso2.61","flusso2.62","flusso2.63","flusso2.64","flusso2.65","flusso2.66","flusso2.67","flusso2.68","flusso2.69","flusso2.70","flusso2.71","flusso2.72","flusso2.73","flusso2.74","flusso2.75","flusso2.76","flusso2.77","flusso2.78","flusso2.79","flusso2.80","flusso2.81","flusso2.82","flusso2.83","flusso2.84","flusso2.85","flusso2.86","flusso2.87","flusso2.88","flusso2.89","flusso2.90","flusso2.91","flusso2.92","flusso2.93","flusso2.94","flusso2.95","flusso2.96","flusso2.97","flusso2.98","flusso2.99","flusso2.100","flusso2.101","flusso2.102","flusso2.103","flusso2.104","flusso2.105","flusso2.106","flusso2.107","flusso2.108","flusso2.109","flusso2.110","flusso2.111","flusso2.112","flusso2.113","flusso2.114","flusso2.115","flusso2.116","flusso2.117","flusso2.118","flusso2.119","flusso2.120","flusso2.121","flusso2.122","flusso2.123","flusso2.124","flusso2.125","flusso2.126","flusso2.127","flusso2.128","flusso2.129","flusso2.130","flusso2.131","flusso2.132","flusso2.133","flusso2.134","flusso2.135","flusso2.136","flusso2.137","flusso2.138","flusso2.139","flusso2.140","flusso2.141","flusso2.142","flusso2.143","flusso2.144","flusso2.145","flusso2.146","flusso2.147","flusso2.148","flusso2.149","flusso2.150","flusso2.151","flusso2.152","flusso2.153","flusso2.154","flusso2.155","flusso2.156","flusso2.157","flusso2.158","flusso2.159","flusso2.160","flusso2.161","flusso2.162","flusso2.163","flusso2.164","flusso2.165","flusso2.166","flusso2.167","flusso2.168","flusso2.169","flusso2.170","flusso2.171","flusso2.172","flusso2.173","flusso2.174","flusso2.175","flusso2.176","flusso2.177","flusso2.178","flusso2.179","flusso2.180","flusso2.181","flusso2.182","flusso2.183","flusso2.184","flusso2.185","flusso2.186","flusso2.187","flusso2.188","flusso2.189","flusso2.190","flusso2.191","flusso2.192","flusso2.193","flusso2.194","flusso2.195","flusso2.196","flusso2.197","flusso2.198","flusso2.199","flusso2.200","flusso2.201","flusso2.202","flusso2.203","flusso2.204","flusso2.205","flusso2.206","flusso2.207","flusso2.208","flusso2.209","flusso2.210","flusso2.211","flusso2.212","flusso2.213","flusso2.214","flusso2.215","flusso2.216","flusso2.217","flusso2.218","flusso2.219","flusso2.220","flusso2.221","flusso2.222","flusso2.223","flusso2.224","flusso2.225","flusso2.226","flusso2.227","flusso2.228","flusso2.229","flusso2.230","flusso2.231","flusso2.232","flusso2.233","flusso2.234","flusso2.235","flusso2.236","flusso2.237","flusso2.238","flusso2.239","flusso2.240","flusso2.241","flusso2.242","flusso2.243","flusso2.244","flusso2.245","flusso2.246","flusso2.247","flusso2.248","flusso2.249","flusso2.250","flusso2.251","flusso2.252","flusso2.253","flusso2.254","flusso2.255","flusso2.256","flusso2.257","flusso2.258","flusso2.259","flusso2.260","flusso2.261","flusso2.262","flusso2.263","flusso2.264"]
    Flusso_3=["flusso3.0","flusso3.1","flusso3.2","flusso3.3","flusso3.4","flusso3.5","flusso3.6","flusso3.7","flusso3.8","flusso3.9","flusso3.10","flusso3.11","flusso3.12","flusso3.13","flusso3.14","flusso3.15","flusso3.16","flusso3.17","flusso3.18","flusso3.19","flusso3.20","flusso3.21","flusso3.22","flusso3.23","flusso3.24","flusso3.25","flusso3.26","flusso3.27","flusso3.28","flusso3.29","flusso3.30","flusso3.31","flusso3.32","flusso3.33","flusso3.34","flusso3.35","flusso3.36","flusso3.37","flusso3.38","flusso3.39","flusso3.40","flusso3.41","flusso3.42","flusso3.43","flusso3.44","flusso3.45","flusso3.46","flusso3.47","flusso3.48","flusso3.49","flusso3.50","flusso3.51","flusso3.52","flusso3.53","flusso3.54","flusso3.55","flusso3.56","flusso3.57","flusso3.58","flusso3.59","flusso3.60","flusso3.61","flusso3.62","flusso3.63","flusso3.64","flusso3.65","flusso3.66","flusso3.67","flusso3.68","flusso3.69","flusso3.70","flusso3.71","flusso3.72","flusso3.73","flusso3.74","flusso3.75","flusso3.76","flusso3.77","flusso3.78","flusso3.79","flusso3.80","flusso3.81","flusso3.82","flusso3.83","flusso3.84","flusso3.85","flusso3.86","flusso3.87","flusso3.88","flusso3.89","flusso3.90","flusso3.91","flusso3.92","flusso3.93","flusso3.94","flusso3.95","flusso3.96","flusso3.97","flusso3.98","flusso3.99","flusso3.100","flusso3.101","flusso3.102","flusso3.103","flusso3.104","flusso3.105","flusso3.106","flusso3.107","flusso3.108","flusso3.109","flusso3.110","flusso3.111","flusso3.112","flusso3.113","flusso3.114","flusso3.115","flusso3.116","flusso3.117","flusso3.118","flusso3.119","flusso3.120","flusso3.121","flusso3.122","flusso3.123","flusso3.124","flusso3.125","flusso3.126","flusso3.127","flusso3.128","flusso3.129","flusso3.130","flusso3.131","flusso3.132","flusso3.133","flusso3.134","flusso3.135","flusso3.136","flusso3.137","flusso3.138","flusso3.139","flusso3.140","flusso3.141","flusso3.142","flusso3.143","flusso3.144","flusso3.145","flusso3.146","flusso3.147","flusso3.148","flusso3.149","flusso3.150","flusso3.151","flusso3.152","flusso3.153","flusso3.154","flusso3.155","flusso3.156","flusso3.157","flusso3.158","flusso3.159","flusso3.160","flusso3.161","flusso3.162","flusso3.163","flusso3.164","flusso3.165","flusso3.166","flusso3.167","flusso3.168","flusso3.169","flusso3.170","flusso3.171","flusso3.172","flusso3.173","flusso3.174","flusso3.175","flusso3.176","flusso3.177","flusso3.178","flusso3.179","flusso3.180","flusso3.181","flusso3.182","flusso3.183","flusso3.184","flusso3.185","flusso3.186","flusso3.187","flusso3.188","flusso3.189","flusso3.190","flusso3.191","flusso3.192","flusso3.193","flusso3.194","flusso3.195","flusso3.196","flusso3.197","flusso3.198","flusso3.199","flusso3.200","flusso3.201","flusso3.202","flusso3.203","flusso3.204","flusso3.205","flusso3.206","flusso3.207","flusso3.208","flusso3.209","flusso3.210","flusso3.211","flusso3.212","flusso3.213","flusso3.214","flusso3.215","flusso3.216","flusso3.217","flusso3.218","flusso3.219","flusso3.220","flusso3.221","flusso3.222","flusso3.223","flusso3.224","flusso3.225","flusso3.226","flusso3.227","flusso3.228","flusso3.229","flusso3.230","flusso3.231","flusso3.232","flusso3.233","flusso3.234","flusso3.235","flusso3.236","flusso3.237","flusso3.238","flusso3.239","flusso3.240","flusso3.241","flusso3.242","flusso3.243","flusso3.244","flusso3.245","flusso3.246","flusso3.247","flusso3.248","flusso3.249","flusso3.250","flusso3.251","flusso3.252","flusso3.253","flusso3.254","flusso3.255"]
    Flusso_4=["flusso4.0","flusso4.1","flusso4.2","flusso4.3","flusso4.4","flusso4.5","flusso4.6","flusso4.7","flusso4.8","flusso4.9","flusso4.10","flusso4.11","flusso4.12","flusso4.13","flusso4.14","flusso4.15","flusso4.16","flusso4.17","flusso4.18","flusso4.19","flusso4.20","flusso4.21","flusso4.22","flusso4.23","flusso4.24","flusso4.25","flusso4.26","flusso4.27","flusso4.28","flusso4.29","flusso4.30","flusso4.31","flusso4.32","flusso4.33","flusso4.34","flusso4.35","flusso4.36","flusso4.37","flusso4.38","flusso4.39","flusso4.40","flusso4.41","flusso4.42","flusso4.43","flusso4.44","flusso4.45","flusso4.46","flusso4.47","flusso4.48","flusso4.49","flusso4.50","flusso4.51","flusso4.52","flusso4.53","flusso4.54","flusso4.55","flusso4.56","flusso4.57","flusso4.58","flusso4.59","flusso4.60","flusso4.61","flusso4.62","flusso4.63","flusso4.64","flusso4.65","flusso4.66","flusso4.67","flusso4.68","flusso4.69","flusso4.70","flusso4.71","flusso4.72","flusso4.73","flusso4.74","flusso4.75","flusso4.76","flusso4.77","flusso4.78","flusso4.79","flusso4.80","flusso4.81","flusso4.82","flusso4.83","flusso4.84","flusso4.85","flusso4.86","flusso4.87","flusso4.88","flusso4.89","flusso4.90","flusso4.91","flusso4.92","flusso4.93","flusso4.94","flusso4.95","flusso4.96","flusso4.97","flusso4.98","flusso4.99","flusso4.100","flusso4.101","flusso4.102","flusso4.103","flusso4.104","flusso4.105","flusso4.106","flusso4.107","flusso4.108","flusso4.109","flusso4.110","flusso4.111","flusso4.112","flusso4.113","flusso4.114","flusso4.115","flusso4.116","flusso4.117","flusso4.118","flusso4.119","flusso4.120","flusso4.121","flusso4.122","flusso4.123","flusso4.124","flusso4.125","flusso4.126","flusso4.127","flusso4.128","flusso4.129","flusso4.130","flusso4.131","flusso4.132","flusso4.133","flusso4.134","flusso4.135","flusso4.136","flusso4.137","flusso4.138","flusso4.139","flusso4.140","flusso4.141","flusso4.142","flusso4.143","flusso4.144","flusso4.145","flusso4.146","flusso4.147","flusso4.148","flusso4.149","flusso4.150","flusso4.151","flusso4.152","flusso4.153","flusso4.154","flusso4.155","flusso4.156","flusso4.157","flusso4.158","flusso4.159","flusso4.160","flusso4.161","flusso4.162","flusso4.163","flusso4.164","flusso4.165","flusso4.166","flusso4.167","flusso4.168","flusso4.169","flusso4.170","flusso4.171","flusso4.172","flusso4.173","flusso4.174","flusso4.175","flusso4.176","flusso4.177","flusso4.178","flusso4.179","flusso4.180","flusso4.181","flusso4.182","flusso4.183","flusso4.184","flusso4.185","flusso4.186","flusso4.187","flusso4.188","flusso4.189","flusso4.190","flusso4.191","flusso4.192","flusso4.193","flusso4.194","flusso4.195","flusso4.196","flusso4.197","flusso4.198","flusso4.199","flusso4.200","flusso4.201","flusso4.202","flusso4.203","flusso4.204","flusso4.205","flusso4.206","flusso4.207","flusso4.208","flusso4.209","flusso4.210","flusso4.211","flusso4.212","flusso4.213","flusso4.214","flusso4.215","flusso4.216","flusso4.217","flusso4.218","flusso4.219","flusso4.220","flusso4.221","flusso4.222","flusso4.223","flusso4.224","flusso4.225","flusso4.226","flusso4.227","flusso4.228","flusso4.229","flusso4.230","flusso4.231","flusso4.232","flusso4.233","flusso4.234","flusso4.235","flusso4.236","flusso4.237","flusso4.238","flusso4.239","flusso4.240","flusso4.241","flusso4.242","flusso4.243","flusso4.244","flusso4.245","flusso4.246","flusso4.247","flusso4.248","flusso4.249","flusso4.250","flusso4.251","flusso4.252","flusso4.253","flusso4.254","flusso4.255","flusso4.256","flusso4.257","flusso4.258","flusso4.259","flusso4.260","flusso4.261","flusso4.262","flusso4.263","flusso4.264","flusso4.265","flusso4.266","flusso4.267","flusso4.268","flusso4.269","flusso4.270","flusso4.271","flusso4.272","flusso4.273","flusso4.274","flusso4.275","flusso4.276","flusso4.277","flusso4.278","flusso4.279","flusso4.280","flusso4.281","flusso4.282","flusso4.283","flusso4.284","flusso4.285","flusso4.286","flusso4.287","flusso4.288","flusso4.289","flusso4.290","flusso4.291","flusso4.292","flusso4.293","flusso4.294","flusso4.295","flusso4.296","flusso4.297","flusso4.298","flusso4.299","flusso4.300","flusso4.301","flusso4.302","flusso4.303","flusso4.304","flusso4.305"]
    #Flusso_5=["flusso5.0","flusso5.1","flusso5.2","flusso5.3","flusso5.4","flusso5.5","flusso5.6","flusso5.7","flusso5.8","flusso5.9","flusso5.10","flusso5.11","flusso5.12","flusso5.13","flusso5.14","flusso5.15","flusso5.16","flusso5.17","flusso5.18","flusso5.19","flusso5.20","flusso5.21","flusso5.22","flusso5.23","flusso5.24","flusso5.25","flusso5.26","flusso5.27","flusso5.28","flusso5.29","flusso5.30","flusso5.31","flusso5.32","flusso5.33","flusso5.34","flusso5.35","flusso5.36","flusso5.37","flusso5.38","flusso5.39","flusso5.40","flusso5.41","flusso5.42","flusso5.43","flusso5.44","flusso5.45","flusso5.46","flusso5.47","flusso5.48","flusso5.49","flusso5.50","flusso5.51","flusso5.52","flusso5.53","flusso5.54","flusso5.55","flusso5.56","flusso5.57","flusso5.58","flusso5.59","flusso5.60","flusso5.61","flusso5.62","flusso5.63","flusso5.64","flusso5.65","flusso5.66","flusso5.67","flusso5.68","flusso5.69","flusso5.70","flusso5.71","flusso5.72","flusso5.73","flusso5.74","flusso5.75","flusso5.76","flusso5.77","flusso5.78","flusso5.79","flusso5.80","flusso5.81","flusso5.82","flusso5.83","flusso5.84","flusso5.85","flusso5.86","flusso5.87","flusso5.88","flusso5.89","flusso5.90","flusso5.91","flusso5.92","flusso5.93","flusso5.94","flusso5.95","flusso5.96","flusso5.97","flusso5.98","flusso5.99","flusso5.100","flusso5.101","flusso5.102","flusso5.103","flusso5.104","flusso5.105","flusso5.106","flusso5.107","flusso5.108","flusso5.109","flusso5.110","flusso5.111","flusso5.112","flusso5.113","flusso5.114","flusso5.115","flusso5.116","flusso5.117","flusso5.118","flusso5.119","flusso5.120","flusso5.121","flusso5.122","flusso5.123","flusso5.124","flusso5.125","flusso5.126","flusso5.127","flusso5.128","flusso5.129","flusso5.130","flusso5.131","flusso5.132","flusso5.133","flusso5.134","flusso5.135","flusso5.136","flusso5.137","flusso5.138","flusso5.139","flusso5.140","flusso5.141","flusso5.142","flusso5.143","flusso5.144","flusso5.145","flusso5.146","flusso5.147","flusso5.148","flusso5.149","flusso5.150","flusso5.151","flusso5.152","flusso5.153","flusso5.154","flusso5.155","flusso5.156","flusso5.157","flusso5.158","flusso5.159","flusso5.160","flusso5.161","flusso5.162","flusso5.163","flusso5.164","flusso5.165","flusso5.166","flusso5.167","flusso5.168","flusso5.169","flusso5.170","flusso5.171","flusso5.172","flusso5.173","flusso5.174","flusso5.175","flusso5.176","flusso5.177","flusso5.178","flusso5.179","flusso5.180","flusso5.181","flusso5.182","flusso5.183","flusso5.184","flusso5.185","flusso5.186","flusso5.187","flusso5.188","flusso5.189","flusso5.190","flusso5.191","flusso5.192","flusso5.193","flusso5.194","flusso5.195","flusso5.196","flusso5.197","flusso5.198","flusso5.199","flusso5.200","flusso5.201","flusso5.202","flusso5.203","flusso5.204","flusso5.205","flusso5.206","flusso5.207","flusso5.208","flusso5.209","flusso5.210","flusso5.211","flusso5.212","flusso5.213","flusso5.214","flusso5.215","flusso5.216","flusso5.217","flusso5.218","flusso5.219","flusso5.220","flusso5.221","flusso5.222","flusso5.223","flusso5.224","flusso5.225","flusso5.226","flusso5.227","flusso5.228","flusso5.229","flusso5.230","flusso5.231","flusso5.232","flusso5.233","flusso5.234","flusso5.235","flusso5.236","flusso5.237","flusso5.238","flusso5.239","flusso5.240","flusso5.241","flusso5.242","flusso5.243","flusso5.244","flusso5.245","flusso5.246","flusso5.247","flusso5.248","flusso5.249","flusso5.250","flusso5.251","flusso5.252","flusso5.253","flusso5.254","flusso5.255","flusso5.256","flusso5.257","flusso5.258","flusso5.259","flusso5.260","flusso5.261","flusso5.262","flusso5.263","flusso5.264","flusso5.265","flusso5.266","flusso5.267","flusso5.268","flusso5.269","flusso5.270","flusso5.271","flusso5.272","flusso5.273","flusso5.274","flusso5.275","flusso5.276","flusso5.277","flusso5.278","flusso5.279","flusso5.280","flusso5.281","flusso5.282","flusso5.283","flusso5.284","flusso5.285","flusso5.286","flusso5.287","flusso5.288","flusso5.289","flusso5.290","flusso5.291","flusso5.292","flusso5.293","flusso5.294","flusso5.295","flusso5.296","flusso5.297","flusso5.298","flusso5.299","flusso5.300","flusso5.301","flusso5.302","flusso5.303","flusso5.304","flusso5.305","flusso5.306","flusso5.307","flusso5.308","flusso5.309","flusso5.310","flusso5.311","flusso5.312","flusso5.313","flusso5.314","flusso5.315","flusso5.316","flusso5.317","flusso5.318","flusso5.319","flusso5.320","flusso5.321","flusso5.322","flusso5.323","flusso5.324","flusso5.325","flusso5.326","flusso5.327","flusso5.328","flusso5.329","flusso5.330","flusso5.331","flusso5.332","flusso5.333","flusso5.334","flusso5.335","flusso5.336","flusso5.337","flusso5.338","flusso5.339","flusso5.340","flusso5.341","flusso5.342","flusso5.343","flusso5.344","flusso5.345","flusso5.346","flusso5.347","flusso5.348","flusso5.349","flusso5.350","flusso5.351","flusso5.352","flusso5.353","flusso5.354","flusso5.355","flusso5.356","flusso5.357","flusso5.358","flusso5.359","flusso5.360","flusso5.361","flusso5.362","flusso5.363","flusso5.364","flusso5.365","flusso5.366","flusso5.367","flusso5.368","flusso5.369","flusso5.370","flusso5.371","flusso5.372","flusso5.373","flusso5.374","flusso5.375","flusso5.376","flusso5.377","flusso5.378","flusso5.379","flusso5.380","flusso5.381","flusso5.382","flusso5.383","flusso5.384","flusso5.385","flusso5.386","flusso5.387","flusso5.388","flusso5.389","flusso5.390","flusso5.391","flusso5.392","flusso5.393","flusso5.394","flusso5.395","flusso5.396","flusso5.397","flusso5.398","flusso5.399","flusso5.400","flusso5.401","flusso5.402","flusso5.403","flusso5.404","flusso5.405","flusso5.406","flusso5.407","flusso5.408","flusso5.409","flusso5.410","flusso5.411","flusso5.412","flusso5.413","flusso5.414","flusso5.415","flusso5.416","flusso5.417","flusso5.418","flusso5.419","flusso5.420","flusso5.421","flusso5.422","flusso5.423","flusso5.424","flusso5.425","flusso5.426","flusso5.427","flusso5.428","flusso5.429","flusso5.430","flusso5.431","flusso5.432","flusso5.433","flusso5.434","flusso5.435","flusso5.436","flusso5.437","flusso5.438","flusso5.439","flusso5.440","flusso5.441","flusso5.442","flusso5.443","flusso5.444","flusso5.445","flusso5.446","flusso5.447","flusso5.448","flusso5.449","flusso5.450","flusso5.451","flusso5.452","flusso5.453","flusso5.454","flusso5.455","flusso5.456","flusso5.457","flusso5.458","flusso5.459","flusso5.460","flusso5.461","flusso5.462","flusso5.463","flusso5.464","flusso5.465","flusso5.466","flusso5.467","flusso5.468","flusso5.469","flusso5.470","flusso5.471","flusso5.472","flusso5.473","flusso5.474","flusso5.475","flusso5.476","flusso5.477","flusso5.478","flusso5.479","flusso5.480","flusso5.481","flusso5.482","flusso5.483","flusso5.484","flusso5.485","flusso5.486","flusso5.487","flusso5.488","flusso5.489","flusso5.490","flusso5.491","flusso5.492","flusso5.493","flusso5.494","flusso5.495","flusso5.496","flusso5.497","flusso5.498","flusso5.499","flusso5.500","flusso5.501","flusso5.502","flusso5.503","flusso5.504","flusso5.505","flusso5.506","flusso5.507","flusso5.508","flusso5.509","flusso5.510","flusso5.511","flusso5.512","flusso5.513","flusso5.514","flusso5.515","flusso5.516","flusso5.517","flusso5.518","flusso5.519","flusso5.520","flusso5.521","flusso5.522","flusso5.523","flusso5.524","flusso5.525","flusso5.526","flusso5.527","flusso5.528","flusso5.529","flusso5.530","flusso5.531","flusso5.532","flusso5.533","flusso5.534","flusso5.535","flusso5.536","flusso5.537","flusso5.538","flusso5.539","flusso5.540","flusso5.541","flusso5.542","flusso5.543","flusso5.544","flusso5.545","flusso5.546","flusso5.547","flusso5.548","flusso5.549","flusso5.550","flusso5.551","flusso5.552","flusso5.553","flusso5.554","flusso5.555","flusso5.556","flusso5.557","flusso5.558","flusso5.559","flusso5.560","flusso5.561","flusso5.562","flusso5.563","flusso5.564","flusso5.565","flusso5.566","flusso5.567","flusso5.568","flusso5.569","flusso5.570","flusso5.571","flusso5.572","flusso5.573","flusso5.574","flusso5.575","flusso5.576","flusso5.577","flusso5.578","flusso5.579","flusso5.580","flusso5.581","flusso5.582","flusso5.583","flusso5.584","flusso5.585","flusso5.586","flusso5.587","flusso5.588","flusso5.589","flusso5.590","flusso5.591","flusso5.592","flusso5.593","flusso5.594","flusso5.595","flusso5.596","flusso5.597","flusso5.598","flusso5.599"]
    #Flusso_6=["flusso6.0","flusso6.1","flusso6.2","flusso6.3","flusso6.4","flusso6.5","flusso6.6","flusso6.7","flusso6.8","flusso6.9","flusso6.10","flusso6.11","flusso6.12","flusso6.13","flusso6.14","flusso6.15","flusso6.16","flusso6.17","flusso6.18","flusso6.19","flusso6.20","flusso6.21","flusso6.22","flusso6.23","flusso6.24","flusso6.25","flusso6.26","flusso6.27","flusso6.28","flusso6.29","flusso6.30","flusso6.31","flusso6.32","flusso6.33","flusso6.34","flusso6.35","flusso6.36","flusso6.37","flusso6.38","flusso6.39","flusso6.40","flusso6.41","flusso6.42","flusso6.43","flusso6.44","flusso6.45","flusso6.46","flusso6.47","flusso6.48","flusso6.49","flusso6.50","flusso6.51","flusso6.52","flusso6.53","flusso6.54","flusso6.55","flusso6.56","flusso6.57","flusso6.58","flusso6.59","flusso6.60","flusso6.61","flusso6.62","flusso6.63","flusso6.64","flusso6.65","flusso6.66","flusso6.67","flusso6.68","flusso6.69","flusso6.70","flusso6.71","flusso6.72","flusso6.73","flusso6.74","flusso6.75","flusso6.76","flusso6.77","flusso6.78","flusso6.79","flusso6.80","flusso6.81","flusso6.82","flusso6.83","flusso6.84","flusso6.85","flusso6.86","flusso6.87","flusso6.88","flusso6.89","flusso6.90","flusso6.91","flusso6.92","flusso6.93","flusso6.94","flusso6.95","flusso6.96","flusso6.97","flusso6.98","flusso6.99","flusso6.100","flusso6.101","flusso6.102","flusso6.103","flusso6.104","flusso6.105","flusso6.106","flusso6.107","flusso6.108","flusso6.109","flusso6.110","flusso6.111","flusso6.112","flusso6.113","flusso6.114","flusso6.115","flusso6.116","flusso6.117","flusso6.118","flusso6.119","flusso6.120","flusso6.121","flusso6.122","flusso6.123","flusso6.124","flusso6.125","flusso6.126","flusso6.127","flusso6.128","flusso6.129","flusso6.130","flusso6.131","flusso6.132","flusso6.133","flusso6.134","flusso6.135","flusso6.136","flusso6.137","flusso6.138","flusso6.139","flusso6.140","flusso6.141","flusso6.142","flusso6.143","flusso6.144","flusso6.145","flusso6.146","flusso6.147","flusso6.148","flusso6.149","flusso6.150","flusso6.151","flusso6.152","flusso6.153","flusso6.154","flusso6.155","flusso6.156","flusso6.157","flusso6.158","flusso6.159","flusso6.160","flusso6.161","flusso6.162","flusso6.163","flusso6.164","flusso6.165","flusso6.166","flusso6.167","flusso6.168","flusso6.169","flusso6.170","flusso6.171","flusso6.172","flusso6.173","flusso6.174","flusso6.175","flusso6.176","flusso6.177","flusso6.178","flusso6.179","flusso6.180","flusso6.181","flusso6.182","flusso6.183","flusso6.184","flusso6.185","flusso6.186","flusso6.187","flusso6.188","flusso6.189","flusso6.190","flusso6.191","flusso6.192","flusso6.193","flusso6.194","flusso6.195","flusso6.196","flusso6.197","flusso6.198","flusso6.199","flusso6.200","flusso6.201","flusso6.202","flusso6.203","flusso6.204","flusso6.205","flusso6.206","flusso6.207","flusso6.208","flusso6.209","flusso6.210","flusso6.211","flusso6.212","flusso6.213","flusso6.214","flusso6.215","flusso6.216","flusso6.217","flusso6.218","flusso6.219","flusso6.220","flusso6.221","flusso6.222","flusso6.223","flusso6.224","flusso6.225","flusso6.226","flusso6.227","flusso6.228","flusso6.229","flusso6.230","flusso6.231","flusso6.232","flusso6.233","flusso6.234","flusso6.235","flusso6.236","flusso6.237","flusso6.238","flusso6.239","flusso6.240","flusso6.241","flusso6.242","flusso6.243","flusso6.244","flusso6.245","flusso6.246","flusso6.247","flusso6.248","flusso6.249","flusso6.250","flusso6.251","flusso6.252","flusso6.253","flusso6.254","flusso6.255","flusso6.256","flusso6.257","flusso6.258","flusso6.259"]
    #Flusso_7=["flusso7.0","flusso7.1","flusso7.2","flusso7.3","flusso7.4","flusso7.5","flusso7.6","flusso7.7","flusso7.8","flusso7.9","flusso7.10","flusso7.11","flusso7.12","flusso7.13","flusso7.14","flusso7.15","flusso7.16","flusso7.17","flusso7.18","flusso7.19","flusso7.20","flusso7.21","flusso7.22","flusso7.23","flusso7.24","flusso7.25","flusso7.26","flusso7.27","flusso7.28","flusso7.29","flusso7.30","flusso7.31","flusso7.32","flusso7.33","flusso7.34","flusso7.35","flusso7.36","flusso7.37","flusso7.38","flusso7.39","flusso7.40","flusso7.41","flusso7.42","flusso7.43","flusso7.44","flusso7.45","flusso7.46","flusso7.47","flusso7.48","flusso7.49","flusso7.50","flusso7.51","flusso7.52","flusso7.53","flusso7.54","flusso7.55","flusso7.56","flusso7.57","flusso7.58","flusso7.59","flusso7.60","flusso7.61","flusso7.62","flusso7.63","flusso7.64","flusso7.65","flusso7.66","flusso7.67","flusso7.68","flusso7.69","flusso7.70","flusso7.71","flusso7.72","flusso7.73","flusso7.74","flusso7.75","flusso7.76","flusso7.77","flusso7.78","flusso7.79","flusso7.80","flusso7.81","flusso7.82","flusso7.83","flusso7.84","flusso7.85","flusso7.86","flusso7.87","flusso7.88","flusso7.89","flusso7.90","flusso7.91","flusso7.92","flusso7.93","flusso7.94","flusso7.95","flusso7.96","flusso7.97","flusso7.98","flusso7.99","flusso7.100","flusso7.101","flusso7.102","flusso7.103","flusso7.104","flusso7.105","flusso7.106","flusso7.107","flusso7.108","flusso7.109","flusso7.110","flusso7.111","flusso7.112","flusso7.113","flusso7.114","flusso7.115","flusso7.116","flusso7.117","flusso7.118","flusso7.119","flusso7.120","flusso7.121","flusso7.122","flusso7.123","flusso7.124","flusso7.125","flusso7.126","flusso7.127","flusso7.128","flusso7.129","flusso7.130","flusso7.131","flusso7.132","flusso7.133","flusso7.134","flusso7.135","flusso7.136","flusso7.137","flusso7.138","flusso7.139","flusso7.140","flusso7.141","flusso7.142","flusso7.143","flusso7.144","flusso7.145","flusso7.146","flusso7.147","flusso7.148","flusso7.149","flusso7.150","flusso7.151","flusso7.152","flusso7.153","flusso7.154","flusso7.155","flusso7.156","flusso7.157","flusso7.158","flusso7.159","flusso7.160","flusso7.161","flusso7.162","flusso7.163","flusso7.164","flusso7.165","flusso7.166","flusso7.167","flusso7.168","flusso7.169","flusso7.170","flusso7.171","flusso7.172","flusso7.173","flusso7.174","flusso7.175","flusso7.176","flusso7.177","flusso7.178","flusso7.179","flusso7.180","flusso7.181","flusso7.182","flusso7.183","flusso7.184","flusso7.185","flusso7.186","flusso7.187","flusso7.188","flusso7.189","flusso7.190","flusso7.191","flusso7.192","flusso7.193","flusso7.194","flusso7.195","flusso7.196","flusso7.197","flusso7.198","flusso7.199","flusso7.200","flusso7.201","flusso7.202","flusso7.203","flusso7.204","flusso7.205","flusso7.206","flusso7.207","flusso7.208","flusso7.209","flusso7.210","flusso7.211","flusso7.212","flusso7.213","flusso7.214","flusso7.215","flusso7.216","flusso7.217","flusso7.218","flusso7.219","flusso7.220","flusso7.221","flusso7.222","flusso7.223","flusso7.224","flusso7.225","flusso7.226","flusso7.227","flusso7.228","flusso7.229","flusso7.230","flusso7.231","flusso7.232","flusso7.233","flusso7.234","flusso7.235","flusso7.236","flusso7.237","flusso7.238","flusso7.239","flusso7.240","flusso7.241","flusso7.242","flusso7.243","flusso7.244","flusso7.245","flusso7.246","flusso7.247","flusso7.248","flusso7.249","flusso7.250","flusso7.251","flusso7.252","flusso7.253","flusso7.254","flusso7.255","flusso7.256","flusso7.257","flusso7.258","flusso7.259"]
    #Flusso_8=["flusso8.0","flusso8.1","flusso8.2","flusso8.3","flusso8.4","flusso8.5","flusso8.6","flusso8.7","flusso8.8","flusso8.9","flusso8.10","flusso8.11","flusso8.12","flusso8.13","flusso8.14","flusso8.15","flusso8.16","flusso8.17","flusso8.18","flusso8.19","flusso8.20","flusso8.21","flusso8.22","flusso8.23","flusso8.24","flusso8.25","flusso8.26","flusso8.27","flusso8.28","flusso8.29","flusso8.30","flusso8.31","flusso8.32","flusso8.33","flusso8.34","flusso8.35","flusso8.36","flusso8.37","flusso8.38","flusso8.39","flusso8.40","flusso8.41","flusso8.42","flusso8.43","flusso8.44","flusso8.45","flusso8.46","flusso8.47","flusso8.48","flusso8.49","flusso8.50","flusso8.51","flusso8.52","flusso8.53","flusso8.54","flusso8.55","flusso8.56","flusso8.57","flusso8.58","flusso8.59","flusso8.60","flusso8.61","flusso8.62","flusso8.63","flusso8.64","flusso8.65","flusso8.66","flusso8.67","flusso8.68","flusso8.69","flusso8.70","flusso8.71","flusso8.72","flusso8.73","flusso8.74","flusso8.75","flusso8.76","flusso8.77","flusso8.78","flusso8.79","flusso8.80","flusso8.81","flusso8.82","flusso8.83","flusso8.84","flusso8.85","flusso8.86","flusso8.87","flusso8.88","flusso8.89","flusso8.90","flusso8.91","flusso8.92","flusso8.93","flusso8.94","flusso8.95","flusso8.96","flusso8.97","flusso8.98","flusso8.99","flusso8.100","flusso8.101","flusso8.102","flusso8.103","flusso8.104","flusso8.105","flusso8.106","flusso8.107","flusso8.108","flusso8.109","flusso8.110","flusso8.111","flusso8.112","flusso8.113","flusso8.114","flusso8.115","flusso8.116","flusso8.117","flusso8.118","flusso8.119","flusso8.120","flusso8.121","flusso8.122","flusso8.123","flusso8.124","flusso8.125","flusso8.126","flusso8.127","flusso8.128","flusso8.129","flusso8.130","flusso8.131","flusso8.132","flusso8.133","flusso8.134","flusso8.135","flusso8.136","flusso8.137","flusso8.138","flusso8.139","flusso8.140","flusso8.141","flusso8.142","flusso8.143","flusso8.144","flusso8.145","flusso8.146","flusso8.147","flusso8.148","flusso8.149","flusso8.150","flusso8.151","flusso8.152","flusso8.153","flusso8.154","flusso8.155","flusso8.156","flusso8.157","flusso8.158","flusso8.159","flusso8.160","flusso8.161","flusso8.162","flusso8.163","flusso8.164","flusso8.165","flusso8.166","flusso8.167","flusso8.168","flusso8.169","flusso8.170","flusso8.171","flusso8.172","flusso8.173","flusso8.174","flusso8.175","flusso8.176","flusso8.177","flusso8.178","flusso8.179","flusso8.180","flusso8.181","flusso8.182","flusso8.183","flusso8.184","flusso8.185","flusso8.186","flusso8.187","flusso8.188","flusso8.189","flusso8.190","flusso8.191","flusso8.192","flusso8.193","flusso8.194","flusso8.195","flusso8.196","flusso8.197","flusso8.198","flusso8.199","flusso8.200","flusso8.201","flusso8.202","flusso8.203","flusso8.204","flusso8.205","flusso8.206","flusso8.207","flusso8.208","flusso8.209","flusso8.210","flusso8.211","flusso8.212","flusso8.213","flusso8.214","flusso8.215","flusso8.216","flusso8.217","flusso8.218","flusso8.219","flusso8.220","flusso8.221","flusso8.222","flusso8.223","flusso8.224","flusso8.225","flusso8.226","flusso8.227","flusso8.228","flusso8.229","flusso8.230","flusso8.231","flusso8.232","flusso8.233","flusso8.234","flusso8.235","flusso8.236","flusso8.237","flusso8.238","flusso8.239","flusso8.240","flusso8.241","flusso8.242","flusso8.243","flusso8.244","flusso8.245","flusso8.246","flusso8.247","flusso8.248","flusso8.249","flusso8.250","flusso8.251","flusso8.252","flusso8.253","flusso8.254","flusso8.255","flusso8.256","flusso8.257","flusso8.258","flusso8.259","flusso8.260","flusso8.261","flusso8.262","flusso8.263","flusso8.264","flusso8.265","flusso8.266","flusso8.267","flusso8.268","flusso8.269","flusso8.270","flusso8.271","flusso8.272","flusso8.273","flusso8.274","flusso8.275","flusso8.276","flusso8.277","flusso8.278","flusso8.279","flusso8.280","flusso8.281","flusso8.282","flusso8.283","flusso8.284","flusso8.285","flusso8.286","flusso8.287","flusso8.288","flusso8.289","flusso8.290","flusso8.291","flusso8.292","flusso8.293","flusso8.294","flusso8.295","flusso8.296","flusso8.297","flusso8.298","flusso8.299","flusso8.300"]
    #Flusso_9=["flusso9.0","flusso9.1","flusso9.2","flusso9.3","flusso9.4","flusso9.5","flusso9.6","flusso9.7","flusso9.8","flusso9.9","flusso9.10","flusso9.11","flusso9.12","flusso9.13","flusso9.14","flusso9.15","flusso9.16","flusso9.17","flusso9.18","flusso9.19","flusso9.20","flusso9.21","flusso9.22","flusso9.23","flusso9.24","flusso9.25","flusso9.26","flusso9.27","flusso9.28","flusso9.29","flusso9.30","flusso9.31","flusso9.32","flusso9.33","flusso9.34","flusso9.35","flusso9.36","flusso9.37","flusso9.38","flusso9.39","flusso9.40","flusso9.41","flusso9.42","flusso9.43","flusso9.44","flusso9.45","flusso9.46","flusso9.47","flusso9.48","flusso9.49","flusso9.50","flusso9.51","flusso9.52","flusso9.53","flusso9.54","flusso9.55","flusso9.56","flusso9.57","flusso9.58","flusso9.59","flusso9.60","flusso9.61","flusso9.62","flusso9.63","flusso9.64","flusso9.65","flusso9.66","flusso9.67","flusso9.68","flusso9.69","flusso9.70","flusso9.71","flusso9.72","flusso9.73","flusso9.74","flusso9.75","flusso9.76","flusso9.77","flusso9.78","flusso9.79","flusso9.80","flusso9.81","flusso9.82","flusso9.83","flusso9.84","flusso9.85","flusso9.86","flusso9.87","flusso9.88","flusso9.89","flusso9.90","flusso9.91","flusso9.92","flusso9.93","flusso9.94","flusso9.95","flusso9.96","flusso9.97","flusso9.98","flusso9.99","flusso9.100","flusso9.101","flusso9.102","flusso9.103","flusso9.104","flusso9.105","flusso9.106","flusso9.107","flusso9.108","flusso9.109","flusso9.110","flusso9.111","flusso9.112","flusso9.113","flusso9.114","flusso9.115","flusso9.116","flusso9.117","flusso9.118","flusso9.119","flusso9.120","flusso9.121","flusso9.122","flusso9.123","flusso9.124","flusso9.125","flusso9.126","flusso9.127","flusso9.128","flusso9.129","flusso9.130","flusso9.131","flusso9.132","flusso9.133","flusso9.134","flusso9.135","flusso9.136","flusso9.137","flusso9.138","flusso9.139","flusso9.140","flusso9.141","flusso9.142","flusso9.143","flusso9.144","flusso9.145","flusso9.146","flusso9.147","flusso9.148","flusso9.149","flusso9.150","flusso9.151","flusso9.152","flusso9.153","flusso9.154","flusso9.155","flusso9.156","flusso9.157","flusso9.158","flusso9.159","flusso9.160","flusso9.161","flusso9.162","flusso9.163","flusso9.164","flusso9.165","flusso9.166","flusso9.167","flusso9.168","flusso9.169","flusso9.170","flusso9.171","flusso9.172","flusso9.173","flusso9.174","flusso9.175","flusso9.176","flusso9.177","flusso9.178","flusso9.179","flusso9.180","flusso9.181","flusso9.182","flusso9.183","flusso9.184","flusso9.185","flusso9.186","flusso9.187","flusso9.188","flusso9.189","flusso9.190","flusso9.191","flusso9.192","flusso9.193","flusso9.194","flusso9.195","flusso9.196","flusso9.197","flusso9.198","flusso9.199","flusso9.200","flusso9.201","flusso9.202","flusso9.203","flusso9.204","flusso9.205","flusso9.206","flusso9.207","flusso9.208","flusso9.209","flusso9.210","flusso9.211","flusso9.212","flusso9.213","flusso9.214","flusso9.215","flusso9.216","flusso9.217","flusso9.218","flusso9.219","flusso9.220","flusso9.221","flusso9.222","flusso9.223","flusso9.224","flusso9.225","flusso9.226","flusso9.227","flusso9.228","flusso9.229","flusso9.230","flusso9.231","flusso9.232","flusso9.233","flusso9.234","flusso9.235","flusso9.236","flusso9.237","flusso9.238","flusso9.239","flusso9.240","flusso9.241","flusso9.242","flusso9.243","flusso9.244","flusso9.245","flusso9.246","flusso9.247","flusso9.248","flusso9.249","flusso9.250","flusso9.251","flusso9.252","flusso9.253","flusso9.254","flusso9.255","flusso9.256","flusso9.257","flusso9.258","flusso9.259","flusso9.260","flusso9.261","flusso9.262","flusso9.263","flusso9.264","flusso9.265","flusso9.266","flusso9.267","flusso9.268","flusso9.269","flusso9.270","flusso9.271","flusso9.272","flusso9.273","flusso9.274","flusso9.275","flusso9.276","flusso9.277","flusso9.278","flusso9.279","flusso9.280","flusso9.281","flusso9.282","flusso9.283","flusso9.284","flusso9.285","flusso9.286","flusso9.287","flusso9.288","flusso9.289","flusso9.290","flusso9.291","flusso9.292","flusso9.293","flusso9.294","flusso9.295","flusso9.296","flusso9.297","flusso9.298","flusso9.299","flusso9.300"]    
    Flusso_10=["flusso10.0","flusso10.1","flusso10.2","flusso10.3","flusso10.4","flusso10.5","flusso10.6","flusso10.7","flusso10.8","flusso10.9","flusso10.10","flusso10.11","flusso10.12","flusso10.13","flusso10.14","flusso10.15","flusso10.16","flusso10.17","flusso10.18","flusso10.19","flusso10.20","flusso10.21","flusso10.22","flusso10.23","flusso10.24","flusso10.25","flusso10.26","flusso10.27","flusso10.28","flusso10.29","flusso10.30","flusso10.31","flusso10.32","flusso10.33","flusso10.34","flusso10.35","flusso10.36","flusso10.37","flusso10.38","flusso10.39","flusso10.40","flusso10.41","flusso10.42","flusso10.43","flusso10.44","flusso10.45","flusso10.46","flusso10.47","flusso10.48","flusso10.49","flusso10.50","flusso10.51","flusso10.52","flusso10.53","flusso10.54","flusso10.55","flusso10.56","flusso10.57","flusso10.58","flusso10.59","flusso10.60","flusso10.61","flusso10.62","flusso10.63","flusso10.64","flusso10.65","flusso10.66","flusso10.67","flusso10.68","flusso10.69","flusso10.70","flusso10.71","flusso10.72","flusso10.73","flusso10.74","flusso10.75","flusso10.76","flusso10.77","flusso10.78","flusso10.79","flusso10.80","flusso10.81","flusso10.82","flusso10.83","flusso10.84","flusso10.85","flusso10.86","flusso10.87","flusso10.88","flusso10.89","flusso10.90","flusso10.91","flusso10.92","flusso10.93","flusso10.94","flusso10.95","flusso10.96","flusso10.97","flusso10.98","flusso10.99","flusso10.100","flusso10.101","flusso10.102","flusso10.103","flusso10.104","flusso10.105","flusso10.106","flusso10.107","flusso10.108","flusso10.109","flusso10.110","flusso10.111","flusso10.112","flusso10.113","flusso10.114","flusso10.115","flusso10.116","flusso10.117","flusso10.118","flusso10.119","flusso10.120","flusso10.121","flusso10.122","flusso10.123","flusso10.124","flusso10.125","flusso10.126","flusso10.127","flusso10.128","flusso10.129","flusso10.130","flusso10.131","flusso10.132","flusso10.133","flusso10.134","flusso10.135","flusso10.136","flusso10.137","flusso10.138","flusso10.139","flusso10.140","flusso10.141","flusso10.142","flusso10.143","flusso10.144","flusso10.145","flusso10.146","flusso10.147","flusso10.148","flusso10.149","flusso10.150","flusso10.151","flusso10.152","flusso10.153","flusso10.154","flusso10.155","flusso10.156","flusso10.157","flusso10.158","flusso10.159","flusso10.160","flusso10.161","flusso10.162","flusso10.163","flusso10.164","flusso10.165","flusso10.166","flusso10.167","flusso10.168","flusso10.169","flusso10.170","flusso10.171","flusso10.172","flusso10.173","flusso10.174","flusso10.175","flusso10.176","flusso10.177","flusso10.178","flusso10.179","flusso10.180","flusso10.181","flusso10.182","flusso10.183","flusso10.184","flusso10.185","flusso10.186","flusso10.187","flusso10.188","flusso10.189","flusso10.190","flusso10.191","flusso10.192","flusso10.193","flusso10.194","flusso10.195","flusso10.196","flusso10.197","flusso10.198","flusso10.199","flusso10.200","flusso10.201","flusso10.202","flusso10.203","flusso10.204","flusso10.205","flusso10.206","flusso10.207","flusso10.208","flusso10.209","flusso10.210","flusso10.211","flusso10.212","flusso10.213","flusso10.214","flusso10.215","flusso10.216","flusso10.217","flusso10.218","flusso10.219","flusso10.220","flusso10.221","flusso10.222","flusso10.223","flusso10.224","flusso10.225","flusso10.226","flusso10.227","flusso10.228","flusso10.229","flusso10.230","flusso10.231","flusso10.232","flusso10.233","flusso10.234","flusso10.235","flusso10.236","flusso10.237","flusso10.238","flusso10.239","flusso10.240","flusso10.241","flusso10.242","flusso10.243","flusso10.244","flusso10.245","flusso10.246","flusso10.247","flusso10.248","flusso10.249","flusso10.250","flusso10.251","flusso10.252","flusso10.253","flusso10.254","flusso10.255","flusso10.256","flusso10.257","flusso10.258","flusso10.259","flusso10.260","flusso10.261","flusso10.262","flusso10.263","flusso10.264","flusso10.265","flusso10.266","flusso10.267","flusso10.268","flusso10.269","flusso10.270","flusso10.271","flusso10.272","flusso10.273","flusso10.274","flusso10.275","flusso10.276","flusso10.277","flusso10.278","flusso10.279","flusso10.280","flusso10.281","flusso10.282","flusso10.283","flusso10.284","flusso10.285","flusso10.286","flusso10.287","flusso10.288","flusso10.289","flusso10.290","flusso10.291","flusso10.292","flusso10.293","flusso10.294","flusso10.295","flusso10.296","flusso10.297","flusso10.298","flusso10.299","flusso10.300","flusso10.301","flusso10.302","flusso10.303","flusso10.304","flusso10.305","flusso10.306","flusso10.307","flusso10.308","flusso10.309","flusso10.310","flusso10.311","flusso10.312","flusso10.313","flusso10.314","flusso10.315","flusso10.316","flusso10.317","flusso10.318","flusso10.319","flusso10.320","flusso10.321","flusso10.322","flusso10.323","flusso10.324","flusso10.325","flusso10.326","flusso10.327","flusso10.328","flusso10.329","flusso10.330","flusso10.331","flusso10.332","flusso10.333","flusso10.334","flusso10.335","flusso10.336","flusso10.337","flusso10.338","flusso10.339","flusso10.340","flusso10.341","flusso10.342","flusso10.343","flusso10.344","flusso10.345","flusso10.346","flusso10.347","flusso10.348","flusso10.349","flusso10.350","flusso10.351","flusso10.352","flusso10.353","flusso10.354","flusso10.355","flusso10.356","flusso10.357","flusso10.358","flusso10.359","flusso10.360","flusso10.361","flusso10.362","flusso10.363","flusso10.364","flusso10.365","flusso10.366","flusso10.367","flusso10.368","flusso10.369","flusso10.370","flusso10.371","flusso10.372","flusso10.373","flusso10.374","flusso10.375","flusso10.376","flusso10.377","flusso10.378","flusso10.379","flusso10.380","flusso10.381","flusso10.382","flusso10.383","flusso10.384","flusso10.385","flusso10.386","flusso10.387","flusso10.388","flusso10.389","flusso10.390","flusso10.391","flusso10.392","flusso10.393","flusso10.394","flusso10.395","flusso10.396","flusso10.397","flusso10.398","flusso10.399","flusso10.400"]
    Flusso_11=["flusso11.0","flusso11.1","flusso11.2","flusso11.3","flusso11.4","flusso11.5","flusso11.6","flusso11.7","flusso11.8","flusso11.9","flusso11.10","flusso11.11","flusso11.12","flusso11.13","flusso11.14","flusso11.15","flusso11.16","flusso11.17","flusso11.18","flusso11.19","flusso11.20","flusso11.21","flusso11.22","flusso11.23","flusso11.24","flusso11.25","flusso11.26","flusso11.27","flusso11.28","flusso11.29","flusso11.30"]
    Flusso_12=["flusso12.0","flusso12.1","flusso12.2","flusso12.3","flusso12.4","flusso12.5","flusso12.6","flusso12.7","flusso12.8","flusso12.9","flusso12.10","flusso12.11","flusso12.12","flusso12.13","flusso12.14","flusso12.15","flusso12.16","flusso12.17","flusso12.18","flusso12.19","flusso12.20","flusso12.21","flusso12.22","flusso12.23","flusso12.24","flusso12.25","flusso12.26","flusso12.27","flusso12.28","flusso12.29","flusso12.30"]
    Flusso_13=["flusso13.0","flusso13.1","flusso13.2","flusso13.3","flusso13.4","flusso13.5","flusso13.6","flusso13.7","flusso13.8","flusso13.9","flusso13.10","flusso13.11","flusso13.12","flusso13.13","flusso13.14","flusso13.15","flusso13.16","flusso13.17","flusso13.18","flusso13.19","flusso13.20","flusso13.21","flusso13.22","flusso13.23","flusso13.24","flusso13.25","flusso13.26","flusso13.27","flusso13.28","flusso13.29","flusso13.30","flusso13.31","flusso13.32","flusso13.33","flusso13.34","flusso13.35","flusso13.36","flusso13.37","flusso13.38","flusso13.39","flusso13.40","flusso13.41","flusso13.42","flusso13.43","flusso13.44","flusso13.45","flusso13.46","flusso13.47","flusso13.48","flusso13.49","flusso13.50","flusso13.51","flusso13.52","flusso13.53","flusso13.54","flusso13.55","flusso13.56","flusso13.57","flusso13.58","flusso13.59","flusso13.60","flusso13.61","flusso13.62","flusso13.63","flusso13.64","flusso13.65","flusso13.66","flusso13.67","flusso13.68","flusso13.69","flusso13.70","flusso13.71","flusso13.72","flusso13.73","flusso13.74","flusso13.75","flusso13.76","flusso13.77","flusso13.78","flusso13.79","flusso13.80","flusso13.81","flusso13.82","flusso13.83","flusso13.84","flusso13.85","flusso13.86","flusso13.87","flusso13.88","flusso13.89","flusso13.90"]
    #Flusso_14=["flusso14.0","flusso14.1","flusso14.2","flusso14.3","flusso14.4","flusso14.5","flusso14.6","flusso14.7","flusso14.8","flusso14.9","flusso14.10","flusso14.11","flusso14.12","flusso14.13","flusso14.14","flusso14.15","flusso14.16","flusso14.17","flusso14.18","flusso14.19","flusso14.20","flusso14.21","flusso14.22","flusso14.23","flusso14.24","flusso14.25","flusso14.26","flusso14.27","flusso14.28","flusso14.29","flusso14.30","flusso14.31","flusso14.32","flusso14.33","flusso14.34","flusso14.35","flusso14.36","flusso14.37","flusso14.38","flusso14.39","flusso14.40","flusso14.41","flusso14.42","flusso14.43","flusso14.44","flusso14.45","flusso14.46","flusso14.47","flusso14.48","flusso14.49","flusso14.50","flusso14.51","flusso14.52","flusso14.53","flusso14.54","flusso14.55","flusso14.56","flusso14.57","flusso14.58","flusso14.59","flusso14.60","flusso14.61","flusso14.62","flusso14.63","flusso14.64","flusso14.65","flusso14.66","flusso14.67","flusso14.68","flusso14.69","flusso14.70"]

    #Vettore per controllare le strade percorse dai flussi
    #Strada=[]

    #Vettori per simulare l'evoluzione della Batteria
    Distanza_Percorsa_1=[]
    Stato_Batteria_1=100
    Batteria=[]
    Batteria_1=[]
    Modalita_Green=[]

    #Definizione del dizionario con le densità dei pedestrians
    Dataset={':1710611042_1':0.1,':Sensore_4_1':0.1,':3371504162_1':0.5,':2917710998_0':1,':gneJ28_0':1,':3370489144_0':0.9,':324345142_1':0.5,':1243093942_1':0.4,':Sensore_18_4':0.3,':18122258_1':0.2,':18122239_0':0.1,'Via_Foria_00':0.1,'Via_Foria_10':0.1,'Via_Foria_20':0.1,'Via_Foria_30':0.3,'Via_Foria_40':0.4,'Via_Foria_50':0.5,'Via_Foria_60':0.9,'Via_Foria_70':1,'Rotonda_6':1,'Rotonda_7':1,'-gneE129':0.9,'-gneE181':0.5,'-gneE62':0.4,'gneE227':0.3,'-Via_Santa_Maria_ai_monti':0.2,'-Calata_Capodichino':0.1,':Sensore_3_1':0.6,':2914575958_1':0.6,':18121969_1':0.6,':3358305318_1':0.6,':197621530_7':0.5,':197621530_4':0.5,'-gneE65':0.5,'gneE102':0.6,'gneE230':0.6,'416695585':0.6,'gneE7':0.6,':4940669814_0':0.5,':2481698031_8':0.6,':18121969_7':0.8,':gneJ39_2':0.7,'Vico_Tutti_i_Santi':0.5,':1784935329_1':0.5,':232348640_0':0.5,':232348332_0':0.5,':gneJ40_6':0.4,':gneJ40_5':0.4,':224821001_2':0.5,':2481698031_5':0.6,':2914575958_4':0.6,':gneJ40_4':0.4,':Sensore_4_6':0.1,'gneE8':0.6,'Corso_Garibaldi_20':0.6,'Corso_Garibaldi_10':0.8,'gneE210':0.7,'Salita_Pontenuovo_2':0.5,'Salita_Pontenuovo_1':0.5,'-gneE211':0.5,'-via_Cesare_Rosaroli':0.6,'-via_Cesare_Rosaroli_10':0.4,':3370628909_2':0.7,':4940669814_2':0.5,':2481698031_2':0.7,':Sensore_3_8':0.4,'-Via_Taddeo_Sessa':0.8,'-Via_Taddeo_Sessa_1':0.7,'-Corso_Meridionale':0.5,'-Corso_Meridionale_1':0.5,'gneE2':0.7,'Via_Carbonara':0.4,':1710611042_6':0.1,':Sensore_22_5':0.1,':gneJ11_3':0.2,':1134071109_2':0.2,':gneJ38_3':0.5,':3362340400_7':0.8,'Via_Nuova_Poggioreale':0.5,'Via_Nuova_Poggioreale_1':0.1,'Via_Nuova_Poggioreale_2':0.2,'Via_Nuova_Poggioreale_3':0.2,'Via_Nuova_Poggioreale_5':0.8,'Via_Nuova_Poggioreale_4':0.5,':3370490096_2':1,':3949081571_0':0.3,':324345712_5':0.3,'gneE228':0.3,':1121869802_2':0.7,'gneE184':0.7,':232346432_4':1,':232346432_3':1,':1784920920_2':0.6,':4692709982_2':0.4,':1784920936_0':0.2,':Sensore_4_13':0.1,'Via_Veterinaia_2':1,'Via_Veterinaia_1':0.6,'Vico_SantaMaria':0.4,'-gneE231':0.3,'-gneE182':0.2,':Sensore_4_8':0.1,':gneJ24_3':0.3,'Via_Miano':1,'gneE159':0.3,':324345712_2':0.3,'-gneE226':0.3,':1243093942_6':0.5,':1243093942_3':0.5,'gneE181':0.5,':324345142_2':0.9,'gneE129':0.9,':3370489144_1':1,'Rotonda_5':1,':3370490096_4':1,':4965737714_0':1,'gneE31':1,':2917710999_2':1,'gneE32':0.9,':2914658867_3':0.7,'SantAntonio_AbateTerzo':0.7,':Sensore_20_2':0.5,':18122239_1':0.3,':3370490096_3':1,':197624056_2':1,':3370490094_1':1,':4965737714_2':1,':2917710998_3':0.9,':gneJ27_2':1,':Sensore_20_3':0.5,':3371504162_4':0.4,':2917630996_3':0.3,':2917660081_3':0.1,':Sensore_4_7':0.1,':1710611042_7':0.1,':3370628909_1':0.7,':224821001_1':0.5,':4940669814_1':0.5,':2481698031_1':0.5,':Sensore_3_4':0.7,'Via_Taddeo_Sessa':0.8,'Via_Taddeo_Sessa_1':0.7,'Corso_Meridionale':0.5,'Corso_Meridionale_1':0.5,'-gneE2':0.7,'-Corso_Arnaldo_Lucci':0.6,'Via_Foria_0': 0.1 ,'Via_Foria_1': 0.1, 'Via_Foria_2': 0.1, 'Via_Foria_3': 0.3, 'Via_Foria_4': 0.4,'Via_Foria_5': 0.5,'Via_Foria_6': 0.9,'Via_Foria_7': 1,'Rotonda_1':1,'Rotonda_2':1,'Rotonda_3':1,'Rotonda_4':1, 'Calata_Capodichino_1':0.3,'Calata_Capodichino':0.1 }
    Densita_Pedoni=[]
    
    #Vettore per controllare quanti veicoli ci sono su una strada e di che colore sono
    Veicoli_su_strada_1=[]
    Veicoli_su_strada_2=[]
    Colori_Veicoli_1=[]
    Colori_Veicoli_2=[]
    Colori_1=[]
    Colori_2=[]
    Velocita_1=[]
    Velocita_2=[]
    Emissioni_1=[]
    Emissioni_2=[]
    Inquinamento_1=[]
    Inquinamento_2=[]
    Inquinamento=[]

    #Definizione Vettori del Quattordicesimo Flusso con 70
    #partenze14=[]
    #arrivi14=[]
    #i=0
    #for i in range(0,70):
    #    partenze14.append(6000)
    #    arrivi14.append(6000)

    #Definizione Vettori del Tredicesimo Flusso con 90
    partenze13=[]
    arrivi13=[]
    i=0
    for i in range(0,90):
        partenze13.append(6000)
        arrivi13.append(6000)

    #Definizione Vettori dell'Undicesimo Flusso e Dodicesimo Flusso con 30
    partenze11=[]
    arrivi11=[]
    partenze12=[]
    arrivi12=[]
    i=0
    for i in range(0,30):
        partenze11.append(6000)
        arrivi11.append(6000)
        partenze12.append(6000)
        arrivi12.append(6000)

    #Definizione Vettori del Decimo Flusso con 400
    partenze10=[]
    arrivi10=[]
    i=0
    for i in range(0,400):
        partenze10.append(6000)
        arrivi10.append(6000)
        
    #Definizione Vettori dell'Ottavo Flusso e Nono Flusso con 300
    #partenze8=[]
    #arrivi8=[]
    #partenze9=[]
    #arrivi9=[]
    #i=0
    #for i in range(0,300):
    #    partenze8.append(6000)
    #    arrivi8.append(6000)
    #    partenze9.append(6000)
    #    arrivi9.append(6000)
        
    #Definizione Vettori del Sesto Flusso e Settimo Flusso con 260
    #partenze6=[]
    #arrivi6=[]
    #partenze7=[]
    #arrivi7=[]
    #i=0
    #for i in range(0,260):
    #    partenze6.append(6000)
    #    arrivi6.append(6000)
    #    partenze7.append(6000)
    #    arrivi7.append(6000)

    #Definizione Vettori del Quinto Flusso con 600 elementi
    #partenze5=[]
    #arrivi5=[]
    #i=0
    #for i in range (0,600):
    #    partenze5.append(6000)
    #    arrivi5.append(6000)

    #Definizione Vettori del Quarto Flusso con 306 elementi
    partenze4=[]
    arrivi4=[]
    i=0
    for i in range (0,306):
        partenze4.append(6000)
        arrivi4.append(6000)

    #Definizione Vettori del Terzo Flusso con 255 elementi
    partenze3=[]
    arrivi3=[]
    i=0
    for i in range (0,255):
        partenze3.append(6000)
        arrivi3.append(6000)
        
    #Definizione Vettori del Secondo Flusso con 265 elementi
    partenze2=[]
    arrivi2=[]
    i=0
    for i in range (0,265):
        partenze2.append(6000)
        arrivi2.append(6000)
    
    #Definizione Vettori del Primo flusso con 300 elementi
    partenze1=[]
    arrivi1=[]
    i=0
    for i in range (0,300):
        partenze1.append(6000)
        arrivi1.append(6000)
    
    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        #Utilizzo della modalità elettrica di un veicolo del Dodicesimo Flusso
        i=0
        for i in range(0,30):
            if Flusso_12[i] in traci.simulation.getDepartedIDList():
                partenze12[i]=step
            if Flusso_12[i] in traci.simulation.getArrivedIDList():
                arrivi12[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 12
        Controllo_Flusso_12(Strade_Elettriche_Flusso_12,Strade_Non_Elettriche_Flusso_12,Flusso_12,step,partenze12,arrivi12)
        
        #Utilizzo della modalità elettrica di un veicolo dell'Undicesimo flusso
        i=0
        for i in range(0,30):
            if Flusso_11[i] in traci.simulation.getDepartedIDList():
                partenze11[i]=step
            if Flusso_11[i] in traci.simulation.getArrivedIDList():
                arrivi11[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 11
        Controllo_Flusso_11(Strade_Elettriche_Flusso_11,Strade_Non_Elettriche_Flusso_11,Flusso_11,step,partenze11,arrivi11)
        
        #Utilizzo della modalità elettrica di un veicolo del Decimo flusso
        i=0
        for i in range(0,400):
            if Flusso_10[i] in traci.simulation.getDepartedIDList():
                partenze10[i]=step
            if Flusso_10[i] in traci.simulation.getArrivedIDList():
                arrivi10[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 10
        Controllo_Flusso_10(Strade_Elettriche_Flusso_10,Strade_Non_Elettriche_Flusso_10,Flusso_10,step,partenze10,arrivi10)
               
        #Utilizzo della modalità elettrica di un veicolo del Tredicesimo flusso
        i=0
        for i in range(0,90):
            if Flusso_13[i] in traci.simulation.getDepartedIDList():
                partenze13[i]=step
            if Flusso_13[i] in traci.simulation.getArrivedIDList():
                arrivi13[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 13
        Controllo_Flusso_13(Strade_Elettriche_Flusso_13,Strade_Non_Elettriche_Flusso_13,Flusso_13,step,partenze13,arrivi13)
        
        #Utilizzo della modalità elettrica di un veicolo del quarto flusso
        i=0
        for i in range(0,306):
            if Flusso_4[i] in traci.simulation.getDepartedIDList():
                partenze4[i]=step
            if Flusso_4[i] in traci.simulation.getArrivedIDList():
                arrivi4[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 4
        Controllo_Flusso_4(Strade_Elettriche_Flusso_4,Strade_Non_Elettriche_Flusso_4,Flusso_4,step,partenze4,arrivi4)

        #Utilizzo della modalità elettrica di un veicolo del terzo flusso
        i=0
        for i in range(0,255):
            if Flusso_3[i] in traci.simulation.getDepartedIDList():
                partenze3[i]=step
            if Flusso_3[i] in traci.simulation.getArrivedIDList():
                arrivi3[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 3
        Controllo_Flusso_3(Strade_Elettriche_Flusso_3,Strade_Non_Elettriche_Flusso_3,Flusso_3,step,partenze3,arrivi3)        

        #Utilizzo della modalità elettrica di un veicolo del secondo flusso
        i=0
        for i in range(0,265):
            if Flusso_2[i] in traci.simulation.getDepartedIDList():
                partenze2[i]=step
            if Flusso_2[i] in traci.simulation.getArrivedIDList():
                arrivi2[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 2
        Controllo_Flusso_2(Strade_Elettriche_Flusso_2,Strade_Non_Elettriche_Flusso_2,Flusso_2,step,partenze2,arrivi2)

        #Utilizzo della modalità elettrica di un veicolo del primo flusso
        i=0
        for i in range(0,300):
            if Flusso_1[i] in traci.simulation.getDepartedIDList():
                partenze1[i]=step
            if Flusso_1[i] in traci.simulation.getArrivedIDList():
                arrivi1[i]=step
        #Chiamata alla funzione che controlla la modalità elettrica di un veicolo del flusso 1
        Controllo_Flusso_1(Strade_Elettriche_Flusso_1,Strade_Non_Elettriche_Flusso_1,Flusso_1,step,partenze1,arrivi1)

        ##Evoluzione dello stato della batteria di un veicolo di un flusso
        #if Flusso_11[0] in traci.simulation.getDepartedIDList():
        #    Stato_Batteria_1 = 100
        #if step >= partenze11[0] and step < arrivi11[0]:
        #    if  traci.vehicle.getColor(Flusso_11[0])==(255,0,0,255):
        #        Distanza_Percorsa_1.append(traci.vehicle.getDistance(Flusso_11[0]))
        #        Densita_Pedoni.append(Dataset[traci.vehicle.getRoadID(Flusso_11[0])])
        #        Batteria_1.append(Stato_Batteria_1)
        #        Modalita_Green.append(0)
        #    if  traci.vehicle.getColor(Flusso_11[0])==(0,255,0,255):
        #        Distanza_Percorsa_1.append(traci.vehicle.getDistance(Flusso_11[0]))
        #        Stato_Batteria_1=Evoluzione_Batteria_1(Stato_Batteria_1,Distanza_Percorsa_1)
        #        Densita_Pedoni.append(Dataset[traci.vehicle.getRoadID(Flusso_11[0])])
        #        Batteria_1.append(Stato_Batteria_1)
        #        Modalita_Green.append(1)
        ##Plot dello stato della batteria, se la modalità elettrica è attiva o no, e il livello di Pedestrians nella strada
        #if step == arrivi11[0]:
        #    plt.figure()
        #    plt.subplot(3,1,1)
        #    plt.plot(Batteria_1)
        #    plt.title("Percentuale di carica Batteria")
        #    plt.subplot(3,1,2)
        #    plt.plot(Modalita_Green)
        #    plt.title("Attivazione modalità elettrica")
        #    plt.subplot(3,1,3)
        #    plt.plot(Densita_Pedoni)
        #    plt.title("Densità dei pedoni")
        #    plt.show()
                    
        ##Utilizzo della modalità elettrica di un veicolo ed evoluzione dello stato della batteria
        if "Veicolo_Elettrico" in traci.simulation.getDepartedIDList():
            Stato_Batteria = 100
            partenza = step
            arrivo = 6000
            Strada_Prevista.append(traci.vehicle.getRoute("Veicolo_Elettrico"))
            #valori=Ottimizzazione()
            valori=Ottimizzazione_Gekko_per_Veicolo_Elettrico()
        if "Veicolo_Elettrico" in traci.simulation.getArrivedIDList():
            arrivo=step
        if step >= partenza and step < arrivo:
            if traci.vehicle.getRoadID("Veicolo_Elettrico") in Strada_Prevista[0]:
                index = Strada_Prevista[0].index(traci.vehicle.getRoadID("Veicolo_Elettrico"))
                if valori[index]==[0.0]:
                    traci.vehicle.setColor("Veicolo_Elettrico",(255,0,0))
                    Modalita_Green.append(0)
                    Densita_Pedoni.append(Dataset[traci.vehicle.getRoadID("Veicolo_Elettrico")])
                    Distanza_Percorsa.append(traci.vehicle.getDistance("Veicolo_Elettrico"))
                    Batteria.append(Stato_Batteria)
                if valori[index]==[1.0]:
                    traci.vehicle.setColor("Veicolo_Elettrico",(0,255,0))
                    Modalita_Green.append(1)
                    Densita_Pedoni.append(Dataset[traci.vehicle.getRoadID("Veicolo_Elettrico")])
                    Distanza_Percorsa.append(traci.vehicle.getDistance("Veicolo_Elettrico"))
                    Stato_Batteria=Evoluzione_Batteria(Stato_Batteria,Distanza_Percorsa)
                    Batteria.append(Stato_Batteria)
        ##Plot dello stato della batteria, se la modalità elettrica è attiva o no, e il livello di Pedestrians nella strada
        if step == arrivo:
            plt.figure()
            plt.subplot(3,1,1)
            plt.plot(Batteria)
            plt.subplot(3,1,2)
            plt.plot(Modalita_Green)
            plt.subplot(3,1,3)
            plt.plot(Densita_Pedoni)
            plt.show()

        ##Numero di veicoli su una strada e i loro colori
        Veicoli_su_strada_1.append(traci.lane.getLastStepVehicleIDs("Via_Foria_6_1"))
        numero_veicoli = traci.lane.getLastStepVehicleNumber("Via_Foria_6_1")
        if numero_veicoli == 0:
            #print("Non ci sono veicoli")
            Inquinamento_1.append(0)
            Veicoli_su_strada_1=[]
        if numero_veicoli > 0:
            i=0
            for i in range(0,numero_veicoli):
                if traci.vehicle.getColor(Veicoli_su_strada_1[0][i])==(255,0,0,255):
                    Velocita_1.append(traci.vehicle.getSpeed(Veicoli_su_strada_1[0][i]))
                Colori_Veicoli_1.append(traci.vehicle.getColor(Veicoli_su_strada_1[0][i]))
            i=0
            for i in range(0,len(Velocita_1)):
                Emissioni_1.append( Velocita_1[i]*Velocita_1[i]*4.29808761 - 101.57881278*Velocita_1[i] + 843.08022187 )
            Inquinamento_1.append(sum(Emissioni_1))
            
                
            #print(numero_veicoli)
            #print(Veicoli_su_strada)
            #print(Colori_Veicoli_1)
            #print(Velocita)
            #print(Emissioni)
            #print(Inquinamento_1)
            #print(step)
            Veicoli_su_strada_1=[]
            Colori_Veicoli_1=[]
            Velocita_1=[]
            Emissioni_1=[]

        ##Numero di veicoli su una strada e i loro colori
        Veicoli_su_strada_2.append(traci.lane.getLastStepVehicleIDs("Via_Foria_60_1"))
        numero_veicoli = traci.lane.getLastStepVehicleNumber("Via_Foria_60_1")
        if numero_veicoli == 0:
            #print("Non ci sono veicoli")
            Inquinamento_2.append(0)
            Veicoli_su_strada_2=[]
        if numero_veicoli > 0:
            i=0
            for i in range(0,numero_veicoli):
                if traci.vehicle.getColor(Veicoli_su_strada_2[0][i])==(255,0,0,255):
                    Velocita_2.append(traci.vehicle.getSpeed(Veicoli_su_strada_2[0][i]))
                Colori_Veicoli_2.append(traci.vehicle.getColor(Veicoli_su_strada_2[0][i]))
            i=0
            for i in range(0,len(Velocita_2)):
                Emissioni_2.append( Velocita_2[i]*Velocita_2[i]*4.29808761 - 101.57881278*Velocita_2[i] + 843.08022187 )
            Inquinamento_2.append(sum(Emissioni_2))
            
                
            #print(numero_veicoli)
            #print(Veicoli_su_strada_2)
            #print(Colori_Veicoli_2)
            #print(Velocita_2)
            #print(Emissioni_2)
            #print(Inquinamento_2)
            #print(step)
            Veicoli_su_strada_2=[]
            Colori_Veicoli_2=[]
            Velocita_2=[]
            Emissioni_2=[]
            
 
        step+=1

    ##Plot dell'inquinamento su una strada ad alta densità pedonale
    i=0
    for i in range(0,len(Inquinamento_1)):
        Inquinamento.append(Inquinamento_1[i]+Inquinamento_2[i])
        
    plt.plot(Inquinamento)
    plt.title("Pollution with hybrid vehicles mg/m")
    plt.show()
    print("Average Pollution: ", sum(Inquinamento)/len(Inquinamento))
        
    traci.close()
    sys.stdout.flush()
	
	
def get_options():
    optParser = optparse.OptionParser()
    optParser.add_option("--nogui", action="store_true",
                         default=False, help="run the commandline version of sumo")
    options, args = optParser.parse_args()
    return options


# this is the main entry point of this script
if __name__ == "__main__":
    options = get_options()

    # this script has been called from the command line. It will start sumo as a
    # server, then connect and run
    if options.nogui:
        sumoBinary = checkBinary('sumo')
    else:
        sumoBinary = checkBinary('sumo-gui')

    # this is the normal way of using traci. sumo is started as a
    # subprocess and then the python script connects and runs
    traci.start([sumoBinary, "-c", "data/Napoli.sumocfg",
                             "--tripinfo-output", "tripinfo.xml"])
    run()
