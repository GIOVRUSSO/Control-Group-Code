from __future__ import absolute_import
from __future__ import print_function

import os
import sys
import optparse
import numpy
import math
import numpy as np
from numpy import zeros
import matplotlib.pyplot as plt
import random
from pulp import*
from scipy import stats
from gekko import GEKKO

# we need to import python modules from the $SUMO_HOME/tools directory
if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:
    sys.exit("please declare environment variable 'SUMO_HOME'")

from sumolib import checkBinary  # noqa
import traci  # noqa

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola la strada in base a dove si trova il veicolo
def Ottimizzazione_Gekko_Caso_Target(Batteria):
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
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (230/150*10*x1) - (178/150*10*x2) - (157/150*10*x3) - (68/150*10*x4) - (54/150*10*x5) - (186/150*10*x6) -(64/150*10*x7) - (858/150*10*x8) - (230/150*10*x9) - (90/150*10*x10) - (50/150*10*x11) - (145/150*10*x12) >= 30)
    #Funzione obiettivo
    m.Obj(-(0.1*230*x1 + 0.1*178*x2 + 0.1*157*x3 + 0.3*68*x4 + 0.4*54*x5 + 0.5*186*x6 + 0.6*64*x7 + 858*x8 + 0.7*230*x9 + 0.7*90*x10 + 0.7*50*x11 + 145*x12 )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value)
    return Variabili

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola la strada in base a dove si trova il veicolo
def Ottimizzazione_Gekko_Caso_1(Batteria):
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
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (382/150*10*x1) - (162/150*10*x2) - (127/150*10*x3) - (64/150*10*x4) - (858/150*10*x5) - (230/150*10*x6) -(90/150*10*x7) - (50/150*10*x8) - (145/150*10*x9) >= 30)
    #Funzione obiettivo
    m.Obj(-(0.2*382*x1 + 0.3*162*x2 + 0.4*127*x3 + 0.6*64*x4 + 858*x5 + 0.7*230*x6 + 0.7*90*x7 + 0.7*50*x8 + 145*x9 )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value)
    return Variabili

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola la strada in base a dove si trova il veicolo
def Ottimizzazione_Gekko_Caso_2(Batteria):
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
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (155/150*10*x1) - (162/150*10*x2) - (127/150*10*x3) - (64/150*10*x4) - (858/150*10*x5) - (230/150*10*x6) -(90/150*10*x7) - (50/150*10*x8) - (145/150*10*x9) >= 30)
    #Funzione obiettivo
    m.Obj(-(0.2*155*x1 + 0.3*162*x2 + 0.4*127*x3 + 0.6*64*x4 + 858*x5 + 0.7*230*x6 + 0.7*90*x7 + 0.7*50*x8 + 145*x9 )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value)
    return Variabili

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola la strada in base a dove si trova il veicolo
def Ottimizzazione_Gekko_Caso_3(Batteria):
    m=GEKKO()

    m.options.SOLVER=1

    x1 = m.Var(value=0, lb=0, ub=1, integer=True)
    x2 = m.Var(value=0, lb=0, ub=1, integer=True)
    x3 = m.Var(value=0, lb=0, ub=1, integer=True)
    x4 = m.Var(value=0, lb=0, ub=1, integer=True)
    x5 = m.Var(value=0, lb=0, ub=1, integer=True)
    x6 = m.Var(value=0, lb=0, ub=1, integer=True)
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (138/150*10*x1) - (134/150*10*x2) - (85/150*10*x3) - (346/150*10*x4) - (50/150*10*x5) - (145/150*10*x6) >= 30)
    #Funzione obiettivo
    m.Obj(-(0.6*138*x1 + 0.6*134*x2 + 0.5*85*x3 + 0.3*346*x4 + 0.7*50*x5 + 145*x6 )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value)
    return Variabili

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola la strada in base a dove si trova il veicolo
def Ottimizzazione_Gekko_Caso_4(Batteria):
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
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (277/150*10*x1) - (60/150*10*x2) - (80/150*10*x3) - (22/150*10*x4) - (9/150*10*x5) - (27/150*10*x6) - (39/150*10*x7) - (19/150*10*x8) - (141/150*10*x9) - (84/150*10*x10) - (346/150*10*x11) - (50/150*10*x12) - (145/150*10*x13) >= 38)
    #Funzione obiettivo
    m.Obj(-(0.5*277*x1 + 0.9*60*x2 + 80*x3 + 22*x4 + 9*x5 + 27*x6 + 39*x7 + 19*x8 + 0.9*141*x9 + 0.5*84*x10 + 0.3*346*x11 + 0.7*50*x12 + 145*x13 )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value,x13.value)
    return Variabili

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola la strada in base a dove si trova il veicolo
def Ottimizzazione_Gekko_Caso_5(Batteria):
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
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (43/150*10*x1) - (60/150*10*x2) - (78/150*10*x3) - (9/150*10*x4) - (27/150*10*x5) - (39/150*10*x6) - (19/150*10*x7) - (141/150*10*x8) - (84/150*10*x9) - (346/150*10*x10) - (50/150*10*x11) - (145/150*10*x12) >= 38)
    #Funzione obiettivo
    m.Obj(-( 0.9*43*x1 + 0.9*60*x2 + 78*x3 + 9*x4 + 27*x5 + 39*x6 + 19*x7 + 0.9*141*x8 + 0.5*84*x9 + 0.3*346*x10 + 0.7*50*x11 + 145*x12 )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value)
    return Variabili

#Funzione che risolve il problema di ottimizzazione utilizzando GEKKO, Calcola le strade dove attivare la modalità elettrica, le strade sono date dai contributor
def Ottimizzazione_Gekko_per_Veicolo_Intelligente(Batteria):
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
    x15 = m.Var(value=0, lb=0, ub=1, integer=True)
    x16 = m.Var(value=0, lb=0, ub=1, integer=True)
    x17 = m.Var(value=0, lb=0, ub=1, integer=True)
    x18 = m.Var(value=0, lb=0, ub=1, integer=True)
    x19 = m.Var(value=0, lb=0, ub=1, integer=True)
    x20 = m.Var(value=0, lb=0, ub=1, integer=True)
    x21 = m.Var(value=0, lb=0, ub=1, integer=True)
    x22 = m.Var(value=0, lb=0, ub=1, integer=True)
    x23 = m.Var(value=0, lb=0, ub=1, integer=True)
    x24 = m.Var(value=0, lb=0, ub=1, integer=True)
    x25 = m.Var(value=0, lb=0, ub=1, integer=True)
    x26 = m.Var(value=0, lb=0, ub=1, integer=True)
    x27 = m.Var(value=0, lb=0, ub=1, integer=True)
    x28 = m.Var(value=0, lb=0, ub=1, integer=True)
    x29 = m.Var(value=0, lb=0, ub=1, integer=True)
    x30 = m.Var(value=0, lb=0, ub=1, integer=True)
    x31 = m.Var(value=0, lb=0, ub=1, integer=True)
    x32 = m.Var(value=0, lb=0, ub=1, integer=True)
    x33 = m.Var(value=0, lb=0, ub=1, integer=True)
    x34 = m.Var(value=0, lb=0, ub=1, integer=True)
    x35 = m.Var(value=0, lb=0, ub=1, integer=True)
    x36 = m.Var(value=0, lb=0, ub=1, integer=True)
    
    #Equazioni sulla salvaguardia della batteria fino al 30 per cento
    m.Equation(Batteria - (230/150*10*x1) - (178/150*10*x11) - (157/150*10*x13) - (68/150*10*x14) - (54/150*10*x15) - (277/150*10*x16) - (43/150*10*x27) - (60/150*10*x28) - (78/150*10*x29) - (9/150*10*x20) - (27/150*10*x21) - (39/150*10*x22) - (19/150*10*x23) - (141/150*10*x24) - (84/150*10*x25) - (346/150*10*x26) - (50/150*10*x9) - (63/150*10*x32) >= 30)
    m.Equation(Batteria - (230/150*10*x1) - (178/150*10*x11) - (155/150*10*x12) - (162/150*10*x3) - (127/150*10*x4) - (64/150*10*x5) - (858/150*10*x6) - (138/150*10*x30) - (134/150*10*x31) - (84/150*10*x25) - (63/150*10*x32) >= 30)
    m.Equation(Batteria - (230/150*10*x1) - (178/150*10*x11) - (157/150*10*x13) - (68/150*10*x14) - (54/150*10*x15) - (277/150*10*x16) - (60/150*10*x17) - (80/150*10*x18) - (22/150*10*x19) - (9/150*10*x20) - (27/150*10*x21) - (39/150*10*x22) - (19/150*10*x23) - (141/150*10*x24) - (84/150*10*x25) - (346/150*10*x26) - (50/150*10*x9) - (145/150*10*x10) >= 30)
    m.Equation(Batteria - (230/150*10*x1) - (382/150*10*x2) - (162/150*10*x3) - (127/150*10**x4) - (64/150*10*x5) - (858/150*10*x6) - (230/150*10*x7) - (90/150*10*x8) - (50/150*10*x9) - (145/150*10*x10) >= 30)
    m.Equation(Batteria - (230/150*10*x1) - (178/150*10*x11) - (157/150*10*x13) - (186/150*10*x33) - (64/150*10*x5) - (187/150*10*x34) - (162/150*10*x35) - (382/150*10*x36) - (178/150*10*x11) - (157/150*10*x13) >= 30)
    m.Equation(Batteria - (230/150*10*x1) - (178/150*10*x11) - (157/150*10*x13) - (68/150*10*x14) - (54/150*10*x15) - (186/150*10*x33) -(64/150*10*x5) - (858/150*10*x6) - (230/150*10*x7) - (90/150*10*x8) - (50/150*10*x9) - (145/150*10*x10) >= 30)
    #Funzione obiettivo
    m.Obj(-(1/2*(0.1*230*x1 + 0.1*178*x11 + 0.1*157*x13 + 0.3*68*x14 + 0.4*54*x15 + 0.5*186*x33 + 0.6*64*x5 + 858*x6 + 0.7*230*x7 + 0.7*90*x8 + 0.7*50*x9 + 145*x10 ) + 1/10*(0.1*230*x1 + 0.1*178*x11 + 0.1*157*x13 + 0.3*68*x14 + 0.4*54*x15 + 0.5*277*x16 + 0.7*43*x27 + 0.9*60*x28 + 78*x29 + 9*x20 + 27*x21 + 39*x22 + 19*x23 + 0.9*141*x24 + 0.5*84*x25 + 0.3*346*x26 + 0.7*50*x9 + 0.4*63*x32 ) + 1/10*(0.1*230*x1 + 0.1*178*x11 + 0.1*157*x13 + 0.3*68*x14 + 0.4*54*x15 + 0.5*277*x16 + 0.9*60*x17 + 80*x18 + 22*x19 + 9*x20 + 27*x21 + 39*x22 + 19*x23 + 0.9*141*x24 + 0.5*84*x25 + 0.3*346*x26 + 0.7*50*x9 + 145*x10 ) + 1/10*(0.1*230*x1 + 0.1*178*x11 + 0.2*155*x12 + 0.3*162*x3 + 0.4*127*x4 + 0.6*64*x5 + 858*x6 + 0.6*138*x30 + 0.6*134*x31 + 0.5*84*x25 + 0.4*63*x32 ) + 1/10*(0.1*230*x1 + 0.2*382*x2 + 0.3*162*x3 + 0.4*127*x4 + 0.6*64*x5 + 858*x6 + 0.7*230*x7 + 0.7*90*x8 + 0.7*50*x9 + 145*x10) + 1/10*(0.1*230*x1 + 0.1*178*x11 + 0.1*157*x13 + 0.5*186*x33 + 0.6*64*x5 + 0.4*187*x34 + 0.1*162*x35 + 0.2*382*x36 + 0.1*178*x11 + 0.1*157*x13 ) )) # Objective

    m.solve() # Solve
    #Come valore dal problema di ottimizzazione tornano le variabili, 0(modalità elettrica non attiva) o 1(modalità elettrica attiva)
    Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value,x13.value,x14.value,x15.value,x16.value,x17.value,x18.value,x19.value,x20.value,x21.value,x22.value,x23.value,x24.value,x25.value,x26.value,x27.value,x28.value,x29.value,x30.value,x31.value,x32.value,x33.value,x34.value,x35.value)
    return Variabili

##Funzione che calcola la divergenza di Kullback-Leibler
def KL(a, b):
    a = np.asarray(a, dtype=np.float64)
    b = np.asarray(b, dtype=np.float64)

    return np.sum(np.where(a != 0, a * np.log(a / b), 0))

#Funzione che simula tutto l'algoritmo di Crowdsourcing
def Simulazione_Crowdsourcing ( numero_via, via ):

    Strada_Calcolata=[]
    Decisione=[]
    Soddisfazione=[]
    Expectation=[]
    Contributor_seguito=[]
    Input=[]
    Probabilita=[]
    Output=[]
    Densita_Pedoni=[]
    p_target=[]
    
    ##Definisco le matrici di probabilità delle girate data la strada target e la quelle dei contributor
    Probabilita_Target = { 3:[0.01, 0.98, 0.01], 6:[0.01,0.98,0.01], 17:[0.98, 0.01, 0.01], 40:[1], 41: [0.01, 0.99], 42: [0.99, 0.01], 47:[0.99, 0.01], 49:[0.99, 0.01], 51:[0.99, 0.01]}  
    Probabilita_Contributor_0 = { 4:[0.9, 0.09, 0.01], 8:[0.6, 0.39, 0.01], 10:[1], 40:[0.1, 0.9], 41:[0.1,0.9], 42:[0.9, 0.1], 47:[0.99, 0.01], 49:[0.99, 0.01], 51:[0.99, 0.01]}
    Probabilita_Contributor_1 = { 3:[0.05, 0.9, 0.05], 5: [0.8, 0.15, 0.05], 10:[0.2, 0.79, 0.01], 40:[0.15, 0.85], 41:[0.15,0.85], 43:[0.3, 0.7], 44:[1], 46:[0.2,0.8], 53:[0.3,0.7]}
    Probabilita_Contributor_2 = { 3:[0.07, 0.86, 0.07], 6:[0.1,0.8,0.1], 17:[0.9, 0.05, 0.05], 40:[1], 9:[0.9,0.1], 8:[1], 4:[0.6,0.4], 6:[0.1,0.8,0.1]}
    Probabilita_Contributor_3 = { 3:[0.1, 0.8, 0.1], 6:[0.2,0.6,0.2], 16:[0.2,0.6,0.2], 19:[0.8,0.2], 31:[0.1,0.9], 34:[0.1,0.9], 35:[0.1, 0.9], 46:[0.15,0.85], 51:[0.99, 0.01]}
    Probabilita_Contributor_4 = { 3:[0.1, 0.8, 0.1], 6:[0.2,0.6,0.2], 16:[0.2,0.6,0.2], 18:[0.2,0.8], 31:[0.1,0.9], 34:[0.1,0.9], 35:[0.1, 0.9], 46:[0.15,0.85], 53:[0.1,0.9]}
    Probabilita_Contributor_5 = { 2:[0.1,0.1,0.8]}

    ##Dizionario che associa strada al numero della strada nella matrice delle frequenze di girata
    Dataset_Strade={'Via_Carbonara_1':2, 'gneE62_1':53, 'gneE221_1':4, 'gneE231_1':8,'Via_SantaMaria_DegliAngeli_1':9, 'gneE127_0':44, 'gneE177_1': 43, 'SantAntonio_AbateTerzo_1':18, 'gneE32_1':23, 'gneE31_1':25,'-gneE231_1':8,'-gneE182_1':5,'gneE228_1':47,'gneE184_1':42,'Via_Veterinaia_0_1':17,'Via_Veterinaia_2_1':41,'Via_Veterinaia_1_1':40,'Vico_SantaMaria_1':10,'-gneE231_1':8,'-gneE221_1':4,'gneE129_1':35, 'gneE181_1':46, '-gneE226_1':48, 'gneE159_1':49, 'Via_Miano_1':51,'Rotonda_5_2':34, 'Rotonda_5_1':34,'Rotonda_4_1':31, 'Rotonda_4_2':31, 'Rotonda_3_1':27,'Rotonda_2_1':26,'Rotonda_1_1':22,'Via_Foria_0_1':1,'Via_Foria_1_1':3,'Via_Foria_2_1':6,'Via_Foria_3_1':11,'Via_Foria_4_1':13,'Via_Foria_5_1':16,'Via_Foria_6_1':19,'Via_Foria_7_1':20}

    ##Dizionario per assegnare le posizioni nella Reward
    Assegnazione_Reward= {'Rotonda_5_1':17, 'Via_Carbonara_1':26, 'gneE62_1':25, 'gneE127_0':24, 'gneE177_1':23, 'Via_SantaMaria_DegliAngeli_1':22, 'Via_Veterinaia_0_1': 21, 'SantAntonio_AbateTerzo_1':20, 'gneE181_1':19, 'gneE129_1':18, 'Rotonda_5_2':17, 'Rotonda_4_1':16, 'Rotonda_4_2':16, 'Via_Foria_6_1':15, 'Via_Foria_5_1':14, 'Via_Foria_2_1':13, '-gneE182_1':12, 'gneE159_1':9,'Via_Miano_1':10, 'Via_Foria_1_1':11,'gneE228_1':8, 'Via_Foria_0_1':1, '-gneE221_1':2, '-gneE231_1':3, 'Vico_SantaMaria_1':4, 'Via_Veterinaia_1_1':5, 'Via_Veterinaia_2_1':6, 'gneE184_1':7} 
    
    #Vettore delle Reward
    Reward = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    Reward[2]  = 0
    Reward[10] = 30 ##Reward della destinazione posta positiva
    Reward[12] = 10
    Reward[13] = 0
    Reward[14] = 0
    Reward[15] = 0
    Reward[20] = 0
    Reward[21] = 0
    Reward[23] = 10

    ##Definisco le Reward per le girate
    Reward_Contributor_0 = { 4:[ Reward[2], Reward[11], 0 ], 8:[Reward[8],0,0], 10:[Reward[4]], 40:[0, Reward[5] ], 41:[0, Reward[6]], 42:[Reward[7], 0], 47:[Reward[8], 0], 49:[Reward[9], 0], 51:[Reward[10],0]}
    Reward_Contributor_1 = { 3:[ Reward[2], Reward[11], 0 ], 5:[Reward[12],0,0], 10:[0,Reward[4],0], 40:[0,Reward[5]], 41:[0,Reward[6]], 43:[0, Reward[23]], 44:[Reward[24]], 46:[0, Reward[25]],53:[0, Reward[19]] }
    Reward_Contributor_2 = { 3:[ Reward[2], Reward[11], 0 ], 6:[0,Reward[13],0], 17:[Reward[21],0,0], 40:[Reward[5]], 9:[0,0], 8:[0], 4:[0,0], 6:[0,0,0] }
    Reward_Contributor_3 = { 3:[ Reward[2], Reward[11], 0 ], 6:[0,Reward[13],0], 16:[0,Reward[14],0], 19:[Reward[15],0], 31:[0,0], 34:[0,0], 35:[0, 0], 46:[0,0], 51:[0, 0]}
    Reward_Contributor_4 = { 3:[ Reward[2], Reward[11], 0 ], 6:[0,Reward[13],0], 16:[0,Reward[14],0], 18:[0,Reward[20]], 31:[0,0], 34:[0,0], 35:[0, 0], 46:[0,0], 53:[0,0]}
    Reward_Contributor_5 = { 2:[ Reward[2], Reward[11], 0 ]}
    
    ##Strade date da 5 contributors 
    Strada_1=['Via_Foria_0_1', '-gneE221_1', '-gneE231_1', 'Vico_SantaMaria_1', 'Via_Veterinaia_1_1', 'Via_Veterinaia_2_1', 'gneE184_1', 'gneE228_1', 'gneE159_1', 'Via_Miano_1']
    Strada_2=['Via_Foria_0_1', 'Via_Foria_1_1', '-gneE182_1', 'Vico_SantaMaria_1', 'Via_Veterinaia_1_1', 'Via_Veterinaia_2_1', 'gneE177_1', 'gneE127_0', 'gneE181_1','gneE62_1']
    Strada_3=['Via_Foria_0_1', 'Via_Foria_1_1', 'Via_Foria_2_1', 'Via_Veterinaia_0_1', 'Via_Veterinaia_1_1', 'Via_SantaMaria_DegliAngeli_1','gneE231_1', 'gneE221_1','Via_Foria_1_1', 'Via_Foria_2_1']
    Strada_4=['Via_Foria_0_1', 'Via_Foria_1_1', 'Via_Foria_2_1', 'Via_Foria_5_1', 'Via_Foria_6_1',  'Rotonda_4_1', 'Rotonda_5_1', 'gneE129_1', 'gneE181_1', 'Via_Miano_1']
    Strada_5=['Via_Foria_0_1', 'Via_Foria_1_1', 'Via_Foria_2_1', 'Via_Foria_5_1', 'SantAntonio_AbateTerzo_1', 'Rotonda_4_2', 'Rotonda_5_2', 'gneE129_1', 'gneE181_1', 'gneE62_1']
    Strada_6=['Via_Foria_0_1', 'Via_Carbonara_1', 'gneE2_1', 'Corso_Garibaldi_2_1', '416695585_1', 'via_Cesare_Rosaroli_1', '-gneE211_1', 'Salita_Pontenuovo_1_1','Salita_Pontenuovo_2_1', 'Vico_Tutti_i_Santi_1']

    #Creo un vettore con queste strade 
    Strada = [Strada_1, Strada_2, Strada_3, Strada_4, Strada_5, Strada_6]

    #Definisco la destinazione Target 
    Target = 'Via_Miano'
    Target_1 = 'Via_Miano_1'

    ##Strada target che deve seguire il veicolo
    Strada_Target=['Via_Foria_0_1', 'Via_Foria_1_1','Via_Foria_2_1', 'Via_Veterinaia_0_1','Via_Veterinaia_1_1', 'Via_Veterinaia_2_1', 'gneE184_1', 'gneE228_1', 'gneE159_1', 'Via_Miano_1']

    print("La Via dove mi trovo è:" , via)
    while via != Target_1:
        ##Calcolo della probabilità data dalla strada Target
        Indice_Provenienza = Dataset_Strade[Strada_Target[numero_via]]
        Indice_Destinazione =  Dataset_Strade[Strada_Target[numero_via+1]]
        p_target= Probabilita_Target[Indice_Destinazione]
        Input.append(max(p_target))

        ##Prima strada 
        if via in Strada[0]:
            Provenienza_0=Dataset_Strade[Strada[0][numero_via]]
            Destinazione_0=Dataset_Strade[Strada[0][numero_via+1]]
            p_0 = Probabilita_Contributor_0[Destinazione_0]
            Probabilita.append(max(p_0))
            Velocita_media_0 = traci.lane.getLastStepMeanSpeed(Strada[0][numero_via+1]) ##Calcolo la velocità media nel prossimo link
            #if Velocita_media_0 < 1:
            #    Reward[Assegnazione_Reward[Strada[0][numero_via+1]]]= -10 ##Così aggiorno la Reward
            reward_0 = Reward_Contributor_0[Destinazione_0]
            Expectation_0 = np.dot(reward_0,p_0)
            Expectation.append(reward_0)
            ##Calcolo divergenza di Kullback Leibler
            Decisione.append(KL(p_target,p_0)-Expectation_0)

        ##Se il veicolo non si trova nella strada del contributor allora inserisco 100 nel vettore delle reward e 0 in quello dell'expectation
        if via not in Strada[0]:
            Decisione.append(100)
            Expectation.append(0)
            Probabilita.append(0)
            p_0 = 0
            reward_0 = 0

        ##Seconda strada 
        if via in Strada[1]:    
            Provenienza_1=Dataset_Strade[Strada[1][numero_via]]
            Destinazione_1=Dataset_Strade[Strada[1][numero_via+1]]
            p_1 = Probabilita_Contributor_1[Destinazione_1]
            Probabilita.append(max(p_1))
            Velocita_media_1 = traci.lane.getLastStepMeanSpeed(Strada[1][numero_via+1]) ##Calcolo la velocità media nel prossimo link
            #if Velocita_media_1 < 1:
            #    Reward[Assegnazione_Reward[Strada[1][numero_via+1]]]= -10 ##Così aggiorno la Reward
            reward_1 = Reward_Contributor_1[Destinazione_1]
            Expectation_1 = np.dot(reward_1,p_1)
            Expectation.append(reward_1)
            ##Calcolo divergenza di Kullback Leibler
            Decisione.append(KL(p_target,p_1)-Expectation_1)

        if via not in Strada[1]:
            Decisione.append(100)
            Expectation.append(0)
            Probabilita.append(0)
            p_1 = 0
            reward_1 = 0

        ##Terza strada 
        if via in Strada[2]:
            Provenienza_2 = Dataset_Strade[Strada[2][numero_via]]
            Destinazione_2 = Dataset_Strade[Strada[2][numero_via+1]]
            p_2 = Probabilita_Contributor_2[Destinazione_2]
            Probabilita.append(max(p_2))
            Velocita_media_2 = traci.lane.getLastStepMeanSpeed(Strada[2][numero_via+1])
            #if Velocita_media_2 < 1:
            #    Reward_Contributor_2[Destinazione_2][0]=-10  ##Così la inserisco nel dizionario
            #    Reward[Assegnazione_Reward[Strada[2][numero_via+1]]]= -10 ##Così aggiorno la Reward
            reward_2 = Reward_Contributor_2[Destinazione_2]
            Expectation_2 = np.dot(reward_2,p_2)
            Expectation.append(reward_2)
            ##Calcolo divergenza di Kullback Leibler
            Decisione.append(KL(p_target,p_2)-Expectation_2)
                    
        if via not in Strada[2]:
            Decisione.append(100)
            Expectation.append(0)
            Probabilita.append(0)
            p_2 = 0
            reward_2 = 0

        ##Quarta strada 
        if via in Strada[3]:
            Provenienza_3=Dataset_Strade[Strada[3][numero_via]]
            Destinazione_3=Dataset_Strade[Strada[3][numero_via+1]]
            p_3 = Probabilita_Contributor_3[Destinazione_3]
            Probabilita.append(max(p_3))
            Velocita_media_3 = traci.lane.getLastStepMeanSpeed(Strada[3][numero_via+1]) ##Calcolo la velocità media nel prossimo link
            #if Velocita_media_3 < 1:
            #    Reward[Assegnazione_Reward[Strada[3][numero_via+1]]]= -10 ##Così aggiorno la Reward
            reward_3 = Reward_Contributor_3[Destinazione_3]
            Expectation_3 = np.dot(reward_3,p_3)
            Expectation.append(reward_3)
            ##Calcolo divergenza di Kullback Leibler
            Decisione.append(KL(p_target,p_3)-Expectation_3)

        if via not in Strada[3]:
            Decisione.append(100)
            Expectation.append(0)
            Probabilita.append(0)
            p_3 = 0
            reward_3 = 0

        ##Quinta strada 
        if via in Strada[4]:
            Provenienza_4=Dataset_Strade[Strada[4][numero_via]]
            Destinazione_4=Dataset_Strade[Strada[4][numero_via+1]]
            p_4 = Probabilita_Contributor_4[Destinazione_4]
            Velocita_media_4 = traci.lane.getLastStepMeanSpeed(Strada[4][numero_via+1]) ##Calcolo la velocità media nel prossimo link
            #if Velocita_media_4 < 1:
            #    Reward[Assegnazione_Reward[Strada[4][numero_via+1]]]= -10 ##Così aggiorno la Reward
            reward_4 = Reward_Contributor_4[Destinazione_4]
            Expectation_4 = np.dot(reward_4,p_4)
            Expectation.append(reward_4)
            ##Calcolo divergenza di Kullback Leibler
            Decisione.append(KL(p_target,p_4)-Expectation_4)

        if via not in Strada[4]:
            Decisione.append(100)
            Expectation.append(0)
            Probabilita.append(0)
            p_4 = 0
            reward_4 = 0

        ##Sesta strada 
        if via in Strada[5]:
            Provenienza_5=Dataset_Strade[Strada[5][numero_via]]
            Destinazione_5=Dataset_Strade[Strada[5][numero_via+1]]
            p_5 = Probabilita_Contributor_5[Destinazione_5]
            Probabilita.append(max(p_5))
            Velocita_media_5 = traci.lane.getLastStepMeanSpeed(Strada[5][numero_via+1]) ##Calcolo la velocità media nel prossimo link
            #if Velocita_media_5 < 1:
            #    Reward[Assegnazione_Reward[Strada[5][numero_via+1]]]= -10 ##Così aggiorno la Reward
            reward_5 = Reward_Contributor_5[Destinazione_5]
            Expectation_5 = np.dot(reward_5,p_5)
            Expectation.append(reward_5)
            ##Calcolo divergenza di Kullback Leibler
            Decisione.append(KL(p_target,p_5)-Expectation_5)

        if via not in Strada[5]:
            Decisione.append(100)
            Expectation.append(0)
            Probabilita.append(0)
            p_5 = 0
            reward_5 = 0
    
            ##Calcolo del contributor di cui dobbiamo seguire la strada    
            Contributor = Decisione.index(min(Decisione))
            #Coefficienti = Passaggio_Valori[Contributor] ##Assegno i coefficienti per il problema di ottimizzazione
            Soddisfazione.append(Expectation[Contributor])
            Contributor_seguito.append(Contributor+1)
            Output.append(Probabilita[Contributor])
                    
            ##Assegnazione della nuova destinazione
            via = Strada[Contributor][numero_via+1]
            Strada_Calcolata.append(via)
            numero_via += 1

            Decisione=[]
            Expectation=[]
            Probabilita=[]
            valori=[]

    return Strada_Calcolata

    
def run():
    """execute the TraCI control loop"""
    step = 0
    
    ##Definisco le matrici di probabilità delle girate data la strada target e la quelle dei contributor
    Probabilita_Target = { 3:[0.01, 0.98, 0.01], 6:[0.01,0.98,0.01], 17:[0.98, 0.01, 0.01], 40:[1], 41: [0.01, 0.99], 42: [0.99, 0.01], 47:[0.99, 0.01], 49:[0.99, 0.01], 51:[0.99, 0.01]}  
    Probabilita_Contributor_0 = { 4:[0.9, 0.09, 0.01], 8:[0.6, 0.39, 0.01], 10:[1], 40:[0.1, 0.9], 41:[0.1,0.9], 42:[0.9, 0.1], 47:[0.99, 0.01], 49:[0.99, 0.01], 51:[0.99, 0.01]}
    Probabilita_Contributor_1 = { 3:[0.05, 0.9, 0.05], 5: [0.8, 0.15, 0.05], 10:[0.2, 0.79, 0.01], 40:[0.15, 0.85], 41:[0.15,0.85], 43:[0.3, 0.7], 44:[1], 46:[0.2,0.8], 53:[0.3,0.7]}
    Probabilita_Contributor_2 = { 3:[0.07, 0.86, 0.07], 6:[0.1,0.8,0.1], 17:[0.9, 0.05, 0.05], 40:[1], 9:[0.9,0.1], 8:[1], 4:[0.6,0.4], 6:[0.1,0.8,0.1]}
    Probabilita_Contributor_3 = { 3:[0.1, 0.8, 0.1], 6:[0.2,0.6,0.2], 16:[0.2,0.6,0.2], 19:[0.8,0.2], 31:[0.1,0.9], 34:[0.1,0.9], 35:[0.1, 0.9], 46:[0.15,0.85], 51:[0.99, 0.01]}
    Probabilita_Contributor_4 = { 3:[0.1, 0.8, 0.1], 6:[0.2,0.6,0.2], 16:[0.2,0.6,0.2], 18:[0.2,0.8], 31:[0.1,0.9], 34:[0.1,0.9], 35:[0.1, 0.9], 46:[0.15,0.85], 53:[0.1,0.9]}
    Probabilita_Contributor_5 = { 2:[0.1,0.1,0.8]}

    ##Dizionario per assegnare le posizioni nella Reward
    Assegnazione_Reward= {'Via_Carbonara_1':26, 'gneE62_1':25, 'gneE127_0':24, 'gneE177_1':23, 'Via_SantaMaria_DegliAngeli_1':22, 'Via_Veterinaia_0_1': 21, 'SantAntonio_AbateTerzo_1':20, 'gneE181_1':19, 'gneE129_1':18, 'Rotonda_5_2':17, 'Rotonda_4_2':16, 'Via_Foria_6_1':15, 'Via_Foria_5_1':14, 'Via_Foria_2_1':13, '-gneE182_1':12, 'gneE159_1':9,'Via_Miano_1':10, 'Via_Foria_1_1':11,'gneE228_1':8, 'Via_Foria_0_1':1, '-gneE221_1':2, '-gneE231_1':3, 'Vico_SantaMaria_1':4, 'Via_Veterinaia_1_1':5, 'Via_Veterinaia_2_1':6, 'gneE184_1':7} 
    
    #Vettore delle Reward
    Reward = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
    Reward[2]  = 0
    Reward[10] = 30 ##Reward della destinazione posta positiva
    Reward[12] = 10
    Reward[13] = 0
    Reward[14] = 0
    Reward[15] = 0
    Reward[20] = 0
    Reward[23] = 10

    ##Definisco le Reward per le girate
    Reward_Contributor_0 = { 4:[ Reward[2], Reward[11], 0 ], 8:[Reward[8],0,0], 10:[Reward[4]], 40:[0, Reward[5] ], 41:[0, Reward[6]], 42:[Reward[7], 0], 47:[Reward[8], 0], 49:[Reward[9], 0], 51:[Reward[10],0]}
    Reward_Contributor_1 = { 3:[ Reward[2], Reward[11], 0 ], 5:[Reward[12],0,0], 10:[0,Reward[4],0], 40:[0,Reward[5]], 41:[0,Reward[6]], 43:[0, Reward[23]], 44:[Reward[24]], 46:[0, Reward[25]],53:[0, Reward[19]] }
    Reward_Contributor_2 = { 3:[ Reward[2], Reward[11], 0 ], 6:[0,Reward[13],0], 17:[Reward[21],0,0], 40:[Reward[5]], 9:[0,0], 8:[0], 4:[0,0], 6:[0,0,0] }
    Reward_Contributor_3 = { 3:[ Reward[2], Reward[11], 0 ], 6:[0,Reward[13],0], 16:[0,Reward[14],0], 19:[Reward[15],0], 31:[0,0], 34:[0,0], 35:[0, 0], 46:[0,0], 51:[0, 0]}
    Reward_Contributor_4 = { 3:[ Reward[2], Reward[11], 0 ], 6:[0,Reward[13],0], 16:[0,Reward[14],0], 18:[0,Reward[20]], 31:[0,0], 34:[0,0], 35:[0, 0], 46:[0,0], 53:[0,0]}
    Reward_Contributor_5 = { 2:[ Reward[2], Reward[11], 0 ]}
    
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

    #Definizione variabili 
    partenza=6000
    arrivo=6000
    Distanza_in_elettrico=0
    Numero_Via=0
    Via=[]
    Velocita=[]
    Emissioni=[]
    Strade_Elettriche=[]
    Strade_Non_Elettriche=[]
    Distanza_Percorsa=[]
    Strada_Seguita=[]
    Stato_Batteria=[]
    Modalita_Elettrica=[]
    Decisione=[]
    Soddisfazione=[]
    Expectation=[]
    Contributor_seguito=[]
    Input=[]
    Probabilita=[]
    Output=[]
    Densita_Pedoni=[]
    p_target=[]
    Strada_calcolata=[]

    ##Strade date da 5 contributors 
    Strada_1=['Via_Foria_0_1', '-gneE221_1', '-gneE231_1', 'Vico_SantaMaria_1', 'Via_Veterinaia_1_1', 'Via_Veterinaia_2_1', 'gneE184_1', 'gneE228_1', 'gneE159_1', 'Via_Miano_1']
    Strada_2=['Via_Foria_0_1', 'Via_Foria_1_1', '-gneE182_1', 'Vico_SantaMaria_1', 'Via_Veterinaia_1_1', 'Via_Veterinaia_2_1', 'gneE177_1', 'gneE127_0', 'gneE181_1','gneE62_1']
    Strada_3=['Via_Foria_0_1', 'Via_Foria_1_1', 'Via_Foria_2_1', 'Via_Veterinaia_0_1', 'Via_Veterinaia_1_1', 'Via_SantaMaria_DegliAngeli_1','gneE231_1', 'gneE221_1','Via_Foria_1_1', 'Via_Foria_2_1']
    Strada_4=['Via_Foria_0_1', 'Via_Foria_1_1', 'Via_Foria_2_1', 'Via_Foria_5_1', 'Via_Foria_6_1',  'Rotonda_4_1', 'Rotonda_5_1', 'gneE129_1', 'gneE181_1', 'Via_Miano_1']
    Strada_5=['Via_Foria_0_1', 'Via_Foria_1_1', 'Via_Foria_2_1', 'Via_Foria_5_1', 'SantAntonio_AbateTerzo_1', 'Rotonda_4_2', 'Rotonda_5_2', 'gneE129_1', 'gneE181_1', 'gneE62_1']
    Strada_6=['Via_Foria_0_1', 'Via_Carbonara_1', 'gneE2_1', 'Corso_Garibaldi_2_1', '416695585_1', 'via_Cesare_Rosaroli_1', '-gneE211_1', 'Salita_Pontenuovo_1_1','Salita_Pontenuovo_2_1', 'Vico_Tutti_i_Santi_1']

    #Creo un vettore con queste strade 
    Strada = [Strada_1, Strada_2, Strada_3, Strada_4, Strada_5, Strada_6]

    #Definisco la destinazione Target 
    Target = 'Via_Miano'
    Target_1 = 'Via_Miano_1'

    ##Strada target che deve seguire il veicolo
    Strada_Target=['Via_Foria_0_1', 'Via_Foria_1_1','Via_Foria_2_1', 'Via_Veterinaia_0_1','Via_Veterinaia_1_1', 'Via_Veterinaia_2_1', 'gneE184_1', 'gneE228_1', 'gneE159_1', 'Via_Miano_1']

    ##Dizionario che associa strada al numero della strada nella matrice delle frequenze di girata
    Dataset_Strade={'Via_Carbonara_1':2, 'gneE62_1':53, 'gneE221_1':4, 'gneE231_1':8,'Via_SantaMaria_DegliAngeli_1':9, 'gneE127_0':44, 'gneE177_1': 43, 'SantAntonio_AbateTerzo_1':18, 'gneE32_1':23, 'gneE31_1':25,'-gneE231_1':8,'-gneE182_1':5,'gneE228_1':47,'gneE184_1':42,'Via_Veterinaia_0_1':17,'Via_Veterinaia_2_1':41,'Via_Veterinaia_1_1':40,'Vico_SantaMaria_1':10,'-gneE231_1':8,'-gneE221_1':4,'gneE129_1':35, 'gneE181_1':46, '-gneE226_1':48, 'gneE159_1':49, 'Via_Miano_1':51,'Rotonda_5_2':34, 'Rotonda_5_1':34,'Rotonda_4_1':31, 'Rotonda_4_2':31, 'Rotonda_3_1':27,'Rotonda_2_1':26,'Rotonda_1_1':22,'Via_Foria_0_1':1,'Via_Foria_1_1':3,'Via_Foria_2_1':6,'Via_Foria_3_1':11,'Via_Foria_4_1':13,'Via_Foria_5_1':16,'Via_Foria_6_1':19,'Via_Foria_7_1':20}

    ##Dizionario per passare i valori al problema di ottimizzazione
    Passaggio_Valori={ 4: [ 1/10, 1/10, 1/10, 1/10, 1/2, 1/10], 3: [ 1/10, 1/10, 1/10, 1/2, 1/10, 1/10], 0: [ 1/2, 1/10, 1/10, 1/10, 1/10, 1/10], 1: [ 1/10, 1/2, 1/10, 1/10, 1/10, 1/10], 2: [ 1/10, 1/10, 1/2, 1/10, 1/10, 1/10] }
    
    ##Dizionario per associare la densità dei pedoni nelle strade
    Dataset= {':2917710999_2':0.9, ':2917710998_3':0.9, ':4965737714_0':1, ':2914658867_3':0.7, 'gneE31':1, 'gneE32':0.9, 'SantAntonio_AbateTerzo':0.7, ':Sensore_20_2':0.5, ':1782416744_3':0.6, ':2917630996_3':0.3, ':2917660081_3':0.1, ':324345142_5':0.6, 'gneE127':0.6, 'gneE177':0.6, ':232346432_2':1, ':3949081571_0':0.3, ':1784920936_3':0.2, '-gneE221':0.2, ':1710611042_13':0.1, ':1710611042_8':0.1, ':4692709982_2':0.4, 'Vico_SantaMaria':0.4, '-gneE231':0.3, ':1784920936_0':0.2, '-gneE182':0.2, ':Sensore_4_13':0.1, ':Sensore_4_8':0.1, ':324345712_2':0.4, ':1243093942_6':0.5, ':1243093942_3':0.5, ':324345142_2':0.9, '-gneE226':0.4, 'gneE181':0.5, 'gneE129':0.9, ':3370489144_1':1, ':3370490096_4':1, ':197624056_2':1, ':3370490094_1':1, ':4965737714_2':1, ':gneJ27_2':1, ':Sensore_20_3':0.5, 'Rotonda_5':1,'Rotonda_4':1,'Rotonda_3':1,'Rotonda_2':1,'Rotonda_1':1, 'Via_Foria_7':1, 'Via_Foria_6':0.9, 'Via_Foria_5':0.5, ':3371504162_4':0.4, ':gneJ24_3':0.7, ':324345712_5':0.7, ':1121869802_2':0.7, ':232346432_4':1, ':232346432_3':1, ':1784920920_2':0.6, ':4692709982_0':0.4, ':3371504162_10':0.4, ':3371504162_5':0.4, 'Via_Veterinaia_0':0.4,'Via_Veterinaia_1':0.5, 'Via_Veterinaia_2':1, 'gneE184':0.7, 'gneE228':0.7, 'gneE159':0.7, 'Via_Miano':1, 'Via_Foria_3':0.3, 'Via_Foria_4':0.4, 'Via_Foria_2':0.1, ':Sensore_4_7':0.1, 'Via_Foria_1': 0.1, ':1710611042_7':0.1, 'Via_Foria_0':0.1}
        
    ##Dizionario per assegnare le strade in cui attivare la modalità elettrica o no
    Assegnazione_Modalita = {36:'gneE221_1', 35:'gneE231_1', 34:'Via_SantaMaria_DegliAngeli_1', 33:'Via_Veterinaia_0_1', 32:'gneE62_1', 31:'gneE127_0', 30:'gneE177_1', 29:'gneE31_1', 28:'gneE32_1', 27:'SantAntonio_AbateTerzo_1', 26:'-gneE226_1', 25:'gneE181_1', 24:'gneE129_1', 23:'Rotonda_5_1', 22:'Rotonda_4_1', 21:'Rotonda_3_1', 20:'Rotonda_2_1', 19:'Rotonda_1_1', 18:'Via_Foria_7_1', 17:'Via_Foria_6_1', 16:'Via_Foria_5_1', 15:'Via_Foria_4_1',14:'Via_Foria_3_1', 13:'Via_Foria_2_1', 12:'-gneE182_1', 11:'Via_Foria_1_1' , 1:'Via_Foria_0_1', 2:'-gneE221_1', 3:'-gneE231_1', 4:'VicoSantaMaria_1', 5:'Via_Veterinaia_1_1', 6:'Via_Veterinaia_2_1', 7:'gneE184_1', 8:'gneE228_1', 9:'gneE159_1', 10:'Via_Miano_1' }

    ##Dizionario per aggiornare il problema di ottimizzazione della batteria
    Assegnazione_Target = {12:'Via_Miano_1', 11:'gneE159_1', 10:'gneE228_1', 9:'gneE184_1', 8:'Via_Veterinaia_2_1', 7:'Via_Veterinaia_1_1', 6:'Via_Veterinaia_0_1', 5:'Via_Foria_4_1', 4:'Via_Foria_3_1', 1:'Via_Foria_0_1', 2:'Via_Foria_1_1', 3:'Via_Foria_2_1'}
    Assegnazione_1 = {9:'Via_Miano_1', 8:'gneE159_1', 7:'gneE228_1', 6:'gneE184_1', 5:'Via_Veterinaia_2_1', 4:'Via_Veterinaia_1_1', 1:'-gneE221_1', 2:'-gneE231_1', 3:'Vico_SantaMaria_1'}
    Assegnazione_2 = {9:'Via_Miano_1', 8:'gneE159_1', 7:'gneE228_1', 6:'gneE184_1', 5:'Via_Veterinaia_2_1', 4:'Via_Veterinaia_1_1', 3:'Vico_SantaMaria_1', 2:'-gneE231_1', 1:'-gneE182_1'}
    Assegnazione_3 = {1:'gneE177_1', 2:'gneE127_1', 3:'gneE181_1', 4:'-gneE226_1', 5:'gneE159_1', 6:'Via_Miano_1' }
    Assegnazione_4 = {13:'Via_Miano_1', 12:'gneE159_1', 11:'-gneE226_1', 10:'gneE181_1',9:'gneE129_1', 8:'Rotonda_5_1', 7:'Rotonda_4_1', 6:'Rotonda_3_1', 1:'Via_Foria_5_1', 2:'Via_Foria_6_1', 3:'Via_Foria_7_1', 4:'Rotonda_1_1', 5:'Rotonda_2_1'}
    Assegnazione_5 = {12:'Via_Miano_1', 11:'gneE159_1', 10:'-gneE226_1', 9:'gneE181_1', 8:'gneE129_1', 7:'Rotonda_5_2', 6:'Rotonda_4_2', 5:'Rotonda_3_1', 1:'SantAntonio_AbateTerzo_1', 2:'gneE32_1', 3:'gneE31_1', 4:'Rotonda_2_1'}
    
    ##Variabili utilizzate per calcolare la frequenza di girata
    #Veicoli=[]
    #Strada=[]
    #Contatore_Veicoli=0
    #Contatore_2=0
    #Contatore_3=0
    #Contatore_4=0

    while traci.simulation.getMinExpectedNumber() > 0:
        traci.simulationStep()

        ##Calcolo istante di partenza e di arrivo del veicolo
        if 'Veicolo_Intelligente' in traci.simulation.getDepartedIDList():
            partenza = step
        if 'Veicolo_Intelligente' in traci.simulation.getArrivedIDList():
            arrivo = step

        ##PRIMO STEP DELL'ALGORITMO DI OTTIMIZZAZIONE 
        if step == partenza: ##Quando il veicolo entra in simulazione
            Via=traci.vehicle.getLaneID("Veicolo_Intelligente") ##Strada dove si trova il veicolo
            
            Strada_Seguita.append(traci.vehicle.getRoadID("Veicolo_Intelligente"))
            Batteria = input ('Inserisci lo stato della batteria del veicolo: ' ) ##inserimento stato della batteria
            Batteria = float(Batteria) ##la rendo un tipo float
            ##Calcolo Strade dove utilizzare la modalità elettrica e le inserisco nei vettori
            #valori=Ottimizzazione_Gekko_per_Veicolo_Intelligente(Batteria) ##risolvo problema di ottimizzazione delle batteria
            valori=Ottimizzazione_Gekko_Caso_Target(Batteria)
            i=0
            ##In base alla risoluzione del probelma di ottimizzazione inserisco le strade in due vettori, quelli dove va attivata la modalità elettrica e dove no
            for i in range (0, len( valori ) ): 
                if valori[i]==[1.0]:
                    Strade_Elettriche.append(Assegnazione_Target[i+1])
                if valori[i]==[0.0]:
                    Strade_Non_Elettriche.append(Assegnazione_Target[i+1])

            if Via == Strada_Target[0]: ##Inizio del primo passo dell'algoritmo di ottimizzazione, se si trova nella prima strada
                    
                ##Calcolo della probabilità di girata della strada Target
                if Via in Strada_Target: ##Se la via si trova nella strada Target
                    Indice_Provenienza = Dataset_Strade[Via] ##Assegno un indice della strada da dove proviene
                    Indice_Destinazione =  Dataset_Strade[Strada_Target[1]] ##Assegno un indice alla strada dove girerà
                    p_target = Probabilita_Target[Indice_Destinazione]
                    Input.append(max(p_target)) #Vettore che serve per l'output, per stampare le probabilità target
                    
                ##Prima strada data dai Contributors 
                if Via in Strada[0]: 
                    ##Calcolo Probabilità di girata con lo stesso procedimento assegnando gli indici corrispondenti nella matrice delle frequenze di girata
                    Provenienza_0 = Dataset_Strade[Strada[0][0]]
                    Destinazione_0 = Dataset_Strade[Strada[0][1]]
                    p_0 = Probabilita_Contributor_0[Destinazione_0]
                    Probabilita.append(max(p_0))
                    reward_0 = Reward_Contributor_0[Destinazione_0] ##Prendo la reward da un vettore dove sono assegnate le reward per i link
                    Expectation_0 = np.dot(reward_0,p_0) ##Calcolo dell'expectation, reward per probabilità
                    Expectation.append(reward_0) ##Creo un vettore che serve per la stampa alla fine
                    ##Calcolo divergenza di Kullback Leibler
                    Decisione.append(KL(p_target,p_0)-Expectation_0)
                    
                ##Seconda strada data dai Contributors                   
                if Via in Strada[1]:
                    ##Stesso procedimento fatto per prima strada
                    ##Calcolo delle probabilità di girata
                    Provenienza_1=Dataset_Strade[Strada[1][0]]
                    Destinazione_1=Dataset_Strade[Strada[1][1]]
                    p_1 = Probabilita_Contributor_1[Destinazione_1]
                    Probabilita.append(max(p_1))
                    reward_1 = Reward_Contributor_1[Destinazione_1]
                    Expectation_1=np.dot(reward_1,p_1)
                    Expectation.append(reward_1)
                    ##Calcolo divergenza di Kullback Leibler
                    Decisione.append(KL(p_target,p_1)-Expectation_1)
                  
                ##Terza strada data dai Contributors 
                if Via in Strada[2]:
                    ##Calcolo Probabilità di girata
                    Provenienza_2=Dataset_Strade[Strada[2][0]]
                    Destinazione_2=Dataset_Strade[Strada[2][1]]
                    p_2 = Probabilita_Contributor_2[Destinazione_2]
                    Probabilita.append(max(p_2))
                    reward_2 = Reward_Contributor_2[Destinazione_2]
                    Expectation_2 = np.dot(reward_2,p_2)
                    Expectation.append(reward_2)
                    ##Calcolo divergenza di Kullback Leibler
                    Decisione.append(KL(p_target,p_2)-Expectation_2)
 
                ##Quarta strada data dai Contributors 
                if Via in Strada[3]:
                    ##Calcolo Probabilità di girata
                    Provenienza_3=Dataset_Strade[Strada[3][0]]
                    Destinazione_3=Dataset_Strade[Strada[3][1]]
                    p_3 = Probabilita_Contributor_3[Destinazione_3]
                    Probabilita.append(max(p_3))
                    reward_3 = Reward_Contributor_3[Destinazione_3]
                    Expectation_3 = np.dot(reward_3,p_3)
                    Expectation.append(reward_3)
                    ##Calcolo divergenza di Kullback Leibler
                    Decisione.append(KL(p_target,p_3)-Expectation_3)

                ##Quinta strada data dai Contributors 
                if Via in Strada[4]:
                    ##Calcolo Probabilità di girata
                    Provenienza_4=Dataset_Strade[Strada[4][0]]
                    Destinazione_4=Dataset_Strade[Strada[4][1]]
                    p_4 = Probabilita_Contributor_4[Destinazione_4]
                    Probabilita.append(max(p_4))
                    reward_4 = Reward_Contributor_4[Destinazione_4]
                    Expectation_4 = np.dot(reward_4,p_4)
                    Expectation.append(reward_4)
                    ##Calcolo divergenza di Kullback Leibler
                    Decisione.append(KL(p_target,p_4)-Expectation_4)

                ##Sesta strada data dai Contributors 
                if Via in Strada[5]:
                    ##Calcolo Probabilità di girata
                    Provenienza_5=Dataset_Strade[Strada[5][0]]
                    Destinazione_5=Dataset_Strade[Strada[5][1]]
                    p_5 = Probabilita_Contributor_5[Destinazione_5]
                    Probabilita.append(max(p_5))
                    reward_5 = Reward_Contributor_5[Destinazione_5]
                    Expectation_5 = np.dot(reward_5,p_5)
                    Expectation.append(reward_5)
                    ##Calcolo divergenza di Kullback Leibler
                    Decisione.append(KL(p_target,p_5)-Expectation_5)


                #Calcolo del contributor da decidere in base alla DKL
                Contributor = Decisione.index(min(Decisione)) ##Prendo il minimo dal vettore creato con DKL meno Expectation
                #Coefficienti = Passaggio_Valori[Contributor] ##Assegno i coefficienti per il problema di ottimizzazione
                Soddisfazione.append(Expectation[Contributor]) ##Assegno in un vettore soddisfazione il valore della Reward, serve per la visualizzazione
                Contributor_seguito.append(Contributor+1) ##Inserisco in un vettore il numero del contributor che viene seguito 
                Output.append(Probabilita[Contributor]) ##Inserisco la probabilita delle svolte della strada seguita dall'agente

                ##Assegnazione della nuova destinazione 
                New_destination = Strada[Contributor][1] ##Calcolo la nuova destinazione
                Numero_Via += 1 ##Incremento questa variabile per capire che ci troviamo nel secondo link
                destination=New_destination[0:len(New_destination)-2] ##Calcolo la nuova destinazione
                traci.vehicle.changeTarget("Veicolo_Intelligente",destination) ##Assegno la nuova destinazione al veicolo
                #print('Probabilita target: ', p_target)
                #print('Probabilita 0: ', p_0)
                #print("Reward 0: ",reward_0)
                #print('Probabilita 1: ', p_1)
                #print("Reward 1: ",reward_1)
                #print('Probabilita 2: ', p_2)
                #print("Reward 2: ",reward_2)
                #print('Probabilita 3: ', p_3)
                #print("Reward 3: ",reward_3)
                #print('Probabilita 4: ', p_4)
                #print("Reward 4: ",reward_4)
                #print('Probabilita 5: ', p_5)
                #print("Reward 5: ",reward_5)
                #print('Vettore Desionale: ',Decisione)
                #print('Nuova destinazione: ',destination)
                #print("Contributor seguito: ",Contributor)
                #print("I coefficienti sono: ", Coefficienti)
                #print("La batteria sta al: ", Batteria, "%")
                
                ##Azzeramento dei vettori dopo l'assegnazione
                Decisione=[]
                Expectation=[]
                Probabilita=[]
                valori=[]
                
        ##CALCOLO DEL PASSO ITERATIVO CHE VA AVANTI FINCHè IL VEICOLO NON ARRIVA A DESTINAZIONE
        if step >= partenza and step < arrivo and New_destination != Target_1: ##Passo iterativo, continua finchè la destinazione non è la destinazione target
            Via = traci.vehicle.getLaneID("Veicolo_Intelligente") ##Restituisce la Via dove si trova il veicolo
            
            if Via == New_destination: ##Quando il veicolo si trova nella nuova destinazione assegnata
                
                Strada_Seguita.append(traci.vehicle.getRoadID("Veicolo_Intelligente"))
                if Via == '-gneE221_1':
                    valori = Ottimizzazione_Gekko_Caso_1(Stato_Batteria[-1]) ##risolvo problema di ottimizzazione delle batteria
                    i=0
                    Strade_Elettriche=[]
                    Strade_Non_Elettriche=[]
                    ##In base alla risoluzione del probelma di ottimizzazione inserisco le strade in due vettori, quelli dove va attivata la modalità elettrica e dove no
                    for i in range (0, len( valori ) ): 
                        if valori[i]==[1.0]:
                            Strade_Elettriche.append(Assegnazione_1[i+1])
                        if valori[i]==[0.0]:
                            Strade_Non_Elettriche.append(Assegnazione_1[i+1])
                if Via == '-gneE182_1':
                    valori = Ottimizzazione_Gekko_Caso_2(Stato_Batteria[-1]) ##risolvo problema di ottimizzazione delle batteria
                    i=0
                    Strade_Elettriche=[]
                    Strade_Non_Elettriche=[]
                    ##In base alla risoluzione del probelma di ottimizzazione inserisco le strade in due vettori, quelli dove va attivata la modalità elettrica e dove no
                    for i in range (0, len( valori ) ): 
                        if valori[i]==[1.0]:
                            Strade_Elettriche.append(Assegnazione_2[i+1])
                        if valori[i]==[0.0]:
                            Strade_Non_Elettriche.append(Assegnazione_2[i+1])
                if Via == 'gneE177_1':
                    valori = Ottimizzazione_Gekko_Caso_3(Stato_Batteria[-1]) ##risolvo problema di ottimizzazione delle batteria
                    i=0
                    Strade_Elettriche=[]
                    Strade_Non_Elettriche=[]
                    ##In base alla risoluzione del probelma di ottimizzazione inserisco le strade in due vettori, quelli dove va attivata la modalità elettrica e dove no
                    for i in range (0, len( valori ) ): 
                        if valori[i]==[1.0]:
                            Strade_Elettriche.append(Assegnazione_3[i+1])
                        if valori[i]==[0.0]:
                            Strade_Non_Elettriche.append(Assegnazione_3[i+1])
                if Via == 'Via_Foria_5_1':
                    valori = Ottimizzazione_Gekko_Caso_4(Stato_Batteria[-1]) ##risolvo problema di ottimizzazione delle batteria
                    i=0
                    Strade_Elettriche=[]
                    Strade_Non_Elettriche=[]
                    ##In base alla risoluzione del probelma di ottimizzazione inserisco le strade in due vettori, quelli dove va attivata la modalità elettrica e dove no
                    for i in range (0, len( valori ) ): 
                        if valori[i]==[1.0]:
                            Strade_Elettriche.append(Assegnazione_4[i+1])
                        if valori[i]==[0.0]:
                            Strade_Non_Elettriche.append(Assegnazione_4[i+1])
                if Via == 'SantAntonio_AbateTerzo_1':
                    valori = Ottimizzazione_Gekko_Caso_5(Stato_Batteria[-1]) ##risolvo problema di ottimizzazione delle batteria
                    i=0
                    Strade_Elettriche=[]
                    Strade_Non_Elettriche=[]
                    ##In base alla risoluzione del probelma di ottimizzazione inserisco le strade in due vettori, quelli dove va attivata la modalità elettrica e dove no
                    for i in range (0, len( valori ) ): 
                        if valori[i]==[1.0]:
                            Strade_Elettriche.append(Assegnazione_5[i+1])
                        if valori[i]==[0.0]:
                            Strade_Non_Elettriche.append(Assegnazione_5[i+1])

                Strada_calcolata = Simulazione_Crowdsourcing(Numero_Via, Via) ##Calcola la nuova strada girando tutto il crowdsourcing
                print("Strada calcolata: ", Strada_calcolata)
                New_destination = Strada_calcolata[0]
                            
                ##Assegnazione della nuova destinazione
                Numero_Via += 1
                destination = New_destination[0:len(New_destination)-2]
                traci.vehicle.changeTarget("Veicolo_Intelligente",destination)
                #print('Probabilita target: ', p_target)
                #print('Probabilita 0: ', p_0)
                #print("Reward 0: ",reward_0)
                #print('Probabilita 1: ', p_1)
                #print("Reward 1: ",reward_1)
                #print('Probabilita 2: ', p_2)
                #print("Reward 2: ",reward_2)
                #print('Probabilita 3: ', p_3)
                #print("Reward 3: ",reward_3)
                #print('Probabilita 4: ', p_4)
                #print("Reward 4: ",reward_4)
                #print('Probabilita 5: ', p_5)
                #print("Reward 5: ",reward_5)
                #print('Vettore Desionale: ',Decisione)
                #print('Nuova destinazione: ',destination)
                #print("Contributor seguito: ",Contributor)
                #print("I Coefficienti sono: ",Coefficienti)
                #print("La batteria sta al: ", Stato_Batteria[-1], "%")

                ##Azzeramento dei vettori alla fine dell'assegnazione
                Decisione=[]
                Expectation=[]
                Probabilita=[]
                valori=[]
                
        ##Quando entro in una nuova strada assegno il colore al veicolo
        if step >= partenza and step < arrivo:
            if traci.vehicle.getLaneID('Veicolo_Intelligente') in Strade_Elettriche:
                traci.vehicle.setColor('Veicolo_Intelligente',(0,255,0))
            if traci.vehicle.getLaneID('Veicolo_Intelligente') in Strade_Non_Elettriche:
                traci.vehicle.setColor('Veicolo_Intelligente',(255,0,0))

        ##Calcolo delle emissioni del veicolo e calcolo dell'evoluzione della Batteria in base al colore dell'auto, l'auto consuma 10% ogni 150 metri
        if step >= partenza and step < arrivo:
            Distanza_Percorsa.append(traci.vehicle.getDistance('Veicolo_Intelligente'))
            if traci.vehicle.getColor('Veicolo_Intelligente') == (255,0,0,255):
                Velocita.append(traci.vehicle.getSpeed("Veicolo_Intelligente"))
                x=traci.vehicle.getSpeed("Veicolo_Intelligente")
                Emissioni.append(x*x*4.29808761 - 101.57881278*x + 843.08022187)
                Densita_Pedoni.append(Dataset[traci.vehicle.getRoadID("Veicolo_Intelligente")])
                Stato_Batteria.append(Batteria)
                Modalita_Elettrica.append(0)
            if traci.vehicle.getColor('Veicolo_Intelligente') == (0,255,0,255):
                Emissioni.append(0)
                distanza = Distanza_Percorsa[len(Distanza_Percorsa)-1]-Distanza_Percorsa[len(Distanza_Percorsa)-2]
                Distanza_in_elettrico += distanza
                Batteria = Batteria - ((distanza/150)*10)
                Stato_Batteria.append(Batteria)
                Densita_Pedoni.append(Dataset[traci.vehicle.getRoadID("Veicolo_Intelligente")])
                Modalita_Elettrica.append(1)

        ##Plot di stato della batteria e delle emissioni
        if step == arrivo:
            plt.figure()
            plt.subplot(4,1,1)
            #plt.plot(Input)
            #plt.title("Probabilita di girata target")
            plt.plot(Emissioni)
            #plt.title("Inquinamento")
            plt.subplot(4,1,2)
            #plt.plot(Output)
            #plt.title("Probabilita di girata in output")
            plt.plot(Stato_Batteria)
            #plt.title("Carica Batteria")
            plt.subplot(4,1,3)
            plt.plot(Modalita_Elettrica)
            #plt.title("Modalita Elettrica")
            #plt.plot(Contributor_seguito)
            #plt.title("Contributor seguito")
            plt.subplot(4,1,4)
            plt.plot(Densita_Pedoni)
            #plt.title("Densità pedonale nei link")
            #plt.plot(Soddisfazione)
            #plt.title("Reward")
            plt.show()

            Strada_Seguita.append("Via_Miano")
            print("La strada seguita dal veicolo è: ", Strada_Seguita)
                
                        
        ##Calcolo delle frequenze di girata ad un incrocio
        #Veicoli.append(traci.simulation.getDepartedIDList())
        #i=0
        #for i in range(0,len(Veicoli[0])):
        #    Strada.append(traci.vehicle.getRoute(Veicoli[0][i]))
        #    if "Vico_SantaMaria" in Strada[0]:
        #        Contatore_Veicoli += 1
        #    if "Vico_SantaMaria" in Strada[0] and "Via_Veterinaia_00" in Strada[0]:
        #        Contatore_2 += 1
        #    if "Vico_SantaMaria" in Strada[0] and "Via_Veterinaia_1" in Strada[0]:
        #        Contatore_3 += 1
        #    if "Rotonda_4" in Strada[0] and "Rotonda_5" in Strada[0]:
        #        Contatore_4 += 1
        #    Strada=[]
        #Veicoli=[]

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
        
    #print(Contatore_Veicoli)
    #print(Contatore_2)
    #print(Contatore_3)
    #print(Contatore_4)
    
    ##Plot dell'inquinamento su una strada ad alta densità pedonale
    i=0
    for i in range(0,len(Inquinamento_1)):
        Inquinamento.append(Inquinamento_1[i]+Inquinamento_2[i])
        
    plt.plot(Inquinamento)
    #plt.title("Pollution without hybrid vehicles mg/m")
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
    traci.start([sumoBinary, "-c", "data/Napoli.sumocfg","--tripinfo-output", "tripinfo.xml",
                  "--no-step-log", "true", "-W", "true",  "--duration-log.disable"],label="master")
    run()
