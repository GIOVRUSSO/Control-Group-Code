from gekko import GEKKO

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

# Equations
#m.Equation(247*x1 + 52*x2 + 95*x3 + 55*x4 + 225*x5 + 166*x6 + 87*x7 + 37*x8 + 372*x9 + 192*x10  <= 1200)
#m.Equation(247*x1 + 52*x2 + 95*x3 + 55*x4 + 218*x11 + 714*x12 + 192*x10 <= 1200)
#m.Equation(247*x1 + 52*x2 + 95*x3 + 484*x13 + 87*x7 + 37*x8 + 372*x9 + 192*x10  <= 1200)
m.Equation(100 - (247/150*10*x1) - (52/150*10*x2) - (95/150*10*x3) - (55/150*10*x4) - (225/150*10*x5) - (166/150*10*x6) - (87/150*10*x7) - (37/150*10*x8) - (372/150*10*x9) - (192/150*10*x10)  >= 35)
m.Equation(100 - (247/150*10*x1) - (52/150*10*x2) - (95/150*10*x3) - (55/150*10*x4) - (218/150*10*x11) - (714/150*10*x12) - (192/150*10*x10) >=35)
m.Equation(100 - (247/150*10*x1) - (52/150*10*x2) - (95/150*10*x3) - (484/150*10*x13) - (87/150*10*x7) - (37/150*10*x8) - (372/150*10*x9) - (192/150*10*x10)  >= 35)
#Objective
m.Obj(-(1/3*(0.5*247*x1 + 0.1*52*x2 + 0.2*95*x3 + 0.2*55*x4 + 0.5*225*x5 + 0.5*166*x6 + 0.3*87*x7 + 0.3*37*x8 + 0.2*372*x9 + 0.1*192*x10) +1/3*( 0.5*247*x1 + 0.1*52*x2 + 0.2*95*x3 + 0.2*55*x4 + 0.8*218*x11 + 0.3*714*x12 + 0.1*192*x10) + 1/3*(0.5*247*x1 + 0.1*52*x2 + 0.2*95*x3 + 0.5*484*x13 + 0.3*87*x7 + 0.3*37*x8 + 0.2*372*x9 + 0.1*192*x10 ) )) # Objective
m.solve() # Solve
print("x1: " + str(x1.value))
print("x2: " + str(x2.value))
print("x3: " + str(x3.value))
print("x4: " + str(x4.value))
print("x5: " + str(x5.value))
print("x6: " + str(x6.value))
print("x7: " + str(x7.value))
print("x8: " + str(x8.value))
print("x9: " + str(x9.value))
print("x10: " + str(x10.value))
print("x11: " + str(x11.value))
print("x12: " + str(x12.value))
print("x13: " + str(x13.value))
print("Objective: " + str(m.options.objfcnval))
Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value,x13.value)


