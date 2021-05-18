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
x14 = m.Var(value=0, lb=0, ub=1, integer=True)
x15 = m.Var(value=0, lb=0, ub=1, integer=True)
x16 = m.Var(value=0, lb=0, ub=1, integer=True)
x17 = m.Var(value=0, lb=0, ub=1, integer=True)

# Equations
#m.Equation(192*x1 + 373*x2 + 37*x3 + 87*x4 + 166*x5 + 225*x6 + 55*x7 + 95*x8 + 52*x9 + 940*x10 + 433*x11 <= 1200)
#m.Equation(192*x1 + 373*x2 + 37*x3 + 87*x4 + 166*x5 + 566*x12 + 71*x13 + 300*x14 + 226*x15 + 217*x16 <= 1200)
#m.Equation(192*x1 + 373*x2 + 37*x3 + 87*x4 + 474*x17 + 95*x8 + 52*x9 + 940*x10 + 433*x11 <= 1200 )
m.Equation(100 - (192/150*10*x1) - (373/150*10*x2) - (37/150*10*x3) - (87/150*10*x4) - (166/150*10*x5) - (225/150*10*x6) - (55/150*10*x7) - (95/150*10*x8) - (52/150*10*x9) - (940/150*10*x10) - (433/150*10*x11) >= 35)
m.Equation(100 - (192/150*10*x1) - (373/150*10*x2) - (37/150*10*x3) - (87/150*10*x4) - (166/150*10*x5) - (566/150*10*x12) - (71/150*10*x13) - (300/150*10*x14) - (226/150*10*x15) - (217/150*10*x16) >= 35)
m.Equation(100 - (192/150*10*x1) - (373/150*10*x2) - (37/150*10*x3) - (87/150*10*x4) - (474/150*10*x17) - (95/150*10*x8) - (52/150*10*x9) - (940/150*10*x10) - (433/150*10*x11) >= 35 )
#Objective
m.Obj(-( 1/3 *(0.1*192*x1 + 0.2*373*x2 + 0.3*37*x3 + 0.3*87*x4 + 0.5*474*x17 + 0.2*95*x8 + 0.2*52*x9 + 0.7*940*x10 + 0.8*433*x11) + 1/3*(0.1*192*x1 + 0.2*373*x2 + 0.3*37*x3 + 0.3*87*x4 + 0.5*166*x5 + 0.8*566*x12 + 0.7*71*x13 + 0.5*300*x14 + 0.5*226*x15 + 0.7*217*x16 + 0.8*433*x11) + 1/3*(0.1*192*x1 + 0.2*373*x2 + 0.3*37*x3 + 0.3*87*x4 + 0.5*166*x5 + 0.5*225*x6 + 0.2*55*x7 + 0.2*95*x8 + 0.2*52*x9 + 0.7*940*x10 + 0.8*433*x11))) # Objective
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
print("x14: " + str(x14.value))
print("x15: " + str(x15.value))
print("x16: " + str(x16.value))
print("x17: " + str(x17.value))
print("Objective: " + str(m.options.objfcnval))
Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value,x13.value,x14.value,x15.value,x16.value,x17.value)

