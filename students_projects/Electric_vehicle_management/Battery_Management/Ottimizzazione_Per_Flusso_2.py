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
x18 = m.Var(value=0, lb=0, ub=1, integer=True)
x19 = m.Var(value=0, lb=0, ub=1, integer=True)
x20 = m.Var(value=0, lb=0, ub=1, integer=True)
x21 = m.Var(value=0, lb=0, ub=1, integer=True)
x22 = m.Var(value=0, lb=0, ub=1, integer=True)
x23 = m.Var(value=0, lb=0, ub=1, integer=True)
x24 = m.Var(value=0, lb=0, ub=1, integer=True)
x25 = m.Var(value=0, lb=0, ub=1, integer=True)

# Equations
#m.Equation(230*x1 + 178*x2 + 157*x3 + 68*x4 + 54*x5 + 277*x6 + 70*x7 + 79*x8 + 22*x9 + 9*x10 + 27*x11 + 38*x12 + 19*x13 + 141*x14 + 85*x15 + 4079*x16 + 50*x17 + 145*x18 <= 1200)
#m.Equation(230*x1 + 178*x2 + 155*x19 + 162*x20 + 127*x21 + 64*x22 + 858*x23 + 230*x24 + 90*x25 + 50*x17 + 145*x18 <= 1200)
m.Equation(100-(230/150*10*x1) -(178/150*10*x2) - (157/150*10*x3) - (68/150*10*x4) - (54/150*10*x5) - (277/150*10*x6) - (70/150*10*x7) - (79/150*10*x8) - (22/150*10*x9) - (9/150*10*x10) - (27/150*10*x11) - (38/150*10*x12) - (19/150*10*x13) - (141/150*10*x14) - (85/150*10*x15) - (4079/150*10*x16) - (50/150*10*x17) - (145/150*10*x18) >= 35)
m.Equation(100-(230/150*10*x1) -(178/150*10*x2) - (155/150*10*x19) - (162/150*10*x20) - (127/150*10*x21) - (64/150*10*x22) - (858/150*10*x23) - (230/150*10*x24) - (90/150*10*x25) - (50/150*10*x17) - (145/150*10*x18) >= 35)
#Objective
m.Obj(-(3/4*(0.1*230*x1 + 0.1*178*x2 + 0.1*157*x3 + 0.3*68*x4 + 0.4*54*x5 + 0.5*277*x6 + 0.9*70*x7 + 79*x8 + 22*x9 + 9*x10 + 27*x11 + 38*x12 + 19*x13 + 0.9*141*x14 + 0.5*85*x15 + 0.3*4079*x16 + 0.2*50*x17 + 145*x18) + 1/4*(0.1*230*x1 + 0.1*178*x2 + 0.2*155*x19 + 0.3*162*x20 + 0.4*127*x21 + 0.6*64*x22 + 858*x23 + 0.7*230*x24 + 0.2*90*x25 + 0.3*50*x17 + 145*x18))) # Objective
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
print("x18: " + str(x18.value))
print("x19: " + str(x19.value))
print("x20: " + str(x20.value))
print("x21: " + str(x21.value))
print("x22: " + str(x22.value))
print("x23: " + str(x23.value))
print("x24: " + str(x24.value))
print("x25: " + str(x25.value))
print("Objective: " + str(m.options.objfcnval))
Variabili=(x1.value,x2.value,x3.value,x4.value,x5.value,x6.value,x7.value,x8.value,x9.value,x10.value,x11.value,x12.value,x13.value,x14.value,x15.value,x16.value,x17.value,x18.value,x19.value,x20.value,x21.value,x22.value,x23.value,x24.value,x25.value)

