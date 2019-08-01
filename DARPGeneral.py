#Import pyomo libraries

from pyomo.environ import *
import pyomo.opt
infinity = float('inf')

#Declare model type
model = AbstractModel()

#Set path to input files
path = "C:\\Users\\tfos11\\DARP Model\\DARP Input Files\\"

#Declare number of requests and number of PTUs and the Big M constant
model.n = Param(initialize = )
model.PTU = Param(initialize = )
model.M = Param(initialize = )

#Declare the node and PTU indices
model.v = RangeSet(0, 2*model.n+1)
model.i = RangeSet(0, 2*model.n+1, within=model.v)
model.j = RangeSet(0, 2*model.n+1, within=model.v)
model.k = RangeSet(1, model.PTU)

#Declare the Pickup, Dropoff, P union D and Depot indices
model.p = RangeSet(1, model.n, within=model.v)
model.d = RangeSet(model.n+1, 2*model.n, within=model.v)
model.pd = RangeSet(1, 2*model.n, within=model.v)
model.dep = RangeSet(0, 2*model.n+1, 2*model.n+1, within=model.v)

#Declare route, service time and vehicle load decision variables
model.x = Var(model.i, model.j, model.k, domain = Binary)
model.service = Var(model.v, model.k, domain = NonNegativeReals)
model.ld = Var(model.v, model.k, domain = NonNegativeReals)

#Declare single indexed paramters: service time, time windows, load

model.di = Param(model.i)
model.ei = Param(model.i)
model.li = Param(model.i)

model.qi = Param(model.i)

#Declare vehicle shift start and end times and capacity
model.MaxTime = Param(model.k, initialize={1:900})
model.MinTime = Param(model.k, initialize={1:0})
model.cap = Param(model.k, initialize={1:1})

#Declare double indexed paramters:travel time and route cost
model.t = Param(model.i, model.j)

model.c = Param(model.i, model.j)

#Load single index data
data = DataPortal()
data.load(filename = path +"di.csv", param = model.di)

data.load(filename = path +"ei.csv", param = model.ei)

data.load(filename = path +"li.csv", param = model.li)

data.load(filename = path +"qi.csv", param = model.qi)

#Load double index data; currently travel time is used for the cost measure

data.load(filename = path +"Tij.csv", param = model.t)

data.load(filename = path +"Tij.csv", param = model.c)

#Declare workload balancing parameters

model.wb2 = Param(initialize = )
model.wb1 = Param(initialize = )

#Objectuve function: Minimize total route cost

def ObjRule(model):
    return sum(model.c[i,j]*model.x[i,j,k] for i in model.i for j in model.j for k in model.k)
model.Obj = Objective(rule=ObjRule, sense=minimize)

#Constraints A through F are the routing constraints

def A_rule(model, i):
    return sum(model.x[i, j, k] for j in model.j for k in model.k) == 1 
model.c_a = Constraint(model.p, rule=A_rule)

def B_rule(model, k):
    return sum(model.x[0, j, k] for j in model.j) == 1 
model.c_b = Constraint(model.k, rule=B_rule)

def C_rule(model, k):
    return sum(model.x[i, 2*model.n+1, k] for i in model.i) == 1 
model.c_c = Constraint(model.k, rule=C_rule)
 
def D_rule(model, i, k):
    return sum(model.x[i, j, k] for j in model.j) - sum(model.x[i+model.n, j, k] for j in model.j) == 0 
model.c_d = Constraint(model.p, model.k, rule=D_rule)

def E_rule(model, i, k):
    return sum(model.x[j, i, k] for j in model.j) - sum(model.x[i, j, k] for j in model.j) == 0 
model.c_e = Constraint(model.pd, model.k, rule=E_rule)

def F_rule(model, i, j, k):
    return model.x[i, i, k] == 0
model.c_f = Constraint(model.i, model.j, model.k, rule=F_rule)

#Constraints G through K are the service time constraints

def G_rule(model, k):
    return model.service[2*model.n+1, k] <= model.MaxTime[k]
model.c_g = Constraint(model.k, rule=G_rule)

def H_rule(model, i, k):
    return model.ei[i] <= model.service[i, k] <= model.li[i]
model.c_h = Constraint(model.i, model.k, rule=H_rule)

def I_rule(model, k):
    return model.service[0, k] >= model.MinTime[k]
model.c_i = Constraint(model.k, rule=I_rule)

def J_rule(model, i, j, k):
    return model.service[j, k] >= model.service[i,k] + model.di[i] + model.t[i,j] - model.M*(1-model.x[i, j, k])
model.c_j = Constraint(model.i, model.j, model.k, rule=J_rule)

def K_rule(model, i, j, k):
    return model.service[i+model.n, k] >= model.service[i,k] + model.di[i] + model.t[i,j] - model.M*(1-model.x[i, j, k])
model.k_f = Constraint(model.p, model.j, model.k, rule=K_rule)

#Constraints L through Q are the load constraints

def L_rule(model, i, k):
    return model.ld[i, k] >= 0
model.c_l = Constraint(model.i, model.k, rule=L_rule)  

def M_rule(model, i, k):
    return model.ld[i, k] >= model.qi[i]
model.c_m = Constraint(model.i, model.k, rule=M_rule) 

def N_rule(model, i, j, k):
    return model.ld[j, k] <= model.ld[i, k] + model.qi[j] + model.M*(1-model.x[i, j, k])
model.c_n = Constraint(model.i, model.j, model.k, rule=N_rule)

def O_rule(model, i, j, k):
    return model.ld[j, k] >= model.ld[i, k] + model.qi[j] - model.M*(1-model.x[i, j, k])
model.c_o = Constraint(model.i, model.j, model.k, rule=O_rule)

def P_rule(model, i, k):
    return model.ld[i, k] <= model.cap[k]
model.c_p = Constraint(model.i, model.k, rule=P_rule)

def Q_rule(model, i, k):
    return model.ld[i, k] <= model.cap[k] + model.qi[i]
model.c_q = Constraint(model.i, model.k, rule=Q_rule)

#Declare workload balancing constraintsL Each vehicle workload must be less then some predetermined value and greater than another

def R_rule (model, k):
   return sum((model.t[i,j] + model.di[i])*model.x[i,j,k] for i in model.i for j in model.j) <= model.wb2
model.c_r = Constraint(model.k, rule=R_rule)

def S_rule (model, k):
    return sum((model.t[i,j] + model.di[i])*model.x[i,j,k] for i in model.i for j in model.j) >= model.wb1
model.c_s = Constraint(model.k, rule=S_rule)

#Create model instance
instance = model.create_instance(data)

#model.write("instance.lp")

#Assign solver and set options if desired
solver = pyomo.opt.SolverFactory('gurobi')
#solver.options["mipgap"]=0.02
solver.options["timelimit"]=21600

#Solve the DARP. Turn tee to False if you do not want to see the status of the solver
results = solver.solve(instance, tee=True)

#See the results
results.write()

#print decision variables along with the node to a text file

with open(path+'results.txt', 'w') as f:
    for i in instance.i:
        for j in instance.j:
            for k in instance.k:
                print(i, j, k, instance.x[i,j, k].value, instance.service[i,k].value, instance.service[j,k].value, instance.ld[i,k].value, instance.t[i,j], instance.di[i], file=f)
