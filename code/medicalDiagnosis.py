from bnetbase import *

bmi = Variable("BMI", ['~18.5', '~24.0', '~28.0', '<18.5'])
F1 = Factor("P(bmi)", [bmi])
F1.add_values(
    [['~18.5', 0.373],
     ['~24.0', 0.406],
     ['~28.0', 0.204],
     ['<18.5', 0.017]])

co = Variable("Central_Obesity", ['YES', 'NO'])
F2 = Factor("P(co|bmi)", [co, bmi])
F2.add_values(
    [['YES', '~18.5', 0.411],
     ['YES', '~24.0', 0.774],
     ['YES', '~28.0', 0.972],
     ['YES', '<18.5', 0.012],
     ['NO', '~18.5', 0.589],
     ['NO', '~24.0', 0.226],
     ['NO', '~28.0', 0.028],
     ['NO', '<18.5', 0.988]])

ht = Variable("Hypertension", ['YES', 'NO'])
F3 = Factor("P(ht|co,bmi)", [ht,co,bmi])
F3.add_values(
    [['YES', 'YES', '~18.5', 0.373],
     ['YES', 'YES', '~24.0', 0.452],
     ['YES', 'YES', '~28.0', 0.845],
     ['YES', 'YES', '<18.5', 0.126],
     ['YES', 'NO', '~18.5', 0.347],
     ['YES', 'NO', '~24.0', 0.409],
     ['YES', 'NO', '~28.0', 0.731],
     ['YES', 'NO', '<18.5', 0.045],
     ['NO', 'YES', '~18.5', 0.627],
     ['NO', 'YES', '~24.0', 0.548],
     ['NO', 'YES', '~28.0', 0.155],
     ['NO', 'YES', '<18.5', 0.874],
     ['NO', 'NO', '~18.5', 0.653],
     ['NO', 'NO', '~24.0', 0.591],
     ['NO', 'NO', '~28.0', 0.269],
     ['NO', 'NO', '<18.5', 0.955]])

hl = Variable("Hyperlipidemia", ['YES', 'NO'])
F4 = Factor("P(hl|co,bmi)", [hl,co,bmi])
F4.add_values(
    [['YES', 'YES', '~18.5', 0.248],
     ['YES', 'YES', '~24.0', 0.481],
     ['YES', 'YES', '~28.0', 0.655],
     ['YES', 'YES', '<18.5', 0.152],
     ['YES', 'NO', '~18.5', 0.193],
     ['YES', 'NO', '~24.0', 0.426],
     ['YES', 'NO', '~28.0', 0.534],
     ['YES', 'NO', '<18.5', 0.087],
     ['NO', 'YES', '~18.5', 0.752],
     ['NO', 'YES', '~24.0', 0.519],
     ['NO', 'YES', '~28.0', 0.345],
     ['NO', 'YES', '<18.5', 0.848],
     ['NO', 'NO', '~18.5', 0.807],
     ['NO', 'NO', '~24.0', 0.574],
     ['NO', 'NO', '~28.0', 0.466],
     ['NO', 'NO', '<18.5', 0.913]])

vg = Variable("Vegetables", ['<400g/d', '400-500g/d', '>500g/d'])
F5 = Factor("P(vg|hl)", [vg, hl])
F5.add_values(
    [['<400g/d', 'YES', 0.579],
     ['<400g/d', 'NO', 0.283],
     ['400-500g/d', 'YES', 0.284],
     ['400-500g/d', 'NO', 0.324],
     ['>500g/d', 'YES', 0.137],
     ['>500g/d', 'NO', 0.393]])

gd = Variable("Gender", ['Male', 'Female'])
F6 = Factor("P(gd|hl)", [gd, hl])
F6.add_values(
    [['Male', 'YES', 0.571],
     ['Male', 'NO', 0.494],
     ['Female', 'YES', 0.429],
     ['Female', 'NO', 0.506]])

rg = Variable("Region", ['Countryside', 'City'])
F7 = Factor("P(rg|ht,hl)", [rg,ht,hl])
F7.add_values(
    [['Countryside', 'YES', 'YES', 0.416],
     ['Countryside', 'YES', 'NO', 0.371],
     ['Countryside', 'NO', 'YES', 0.598],
     ['Countryside', 'NO', 'NO', 0.543],
     ['City', 'YES', 'YES', 0.584],
     ['City', 'YES', 'NO', 0.629],
     ['City', 'NO', 'YES', 0.402],
     ['City', 'NO', 'NO', 0.457]])

db = Variable("Diabetes", ['YES', 'NO'])
F8 = Factor("P(db|ht,hl)", [db,ht,hl])
F8.add_values(
    [['YES', 'YES', 'YES', 0.693],
     ['YES', 'YES', 'NO', 0.596],
     ['YES', 'NO', 'YES', 0.587],
     ['YES', 'NO', 'NO', 0.221],
     ['NO', 'YES', 'YES', 0.307],
     ['NO', 'YES', 'NO', 0.404],
     ['NO', 'NO', 'YES', 0.413],
     ['NO', 'NO', 'NO', 0.779]])

ag = Variable("Age", ['~60', '~40', '<40'])
F9 = Factor("P(ag|ht,hl)", [ag,ht,hl])
F9.add_values(
    [['~60', 'YES', 'YES', 0.412],
     ['~60', 'YES', 'NO', 0.395],
     ['~60', 'NO', 'YES', 0.375],
     ['~60', 'NO', 'NO', 0.221],
     ['~40', 'YES', 'YES', 0.367],
     ['~40', 'YES', 'NO', 0.334],
     ['~40', 'NO', 'YES', 0.341],
     ['~40', 'NO', 'NO', 0.314],
     ['<40', 'YES', 'YES', 0.221],
     ['<40', 'YES', 'NO', 0.271],
     ['<40', 'NO', 'YES', 0.284],
     ['<40', 'NO', 'NO', 0.465]])

ac = Variable("Activity", ['Insufficient', 'Normal', 'Sufficient'])
F10 = Factor("P(ac|gd,hl,ag)", [ac,gd,hl,ag])
F10.add_values(
    [['Insufficient', 'Male', 'YES', '~60', 0.461],
     ['Insufficient', 'Male', 'YES', '~40', 0.413],
     ['Insufficient', 'Male', 'YES', '<40', 0.386],
     ['Insufficient', 'Male', 'NO', '~60', 0.393],
     ['Insufficient', 'Male', 'NO', '~40', 0.381],
     ['Insufficient', 'Male', 'NO', '<40', 0.291],
     ['Insufficient', 'Female', 'YES', '~60', 0.482],
     ['Insufficient', 'Female', 'YES', '~40', 0.431],
     ['Insufficient', 'Female', 'YES', '<40', 0.416],
     ['Insufficient', 'Female', 'NO', '~60', 0.412],
     ['Insufficient', 'Female', 'NO', '~40', 0.413],
     ['Insufficient', 'Female', 'NO', '<40', 0.312],
     ['Normal', 'Male', 'YES', '~60', 0.294],
     ['Normal', 'Male', 'YES', '~40', 0.335],
     ['Normal', 'Male', 'YES', '<40', 0.360],
     ['Normal', 'Male', 'NO', '~60', 0.298],
     ['Normal', 'Male', 'NO', '~40', 0.336],
     ['Normal', 'Male', 'NO', '<40', 0.371],
     ['Normal', 'Female', 'YES', '~60', 0.295],
     ['Normal', 'Female', 'YES', '~40', 0.331],
     ['Normal', 'Female', 'YES', '<40', 0.363],
     ['Normal', 'Female', 'NO', '~60', 0.299],
     ['Normal', 'Female', 'NO', '~40', 0.338],
     ['Normal', 'Female', 'NO', '<40', 0.378],
     ['Sufficient', 'Male', 'YES', '~60', 0.245],
     ['Sufficient', 'Male', 'YES', '~40', 0.252],
     ['Sufficient', 'Male', 'YES', '<40', 0.254],
     ['Sufficient', 'Male', 'NO', '~60', 0.309],
     ['Sufficient', 'Male', 'NO', '~40', 0.283],
     ['Sufficient', 'Male', 'NO', '<40', 0.338],
     ['Sufficient', 'Female', 'YES', '~60', 0.223],
     ['Sufficient', 'Female', 'YES', '~40', 0.238],
     ['Sufficient', 'Female', 'YES', '<40', 0.221],
     ['Sufficient', 'Female', 'NO', '~60', 0.289],
     ['Sufficient', 'Female', 'NO', '~40', 0.249],
     ['Sufficient', 'Female', 'NO', '<40', 0.310]])

medical = BN('Medical Diagnosis',
         [bmi, co, ht, hl, vg, gd, rg, db, ag, ac],
         [F1, F2, F3, F4, F5, F6, F7, F8, F9, F10])

if __name__ == '__main__':

    for v in [bmi, co, ht, hl, vg, gd, rg, db, ag, ac]:
        print("Variable:", v.name)
        probs = VE(medical, v, [])
        doms = v.domain()
        for i in range(len(probs)):
            print("P({0:} = {1:}) = {2:0.1f}".format(v.name, doms[i], 100*probs[i]))
        print()

    print('**********************')

    v = bmi
    for t in [ht, hl, vg, gd, rg, db, ag, ac]:
        print("Variable:", t.name)
        probs = VE(medical, v, [t, ht])
        probs1 = VE(medical, v, [ht])
        print(probs1)
        print(probs)
        doms = v.domain()
        #for i in range(len(probs)):
        #    for j in range(len(probs)):
        #        print("P({0:} = {1:}|{0:} = {1:}) = {2:0.1f}".format(v.name, t.name, doms[i], 100*probs[i]))
        print()
