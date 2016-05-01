import matplotlib
matplotlib.use('QT4Agg')
import time
import numpy as np
import matplotlib.pyplot as plt
import csv



def import_data(textfilename):

    ifile  = open(textfilename, "rb")
    reader = csv.reader(ifile)

    bidon=[]
    rownum = 0
    data=[]
    for row in reader:
        # Save header row.
        if rownum == 0:
            header = row
        else:
            data.append(row)
                
        rownum += 1

    bidon.append(header)
    bidon.append(data)
    
    print header
    print data[0]

    ifile.close()

    return bidon


def norm(x,y,z):

    return np.sqrt(x**2+y**2+z**2)


def collect_column(index_start, index_end, step, index_col, data):

    col=[]
    for i in range(index_start,index_end,step):
        col.append(float(data[i][index_col]))

    return np.array(col)


def finalize_plot(fig, ax):
    a=1




###################################
#MAIN
###################################

input_dir='E:\\projet autoentrepreneur\\prise de vue aerienne\\ardupilot\\datalog'
filename='essai gragnagues autonome total 13-12-2014\\LOG00321.csv'
time_start=60 #(s)
duration_to_plot=4 #(min)
skip=1

bidon = import_data('%s\\%s' %(input_dir, filename))
header=bidon[0]
data=bidon[1]
time_end=time_start+duration_to_plot*60


nb_seconds = len(data)/4
print 'duration of flight (s)', nb_seconds

if(time_end > nb_seconds):
    time_end=nb_seconds

#store data up to line_to_plot



time=[]
X=[]
Y=[]
Z=[]
V_SOL_N=[]
V_AERO_N=[]
V_WIND_N=[]
OUT_1=[]
OUT_2=[]
OUT_3=[]
PITCH=[]
ROLL=[]
COEF_V=0


#NEW SYNTAX
time0=float(data[0][0])
time = collect_column(time_start*4, time_end*4, 1, 0, data) - time0
X = collect_column(time_start*4, time_end*4, 1, 48, data)

for i in range(time_start*4,time_end*4,1):
    #time.append(float(data[i][0])-time0)
    #X.append(float(data[i][48]))
    Y.append(float(data[i][49]))
    Z.append(float(data[i][50]))
    V_SOL_N.append(float(data[i][20]))
    V_AERO_N.append(0.01*float(data[i][25]))
    V_wind_X=float(data[i][26])
    V_wind_Y=float(data[i][27])
    V_wind_Z=float(data[i][28])
    V_WIND_N.append(0.01*norm(V_wind_X,V_wind_Y,V_wind_Z))
    #throttle
    OUT_1.append(float(data[i][37]))
    #roll
    OUT_2.append(float(data[i][38]))
    #pitch
    OUT_3.append(float(data[i][39]))
    PITCH.append(float(data[i][16]))
    ROLL.append(float(data[i][17]))
    

f, axarr = plt.subplots()
axarr.plot(X,Y,'b-')



#plot vectors (dedicted function should exist no?)

for line_to_plot in range(time_start*4,time_end*4,skip*4):

    x=float(data[line_to_plot][48])
    y=float(data[line_to_plot][49])
    V_wind_X=float(data[line_to_plot][26])
    V_wind_Y=float(data[line_to_plot][27])
    V_wind_Z=float(data[line_to_plot][28])
    V_wind_N=norm(V_wind_X,V_wind_Y,V_wind_Z)
    tmp_x=[x, x+COEF_V*V_wind_X] #/V_wind_N]
    tmp_y=[y, y+COEF_V*V_wind_Y] #/V_wind_N]
    #axarr[0, 0].annotate(str(0.01*V_WIND_N),xy=(X[-1],Y[-1]))
    axarr.plot(tmp_x,tmp_y,'r-')
    #print 'V_WIND', 0.01*V_wind_N

    V_sol_X=float(data[line_to_plot][57])
    V_sol_Y=float(data[line_to_plot][58])
    V_sol_N=norm(V_sol_X,V_sol_Y,0)
    tmp_x=[x, x+COEF_V*V_sol_X] #/V_sol_N]
    tmp_y=[y, y+COEF_V*V_sol_Y] #/V_sol_N]
    #axarr[0, 0].annotate('%.2f'%(0.01*V_SOL_N),xy=(X[-1],Y[-1]))
    axarr.plot(tmp_x,tmp_y,'g-')


#NEW SYNTAX
fig = plt.figure()
ax=fig.add_subplot(2,1,1)
ax.plot(time,Z,'b-', label='altitude')

ax=fig.add_subplot(2,1,2)
ax.plot(time,V_SOL_N,'b-', label='ground speed')
ax.plot(time,V_AERO_N,'r-', label='aero speed')
ax.plot(time,V_WIND_N,'g-', label='wind speed')


f, axarr = plt.subplots(2)
axarr[0].plot(time,OUT_1,'b-', label='throttle order')
axarr[0].plot(time,OUT_2,'r-', label='roll order')
axarr[0].plot(time,OUT_3,'g-', label='pitch order')
axarr[1].plot(time,ROLL,'r-', label='roll angle')
axarr[1].plot(time,PITCH,'g-', label='pitch angle')
#legend=axarr[0].legend()
#legend=axarr[1].legend()

plt.show()

#plt.ion()


