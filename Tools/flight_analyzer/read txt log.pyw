import matplotlib
matplotlib.use('QT4Agg')
import time
import numpy as np
import matplotlib.pyplot as plt
import csv
import string
import os, sys
import copy
from toolbox import *
import shutil
import datetime
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
plt.rcParams['animation.ffmpeg_path'] = \
'E:\\prgm\\ffmpeg\\ffmpeg-20151219-git-2dba040-win64-static\\bin\\ffmpeg.exe'
from telemetry_lib import *


################################################################
def norm(x,y,z):

    return np.sqrt(x**2+y**2+z**2)

################################################################

def concatenate(arr1, arr2):

    arr=np.zeros((arr1.shape[0]+arr2.shape[0]), 'd')

    arr[:arr1.shape[0]]=arr1[:]
    arr[arr1.shape[0]:]=arr2[:]

    return arr


###################################

def limit_value(value, limit):

    value[np.where(value>limit)]=limit
    value[np.where(value<-limit)]=-limit

###################################





###################################
#MAIN
###################################

input_dir='F:'
#input_dir='E:\\projet autoentrepreneur\\prise de vue aerienne\\tests\\log files'
nb_subplot_v=2
nb_subplot_h=2
subplot_location=1
fontsize=10
line_len=71
t0=40
tf=None
HEARTBEAT_HZ=80
throttle_offset=0.6*2000+2244
HEARTBEAT_UDB=10
ZKP=0.06
LIMIT_VZ=450.
VZKP=0.1
LIMIT_ACCZ=2500.
VZKI=0.
ACCZKP=0.06
ACCZKI=0.35
MAX_THRUST=17
AIRCRAFT_MASS=1.15
F_SYNTHETIC_THROTTLE=1
MEASURED_TO_EXPECTED_ACCZ=1.
MEASURED_TO_EXPECTED_VZ=1.
plot_accz=1
plot_vz=1
plot_z=1
plot_synthetic_throttle=0
plot_rmat=0
invdeltafilter_sonar=2
SONAR=1
BAROMETER=1
MAX_HOVER_RADIUS = 7


#1987: elevation a 5m avec perche, no prop
#1988: elevation a 5m avec perche, prop
#1990: test GPS: demi tour vers direction opposee
#2013 et 2017: barometer only, hovering except bug at the very end

#2030:hovering below max sonar z
#2031:hovering, max sonar is 450, attempt to go above sonar, bug due to sonar distance=78cm above 550cm
#2032:hovering, max sonar is 150, attempt to go above 150cm, control based on barometer vz, we remain below 550cm, no sonar bug
#2034:hovering, max sonar is 150, attempt to go above 150cm, control based on barometer vz, bug due to sonar distance=78cm above 550cm

#test case for hysteresis on rmat5, and decrease of rmat2=6 with rmat8 increase : 2221, 2222
file_number=2307
plot_name='hover_measured'
savegard_name='target_v_indoor'

#save current log file to disk
now = datetime.datetime.now()
date_time=now.strftime("%Y-%m-%d")
#shutil.copy2('%s\\LOG0%04d.TXT' %(input_dir, file_number), 'E:\\projet autoentrepreneur\\prise de vue aerienne\\tests\\log files\\%s_%d_%s.TXT' %(savegard_name, file_number, date_time))

xmin=None
xmax=None

filename='LOG%05d.TXT' %file_number
#filename='target_v_indoor_2262_2016-07-03.TXT'

tf=find_final_time(filename, input_dir, line_len, 'cpu', tf, HEARTBEAT_UDB)

##imu_z=extract_var(filename, input_dir, 43, line_len, 'imz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
cpu=extract_var(filename, input_dir, line_len, 'cpu')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
time=0.001*extract_var(filename, input_dir, line_len, 'T')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

time_bis=np.zeros((len(time)))
for i in range(len(time)-8):
    time_bis[i+8]=i*1./HEARTBEAT_UDB

time=time_bis

imu_accz=extract_var(filename, input_dir, line_len, 'accz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

if SONAR:
    sonar_dist=extract_var(filename, input_dir, line_len, 'sond')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
    sonar_height=extract_var(filename, input_dir, line_len, 'sonhtg')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
else:
    sonar_dist=np.zeros((len(time)))
    sonar_height=np.zeros((len(time)))
    
##sonar_dist=clean_sonar(sonar_dist)

accz=extract_var(filename, input_dir, line_len, 'accz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

imu_x=extract_var(filename, input_dir, line_len, 'imx')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
imu_y=extract_var(filename, input_dir, line_len, 'imy')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
imu_z=extract_var(filename, input_dir, line_len, 'imz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
imu_vx=extract_var(filename, input_dir, line_len, 'tx')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
imu_vy=extract_var(filename, input_dir, line_len, 'ty')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
imu_vz=extract_var(filename, input_dir, line_len, 'tz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
wind_vx=extract_var(filename, input_dir, line_len, 'wvx')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
wind_vy=extract_var(filename, input_dir, line_len, 'wvy')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
wind_vz=extract_var(filename, input_dir, line_len, 'wvz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

error_z_integral=extract_var(filename, input_dir, line_len, 'ezi')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
error_vz_integral=extract_var(filename, input_dir, line_len, 'evzi')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
error_accz_integral=extract_var(filename, input_dir, line_len, 'eacczi')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

target_z=extract_var(filename, input_dir, line_len, 'tgz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
target_vz=extract_var(filename, input_dir, line_len, 'tgvz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
target_accz=extract_var(filename, input_dir, line_len, 'tgaccz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
accz_filt=extract_var(filename, input_dir, line_len, 'inaccz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
vz_filt=extract_var(filename, input_dir, line_len, 'invz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
z_filt=extract_var(filename, input_dir, line_len, 'inz')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

##error_x=extract_var(filename, input_dir, line_len, 'ex')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
##error_x_integral=extract_var(filename, input_dir, line_len, 'exi')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
##error_y=extract_var(filename, input_dir, line_len, 'ey')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
##error_y_integral=extract_var(filename, input_dir, line_len, 'eyi')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

throttle=extract_var(filename, input_dir, line_len, 'p1o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
aileron=extract_var(filename, input_dir, line_len, 'p2o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
cpu_load=extract_var(filename, input_dir, line_len, 'cpu')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
waypoint_index=extract_var(filename, input_dir, line_len, 'W')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
goal=extract_var(filename, input_dir, line_len, 'G', 1)[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
waypoint_index=extract_var(filename, input_dir, line_len, 'W')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
#segment_index=extract_var(filename, input_dir, line_len, 'segi')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
lat_gps0=extract_var(filename, input_dir, line_len, 'N')[40*HEARTBEAT_UDB]
long_gps0=extract_var(filename, input_dir, line_len, 'E')[40*HEARTBEAT_UDB]
alt_gps0=extract_var(filename, input_dir, line_len, 'A')[40*HEARTBEAT_UDB]
lat_gps=extract_var(filename, input_dir, line_len, 'N')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
long_gps=extract_var(filename, input_dir, line_len, 'E')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
alt_gps=extract_var(filename, input_dir, line_len, 'A')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

relative_vx=imu_vx-wind_vx
relative_vy=imu_vy-wind_vy
relative_vz=imu_vz-wind_vz

print(imu_vx.shape)
print(imu_vx[0])
print(imu_vy)
print(imu_vz)

imu_velocity_norm=np.sqrt(imu_vx**2+imu_vy**2+imu_vz**2)
wind_velocity_norm=np.sqrt(wind_vx**2+wind_vy**2+wind_vz**2)
relative_velocity_norm=np.sqrt(relative_vx**2+relative_vy**2+relative_vz**2)

if BAROMETER:
    barometer_pressure=extract_var(filename, input_dir, line_len, 'prs')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
    barometer_temperature=extract_var(filename, input_dir, line_len, 'tmp')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
    barometer_altitude=extract_var(filename, input_dir, line_len, 'alt')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
    filt_barometer_altitude = exp_filter(barometer_altitude, 2, HEARTBEAT_HZ)
    barometer_vz=np.diff(filt_barometer_altitude) * HEARTBEAT_UDB
    filt_barometer_vz = exp_filter(barometer_vz, 4, HEARTBEAT_HZ)
else:
    barometer_pressure=np.zeros((len(time)))
    barometer_temperature=np.zeros((len(time)))
    barometer_altitude=np.zeros((len(time)))
    barometer_vz=np.zeros((len(time)))
    filt_barometer_altitude=np.zeros((len(time)))
    filt_barometer_vz=np.zeros((len(time)))

ma=extract_var(filename, input_dir, line_len, 'ma')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
mb=extract_var(filename, input_dir, line_len, 'mb')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
mc=extract_var(filename, input_dir, line_len, 'mc')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

rmat0=extract_var(filename, input_dir, line_len, 'a')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat1=extract_var(filename, input_dir, line_len, 'b')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat2=extract_var(filename, input_dir, line_len, 'c')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat3=extract_var(filename, input_dir, line_len, 'd')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat4=extract_var(filename, input_dir, line_len, 'e')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat5=extract_var(filename, input_dir, line_len, 'f')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat6=extract_var(filename, input_dir, line_len, 'g')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat7=extract_var(filename, input_dir, line_len, 'h')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
rmat8=extract_var(filename, input_dir, line_len, 'i')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

p1o=extract_var(filename, input_dir, line_len, 'p1o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
p2o=extract_var(filename, input_dir, line_len, 'p2o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
p3o=extract_var(filename, input_dir, line_len, 'p3o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
p4o=extract_var(filename, input_dir, line_len, 'p4o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
p5o=extract_var(filename, input_dir, line_len, 'p5o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
p6o=extract_var(filename, input_dir, line_len, 'p6o')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

add1=extract_var(filename, input_dir, line_len, 'add1')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add2=extract_var(filename, input_dir, line_len, 'add2')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add3=extract_var(filename, input_dir, line_len, 'add3')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add4=extract_var(filename, input_dir, line_len, 'add4')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add5=extract_var(filename, input_dir, line_len, 'add5')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add6=extract_var(filename, input_dir, line_len, 'add6')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add7=extract_var(filename, input_dir, line_len, 'add7')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add8=extract_var(filename, input_dir, line_len, 'add8')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]
add9=extract_var(filename, input_dir, line_len, 'add9')[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB]

nb_subplot_v=4
nb_subplot_h=1

#fig = plt.figure(figsize=(16.0, 9.0))
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)

xlabel=''
ylabel=''

fig.subplots_adjust(hspace=0.5)

#ax1 = fig.add_subplot(nb_subplot_v, nb_subplot_h, 1)
ax1.set_title('error', fontweight='bold', fontsize=fontsize)

ymin=-500.
ymax=None
ax1.plot(time, accz, '0.8', marker=None, label='accz')
##ax.plot(time, imu_accz, 'b--', marker=None, label='imu accz')
ax1.plot(time, accz_filt, 'r-', label='accz filt')
##ax.plot(time, expected_accz, 'c-', label='expected accz')
ax1.plot(time, target_accz, 'k-', label='target accz')
#ax.plot(time, target_accz_debug, 'y-', label='target accz debug')
#ax.plot(time, error_accz, 'b-', label='error accz')
ax1.plot(time, throttle-2244, 'g-', label='throttle')
#ax.plot(time, throttle_debug, 'm-', label='throttle debug')

finalize_plot(fig, ax1, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

#ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 2)
ymax=None

ax2.plot(time, imu_velocity_norm, 'm-', label='IMU velocity')
ax2.plot(time, wind_vx, 'b--', label='wind vx')
ax2.plot(time, wind_vy, 'b:', label='wind vy')
ax2.plot(time, wind_velocity_norm, 'b-', label='wind velocity')
ax2.plot(time, relative_velocity_norm, 'r:', label='relative velocity')
ax2.plot(time, vz_filt, 'c-', label=' vz filt')
ax2.plot(time, target_vz, 'k--', label='target vz')
#ax.plot(time, filt_barometer_vz, 'g-', label='barometer vz')
#ax.plot(time, target_vz_debug, 'y-', label='target vz debug')
ax2.plot(time, error_vz_integral, 'g--', label='error vz integral')

finalize_plot(fig, ax2, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_right', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

#ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 3)


ymin=-1000.
ymax=1000.

ax3.plot(time, sonar_height, 'c-', label='sonar_height')
#ax.plot(time, sonar_height, 'm--', label='sonar_height')
ax3.plot(time, z_filt, 'b--', label='z')
ax3.plot(time, 100*imu_z+50, 'b:', label='imu z')
#ax.plot(time, barometer_pressure-1.e5, 'b-', label='pressure')
#ax.plot(time, barometer_temperature, 'go-', label='temperature')
ax3.plot(time, barometer_altitude, '0.8', label='pressure altitude')
ax3.plot(time, filt_barometer_altitude, 'r-', label='filtered pressure altitude (reconstructed)')
#ax.plot(time, add1*100, 'm:', label='add1')
#ax.plot(time, add2*100, 'y-', label='add3')
#ax.plot(time, barometer_temperature/10, 'g-', label='temperature')
ax3.plot(time, target_z, 'k--', label='target height')
ax3.plot(time, error_z_integral, 'g--', label='error z integral')
#ax3.plot(time, (p2o-3200), 'y--', label='p2o')
#ax.plot(time[:-2], expected_z, 'y-', label='expected z')
#ax.plot(time, segment_index, 'm-', label='segment index')

finalize_plot(fig, ax3, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)


ymin=None
ymax=None

#ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 4)
ax4.plot(time, cpu, 'k:', label='cpu')
#ax.plot(time, add1, 'k-', label='gps data age')
#ax.plot(time, add3*100, 'y-', label='is gps valid')
ax4.plot(time, waypoint_index*10, 'm-', label='waypoint index')

ax4.plot(time, imu_x, 'c-', label='imu x')
ax4.plot(time, imu_y, 'y-', label='imu y')

ax4.plot(time, imu_z, 'r-', label='imu z')
ax4.plot(time, (alt_gps-alt_gps0)/100, 'b--', label='gps z')
ax4.plot(time, goal[:,2], 'g:', label='goal z')

ax4.plot(time, imu_vx, 'c--', label='imu vx')
ax4.plot(time, imu_vy, 'y--', label='imu vy')

finalize_plot(fig, ax4, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='.', output_file=plot_name+'_z_dynamic.png', \
                  show_legend=True, legend_type='outer_right', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)



forward_speed=5

fig, ax = plt.subplots()
ax.set_xlim([-300., 300.])
ax.set_ylim([-300., 300.])
ax.plot(goal[10:,0], goal[10:,1], 'go', label='goal')
line_1, = ax.plot([], [], 'bo-')
#line_2, = ax.plot([], [], 'ro-')
wind_velocity, = ax.plot([], [], 'm-')
#throttle, = ax.plot([], [], 'k-')
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    """initialize animation"""
    line_1.set_data([], [])
    #line_2.set_data([], [])
    wind_velocity.set_data([], [])
    #throttle.set_data([], [])
    time_text.set_text('')
    return line_1, wind_velocity, time_text

def animate(i):
    """perform animation step"""
    line_1.set_data([imu_x[i]], [imu_y[i]])
    #line_2.set_data([(lat_gps[i]-lat_gps0)/90], [(long_gps[i]-long_gps0)/90*np.cos(np.pi/4)])
    wind_velocity.set_data([0,wind_vx[i]/10], [0,wind_vy[i]/10])
    #throttle.set_data([0,0.1*(p1o[i]-2000)], [0,0])
    time_text.set_text('time = %.3f s' % time[i])
    return line_1, wind_velocity, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(time)), init_func=init,
                              interval=1000./(HEARTBEAT_UDB * forward_speed), blit=True)
plt.show()









nb_subplot_v=1
nb_subplot_h=1
 
fig = plt.figure(figsize=(16.0, 9.0))

xlabel=''
ylabel=''

fig.subplots_adjust(hspace=0.5)






##xy GPS position control in hover

##KP=1
##HOVER_MAX_ANGLE = 30.
##pitchToWP = -KP * add3 * (add4/MAX_HOVER_RADIUS) * HOVER_MAX_ANGLE / 57.3
##yawToWP = -KP * add5 * (add4/MAX_HOVER_RADIUS) * HOVER_MAX_ANGLE / 57.3
##
##
##ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 1)
##ax.set_title('navigation', fontweight='bold', fontsize=fontsize)

#ax.plot(time, imu_x*16384/MAX_HOVER_RADIUS, 'k-', label='imu_x/MAX_RADIUS')
#ax.plot(time, imu_y*16384/MAX_HOVER_RADIUS, 'b-', label='imu_y/MAX_RADIUS')
#ax.plot(time, add4*16384/MAX_HOVER_RADIUS, 'm-', label='tofinish_line/MAX_RADIUS')
#ax.plot(time, add3, 'k--', label='hold_pitch_dir')
#ax.plot(time, add5, 'b--', label='hold_yaw_dir')

##ax.plot(time, add5, 'k--', label='rmat6')
##ax.plot(time, add3, 'b--', label='dist yaw corr')
##ax.plot(time, sonar_height, 'r--', label='sonar height')
##ax.plot(time, sonar_dist, 'g--', label='sonar dist')





##roll control in hover

ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 1)
ax.set_title('navigation', fontweight='bold', fontsize=fontsize)

ymin=-16384
ymax=16384

int16_to_degrees=90./16387.

rollAngle=  add1
averaged_roll_error = add2
rmat2 = add3
roll_error_instant = add5
error_integral = add6
roll_nav_corr = add7
rollNavDeflection = add8
rmat8 = add9

##corr_rmat6 = rmat6 * (1 - np.abs(rmat8)/16384)**2
##
##rollAngle = np.zeros((len(add1)))
##for i in range(len(add1)):
##    
##    rollAngle[i] = -np.arcsin(rmat2[i]/16384)*128/np.pi
##    if rmat5[i] < 0.:
##        rollAngle[i] = -128-rollAngle[i]
##        
##filt_rollAngle_debug = exp_filter(rollAngle, 10, HEARTBEAT_HZ)

#roll_corr_debug = rmat2 - np.sin(-filt_rollAngle*np.pi/128)*16384

ax.plot(time, rollNavDeflection, 'k-', label='rollNavDeflection')
ax.plot(time, rollAngle, 'y-', label='rollAngle')
ax.plot(time, roll_error_instant, 'b--', label='roll_error_instant')
ax.plot(time, averaged_roll_error, 'g--', label='averaged_roll_error')
ax.plot(time, roll_error_instant+averaged_roll_error, 'g-', label='roll_error')
ax.plot(time, error_integral, 'r-', label='error_integral')
ax.plot(time, roll_nav_corr, 'm-', label='roll_nav_corr')
ax.plot(time, 10*(p2o-3200), 'c-', label='p2o')

finalize_plot(fig, ax, xmin, xmax, -32000, 32000, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)




nb_subplot_v=1
nb_subplot_h=1
 
fig = plt.figure(figsize=(16.0, 9.0))

xlabel=''
ylabel=''

ymin=None
ymax=None

fig.subplots_adjust(hspace=0.5)

ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 1)
ax.set_title('mag field', fontweight='bold', fontsize=fontsize)

##ax.plot(time, rmat6, 'y-', label='rmat6')
##ax.plot(time, sonar_height, 'c-', label='sonar_height')
##ax.plot(time, sonar_dist, 'm-', label='sonar_distance')

##ax.plot(time, mc, 'm-', label='mag field earth')
ax.plot(time, p2o, 'm-', label='p2o')

finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)







##if plot_synthetic_throttle:
##    ax.plot(time, accz-np.mean(accz), '0.8', marker=None, label='accz')
##    ax.plot(time, accz_filt-np.mean(accz_filt), 'r-', label='accz filt')
##    ax.plot(time, expected_accz, 'c-', label='expected accz')
##    ax.plot(time, (vz-np.mean(vz))*(6.28*F_SYNTHETIC_THROTTLE), 'b--', label='vz')
##    ax.plot(time[:-1], expected_vz*(6.28*F_SYNTHETIC_THROTTLE), 'g--', label='expected vz')
##    ax.plot(time, (sonar_dist-np.mean(sonar_dist))*(6.28*F_SYNTHETIC_THROTTLE)**2, 'm-', label='sonar_dist')
##    ax.plot(time, (z_filt-np.mean(z_filt))*(6.28*F_SYNTHETIC_THROTTLE)**2, 'k-', label='z filt')
##    ax.plot(time[:-2], expected_z*(6.28*F_SYNTHETIC_THROTTLE)**2, 'y-', label='expected z')
##    ax.plot(time, throttle, 'k--', label='throttle')
##
##    finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
##                  show_legend=True, legend_type='basic', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)
##
##
##
##if plot_rmat:

##    ax.plot(time, add1, 'g--', marker=None, label='add1')
##    ax.plot(time, p6o, 'k--', marker=None, label='p6o')
##    
##    
##    finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
##                  show_legend=True, legend_type='basic', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)
##
##print ('rms accz', np.std(accz-np.mean(accz)))
##
##ymin=None #2500
##ymax=None #4200

##ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 3)
##fig.subplots_adjust(hspace=0.5)           
##ax.set_title('throttle', fontweight='bold', fontsize=fontsize)
##ax.plot(time, throttle, 'k-o', label='throttle')
##ax.plot(time, throttle_offset-ACCZKP*accz_filt, 'g-', label='accz contrib')
####ax.plot(time, throttle_offset-ACCZKP*VZKP*HEARTBEAT_UDB*(-IMU_zPoint), 'r-', label='vz contrib')
##ax.plot(time, throttle_offset-ACCZKP*VZKP*ZKP*HEARTBEAT_UDB*HEARTBEAT_UDB*error_z, 'b-', label='z contrib')
####ax.plot(time, throttle_offset-ACCZKP*VZKP*ZKP*HEARTBEAT_UDB*HEARTBEAT_UDB*heightIn-ACCZKP*VZKP*HEARTBEAT_UDB*(-IMU_zPoint)-ACCZKP*accz_filt, 'm-', label='total contrib')
##finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
##                  show_legend=True, legend_type='basic', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)

##write(bidon, 'E:\\projet autoentrepreneur\\prise de vue aerienne\\ardupilot\\datalog\\kalman filter debug\\z_zPoint.txt')


