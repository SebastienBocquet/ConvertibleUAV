# matplotlib.use('QT4Agg')
import time
import numpy as np
import matplotlib.pyplot as plt
import toolbox as tool
import datetime
# from matplotlib import animation
# plt.rcParams['animation.ffmpeg_path'] = \
# 'E:\\prgm\\ffmpeg\\ffmpeg-20151219-git-2dba040-win64-static\\bin\\ffmpeg.exe'
import telemetry_lib as lib
import os


################################################################

def concatenate(arr1, arr2):
    """concatenates two arrays"""

    arr = np.zeros((arr1.shape[0] + arr2.shape[0]), 'd')

    arr[:arr1.shape[0]] = arr1[:]
    arr[arr1.shape[0]:] = arr2[:]

    return arr

################################################################


###################################
# MAIN
###################################

input_dir = 'G:'
fontsize = 10
line_len = 65
t0 = 0
tf = None
HEARTBEAT_HZ = 160
throttle_offset = 0.6 * 2000 + 2244
HEARTBEAT_EXPORT = 10
F_SYNTHETIC_THROTTLE = 1
MEASURED_TO_EXPECTED_ACCZ = 1.
MEASURED_TO_EXPECTED_VZ = 1.
LIDAR = 1
SONAR = 1
BAROMETER = 1
EXPORT = 'LIGHT'
SERVO_RANGE = 1000

file_number = 2697
plot_name = 'hover_measured'
savegard_name = 'target_v_indoor'

# save current log file to disk
now = datetime.datetime.now()
date_time = now.strftime("%Y-%m-%d")
# shutil.copy2('%s\\LOG0%04d.TXT' %(input_dir, file_number), \
# 'E:\\projet autoentrepreneur\\prise de vue aerienne\\tests\\log files\\%s_%d_%s.TXT' \
# %(savegard_name, file_number, date_time))

xmin = None
xmax = None

filename = 'LOG%05d.TXT' % file_number

ifile = open(os.path.join(input_dir, filename), "rb")
bidon = ifile.readlines()

tf = lib.find_final_time(bidon, line_len,
                         'cpu', tf, HEARTBEAT_EXPORT)

cpu = lib.extract_var(bidon, line_len, 'cpu')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

time = np.zeros((len(cpu)))
for i in range(len(cpu) - 8):
    time[i + 8] = i * 1. / HEARTBEAT_EXPORT

if SONAR:
    sonar_height = lib.extract_var(bidon, line_len, 'sonh')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
else:
    sonar_height = np.zeros((len(time)))

if LIDAR:
    lidar_height = lib.extract_var(bidon, line_len, 'lidh')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
else:
    lidar_height = np.zeros((len(time)))

voltage = 0.1 * lib.extract_var(bidon,
                line_len, 'aqv')[t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
current = 0.1 * lib.extract_var(bidon,
                line_len, 'aqc')[t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
mAh_used = lib.extract_var(bidon, line_len, 'aqu')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

imu_x = lib.extract_var(bidon, line_len, 'imx')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
imu_y = lib.extract_var(bidon, line_len, 'imy')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
imu_z = lib.extract_var(bidon, line_len, 'imz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
imu_vx = lib.extract_var(bidon, line_len, 'tx')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
imu_vy = lib.extract_var(bidon, line_len, 'ty')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
imu_vz = lib.extract_var(bidon, line_len, 'tz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
goal = lib.extract_var(bidon, line_len, 'G', 1)[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

accz = lib.extract_var(bidon, line_len, 'accz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
target_z = lib.extract_var(bidon, line_len, 'tgz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
target_vz = lib.extract_var(bidon, line_len, 'tgvz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
target_accz = lib.extract_var(bidon, line_len, 'tgaccz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
accz_filt = lib.extract_var(bidon, line_len, 'inaccz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
vz_filt = lib.extract_var(bidon, line_len, 'invz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
z_filt = lib.extract_var(bidon, line_len, 'inz')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
error_z_integral = lib.extract_var(bidon, line_len, 'ezi')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

roll_error = lib.extract_var(bidon, line_len, 'rerr')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
pitch_error = lib.extract_var(bidon, line_len, 'perr')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
yaw_error = lib.extract_var(bidon, line_len, 'yerr')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

if BAROMETER:
    # barometer_pressure=lib.extract_var(bidon, line_len, 'prs')[t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    # barometer_temperature=lib.extract_var(bidon, line_len, 'tmp')[t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    barometer_altitude = lib.extract_var(bidon, line_len, 'alt')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    filt_barometer_altitude = tool.exp_filter(
        barometer_altitude, 2, HEARTBEAT_HZ)
    barometer_vz = np.diff(filt_barometer_altitude) * HEARTBEAT_EXPORT
    filt_barometer_vz = tool.exp_filter(barometer_vz, 4, HEARTBEAT_HZ)
else:
    barometer_pressure = np.zeros((len(time)))
    barometer_temperature = np.zeros((len(time)))
    barometer_altitude = np.zeros((len(time)))
    barometer_vz = np.zeros((len(time)))
    filt_barometer_altitude = np.zeros((len(time)))
    filt_barometer_vz = np.zeros((len(time)))

cpu_load = lib.extract_var(bidon, line_len, 'cpu')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

ma = lib.extract_var(bidon, line_len, 'ma')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
mb = lib.extract_var(bidon, line_len, 'mb')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
mc = lib.extract_var(bidon, line_len, 'mc')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

roll_hover_corr = lib.extract_var(bidon, line_len, 'rco')[t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
pitch_hover_corr = lib.extract_var(bidon, line_len, 'pco')[t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

rmat0 = lib.extract_var(bidon, line_len, 'a')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat1 = lib.extract_var(bidon, line_len, 'b')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat2 = lib.extract_var(bidon, line_len, 'c')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat3 = lib.extract_var(bidon, line_len, 'd')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat4 = lib.extract_var(bidon, line_len, 'e')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat5 = lib.extract_var(bidon, line_len, 'f')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat6 = lib.extract_var(bidon, line_len, 'g')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat7 = lib.extract_var(bidon, line_len, 'h')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
rmat8 = lib.extract_var(bidon, line_len, 'i')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

omega0 = lib.extract_var(bidon, line_len, 'om0')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
omega1 = lib.extract_var(bidon, line_len, 'om1')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
omega2 = lib.extract_var(bidon, line_len, 'om2')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

p1o = lib.extract_var(bidon, line_len, 'p1o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p2o = lib.extract_var(bidon, line_len, 'p2o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p3o = lib.extract_var(bidon, line_len, 'p3o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p4o = lib.extract_var(bidon, line_len, 'p4o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p5o = lib.extract_var(bidon, line_len, 'p5o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p6o = lib.extract_var(bidon, line_len, 'p6o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p7o = lib.extract_var(bidon, line_len, 'p7o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
p8o = lib.extract_var(bidon, line_len, 'p8o')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

is_in_flight = lib.extract_var(bidon, line_len, 'inf')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]    
emergency_landing = lib.extract_var(bidon, line_len, 'eml')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]  
low_battery = lib.extract_var(bidon, line_len, 'lowb')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]  
automatic_landing = lib.extract_var(bidon, line_len, 'autl')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]  
engines_off = lib.extract_var(bidon, line_len, 'engo')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]  

add1 = lib.extract_var(bidon, line_len, 'add1')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
add2 = lib.extract_var(bidon, line_len, 'add2')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
add3 = lib.extract_var(bidon, line_len, 'add3')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
add4 = lib.extract_var(bidon, line_len, 'add4')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
add5 = lib.extract_var(bidon, line_len, 'add5')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
add6 = lib.extract_var(bidon, line_len, 'add6')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
add7 = lib.extract_var(bidon, line_len, 'add7')[
    t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]


if EXPORT == 'EXTRA':

    wind_vx = lib.extract_var(bidon, line_len, 'wvx')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    wind_vy = lib.extract_var(bidon, line_len, 'wvy')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    wind_vz = lib.extract_var(bidon, line_len, 'wvz')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

    waypoint_index = lib.extract_var(bidon, line_len, 'W')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    waypoint_index = lib.extract_var(bidon, line_len, 'W')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    lat_gps0 = lib.extract_var(bidon, line_len, 'N')[
        40 * HEARTBEAT_EXPORT]
    long_gps0 = lib.extract_var(bidon, line_len, 'E')[
        40 * HEARTBEAT_EXPORT]
    alt_gps0 = lib.extract_var(bidon, line_len, 'A')[
        40 * HEARTBEAT_EXPORT]
    lat_gps = lib.extract_var(bidon, line_len, 'N')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    long_gps = lib.extract_var(bidon, line_len, 'E')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]
    alt_gps = lib.extract_var(bidon, line_len, 'A')[
        t0 * HEARTBEAT_EXPORT:tf * HEARTBEAT_EXPORT]

    relative_vx = imu_vx - wind_vx
    relative_vy = imu_vy - wind_vy
    relative_vz = imu_vz - wind_vz

    imu_velocity_norm = np.sqrt(imu_vx**2 + imu_vy**2 + imu_vz**2)
    wind_velocity_norm = np.sqrt(wind_vx**2 + wind_vy**2 + wind_vz**2)
    relative_velocity_norm = np.sqrt(
        relative_vx**2 + relative_vy**2 + relative_vz**2)

throttle = (p1o + p2o + p3o + p4o) / 4


# monitoring of quadricopter motor control

TILT_KI = 0.2
TILT_KP = 0.34
TILT_KD = 0.

YAW_KI = 0.
YAW_KP = 0.45
YAW_KD = 0.

TILT_RATE_KP = 0.12
TILT_RATE_KD = 0.41
YAW_RATE_KP = 0.41

roll_rate = -omega1
pitch_rate = -omega0

desired_roll = -TILT_KP * roll_error
desired_pitch = -TILT_KP * pitch_error
desired_yaw = -YAW_KP * yaw_error

roll_quad_control = -TILT_RATE_KP * (roll_rate - desired_roll)
pitch_quad_control = -TILT_RATE_KP * (pitch_rate - desired_pitch)

yaw_quad_control = 0

pitch_body_frame_control = 3 * ((pitch_quad_control - roll_quad_control) / 4)
roll_body_frame_control = 3 * ((pitch_quad_control + roll_quad_control) / 4)

motor_A = 2000 + yaw_quad_control + pitch_body_frame_control
motor_B = 2000 - yaw_quad_control - roll_body_frame_control
motor_C = 2000 + yaw_quad_control - pitch_body_frame_control
motor_D = 2000 - yaw_quad_control + roll_body_frame_control


# debug of gps waypoint mode in hovering

headingToWP = add1
hovering_roll_order = add3
hovering_pitch_order = add4
goal_x = 0
goal_y = 0
earthYaw = add2
udb_magFieldBody0 = add5
udb_magFieldBody1 = add6
z_baro_filt = add7


nb_subplot_v = 4
nb_subplot_h = 1

# fig = plt.figure(figsize=(16.0, 9.0))
fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)

xlabel = ''
ylabel = ''

fig.subplots_adjust(hspace=0.5)

ax1.set_title('motor control', fontweight='bold', fontsize=fontsize)

ymin = None
ymax = None

ax1.plot(time, motor_A, 'k--', marker=None, label='motorA reconst')
ax1.plot(time, motor_B, 'b--', marker=None, label='motorB reconst')
ax1.plot(time, motor_C, 'g--', marker=None, label='motorC reconst')
ax1.plot(time, motor_D, 'r--', marker=None, label='motorD reconst')
ax1.plot(time, p1o, 'k-', marker=None, label='motorA')
ax1.plot(time, p2o, 'b-', marker=None, label='motorB')
ax1.plot(time, p3o, 'g-', marker=None, label='motorC')
ax1.plot(time, p4o, 'r-', marker=None, label='motorD')

tool.finalize_plot(fig, ax1, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ymin = -16000
ymax = 16000
ax2.plot(time, rmat6, 'k--', marker=None, label='roll_angle')
ax2.plot(time, roll_error, 'b--', marker=None, label='roll error')
ax2.plot(time, roll_rate, 'r-', marker=None, label='roll rate')
ax2.plot(time, roll_quad_control, 'm-',
         marker=None, label='roll control reconst')
tool.finalize_plot(fig, ax2, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)


ymin = -16000
ymax = 16000
ax3.plot(time, pitch_error, 'b--', marker=None, label='pitch error')
ax3.plot(time, pitch_rate, 'r-', marker=None, label='pitch rate')
ax3.plot(time, pitch_quad_control, 'm-',
         marker=None, label='pitch control reconst')
tool.finalize_plot(fig, ax3, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ymin = -33000
ymax = 33000
ax4.plot(time, rmat0, 'k--', marker=None, label='rmat0')
ax4.plot(time, earthYaw, 'k-', marker=None, label='earthYaw')
ax4.plot(time, yaw_error, 'b--', marker=None, label='yaw error')
ax4.plot(time, -omega2, 'c-', marker=None, label='omega gyro')
ax4.plot(time, -YAW_RATE_KP * (-omega2 - desired_yaw),
         'm--', marker=None, label='yaw control reconst')

tool.finalize_plot(fig, ax4, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)


ymin = None
ymax = None
fig, (ax) = plt.subplots(1, 1, sharex=True)
ax.plot(time, ma, 'k-', marker=None, label='ma')
ax.plot(time, mb, 'b-', marker=None, label='mb')
ax.plot(time, mc, 'g-', marker=None, label='mc')
ax.plot(time, (180. / np.pi) * np.arctan2(udb_magFieldBody1, udb_magFieldBody0),
        'r-', marker='', label='mag earth angle')
ax.plot(time, (180. / np.pi) * np.arctan2(add6, add5),
        'r--', marker='', label='mag udb angle')
ax.plot(time, cpu, 'c-', marker='', label='cpu load')
tool.finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)


fig, (ax1, ax2) = plt.subplots(2, 1, sharex=True)

xlabel = ''
ylabel = ''

fig.subplots_adjust(hspace=0.5)

ax1.set_title('gps waypoint mode control',
              fontweight='bold', fontsize=fontsize)

ymin = None
ymax = None

ax1.plot(time, headingToWP, 'b--', marker=None, label='headingToWP')
ax1.plot(time, earthYaw, 'k--', marker=None, label='earthYaw')
ax1.plot(time, (headingToWP-earthYaw), 'g-',
         marker=None, label='heading_toward_goal in udb frame')
ax1.plot(time, hovering_roll_order, 'r-.',
         marker=None, label='roll order toward goal in udb frame')
ax1.plot(time, hovering_pitch_order, 'm-',
         marker=None, label='pitch order toward goal in udb frame')

tool.finalize_plot(fig, ax1, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ax2.plot(time, roll_hover_corr, 'y-',
         marker=None, label='roll correction correction')
ax2.plot(time, pitch_hover_corr, 'c-',
         marker=None, label='pitch hover correction')
ax2.plot(time, 1000*imu_x/10, 'y--', marker=None, label='imy x')
ax2.plot(time, 1000*imu_y/10, 'c--', marker=None, label='imu y')

tool.finalize_plot(fig, ax2, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)


# monitoring of altitude control

fig, (ax1, ax2, ax3, ax4) = plt.subplots(4, 1, sharex=True)

xlabel = ''
ylabel = ''

fig.subplots_adjust(hspace=0.5)

ax1.set_title('error', fontweight='bold', fontsize=fontsize)

ymin = None
ymax = None
ax1.plot(time, accz, '0.8', marker=None, label='accz')
ax1.plot(time, accz_filt, 'r-', label='accz filt')
ax1.plot(time, target_accz, 'k-', label='target accz')

tool.finalize_plot(fig, ax1, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ymin = None
ymax = None

ax2.plot(time, vz_filt, 'c-', label=' vz filt')
ax2.plot(time, target_vz, 'k--', label='target vz')

tool.finalize_plot(fig, ax2, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_right', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ymin = None
ymax = None
ax3.plot(time, z_filt, 'k-', label='z filt')
ax3.plot(time, lidar_height, 'c-', label='lidar height')
ax3.plot(time, sonar_height, 'r-', label='sonar height')
ax3.plot(time, z_baro_filt, 'k--', label='baro height filtered')
ax3.plot(time, barometer_altitude, '0.8', label='pressure altitude')
ax3.plot(time, target_z, 'm--', label='target height')
ax3.plot(time, error_z_integral, 'g--', label='error z integral')

tool.finalize_plot(fig, ax3, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)


ymin = None
ymax = None

ax4.plot(time, p1o, 'k-', marker=None, label='motorA')
ax4.plot(time, p2o, 'b-', marker=None, label='motorB')
ax4.plot(time, p3o, 'g-', marker=None, label='motorC')
ax4.plot(time, p4o, 'r-', marker=None, label='motorD')
ax4.plot(time, SERVO_RANGE * is_in_flight, 'k-', marker=None, label='is in flight')
ax4.plot(time, SERVO_RANGE * automatic_landing, 'b-', marker=None, label='auto land')
ax4.plot(time, SERVO_RANGE * low_battery, 'g-', marker=None, label='low battery')
ax4.plot(time, SERVO_RANGE * emergency_landing, 'r-', marker=None, label='emergency landing')
ax4.plot(time, SERVO_RANGE * engines_off, 'g--', marker=None, label='engines off')

tool.finalize_plot(fig, ax4, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)


fig, (ax1, ax2, ax3) = plt.subplots(3, 1, sharex=True)

xlabel = ''
ylabel = ''

fig.subplots_adjust(hspace=0.5)

ax1.set_title('energy draw', fontweight='bold', fontsize=fontsize)

ymin = None
ymax = None

ax1.plot(time, voltage, 'k-', marker=None, label='battery voltage')

tool.finalize_plot(fig, ax1, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ax2.plot(time, current, 'k-', marker=None, label='battery current')

tool.finalize_plot(fig, ax2, xmin, xmax, 0., 80., xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ax3.plot(time, mAh_used, 'k-', marker=None, label='battery conso (mAh)')

tool.finalize_plot(fig, ax3, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='',
                   show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)


# forward_speed=5
#
# fig, ax = plt.subplots()
# ax.set_xlim([-300., 300.])
# ax.set_ylim([-300., 300.])
# ax.plot(goal[10:,0], goal[10:,1], 'go', label='goal')
# line_1, = ax.plot([], [], 'bo-')
# line_2, = ax.plot([], [], 'ro-')
# wind_velocity, = ax.plot([], [], 'm-')
# #throttle, = ax.plot([], [], 'k-')
# time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)
#
# def init():
#    """initialize animation"""
#    line_1.set_data([], [])
#    #line_2.set_data([], [])
#    wind_velocity.set_data([], [])
#    #throttle.set_data([], [])
#    time_text.set_text('')
#    return line_1, wind_velocity, time_text
#
# def animate(i):
#    """perform animation step"""
#    line_1.set_data([imu_x[i]], [imu_y[i]])
#    #line_2.set_data([(lat_gps[i]-lat_gps0)/90], [(long_gps[i]-long_gps0)/90*np.cos(np.pi/4)])
#    wind_velocity.set_data([0,wind_vx[i]/10], [0,wind_vy[i]/10])
#    #throttle.set_data([0,0.1*(p1o[i]-2000)], [0,0])
#    time_text.set_text('time = %.3f s' % time[i])
#    return line_1, wind_velocity, time_text
#
# ani = animation.FuncAnimation(fig, animate, np.arange(1, len(time)), init_func=init,
#                              interval=1000./(HEARTBEAT_EXPORT * forward_speed), blit=True)
# plt.show()
