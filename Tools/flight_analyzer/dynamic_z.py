import matplotlib
matplotlib.use('QT4Agg')
import time
import numpy as np
import matplotlib.pyplot as plt
import csv
import string
import os, sys
import copy
from matplotlib import animation
import mpl_toolkits.mplot3d.axes3d as p3
plt.rcParams['animation.ffmpeg_path'] = \
'E:\\prgm\\ffmpeg\\ffmpeg-20151219-git-2dba040-win64-static\\bin\\ffmpeg.exe'
from toolbox import *
import math as m



class Aircraft():

    def __init__(self, mass, Enva1, Cempa1, Csauma1, Fla1, Csauma2, Fla2, \
                 Cambmaxa, PosCambmaxa, \
                 Envs, Cemps, Csaums, Fls, \
                 L, l, Xav, xFf, Sf, Dx, Dz, XCG):
        self.mass = mass
        self.Xav=Xav
        self.Dx=Dx
        self.Dz=Dz
        #primary wing is the part of the wing subjected to both propeller + ambiant wind
        self.wing1 = wing(Enva1, Cempa1, Csauma1, Fla1, Cambmaxa, PosCambmaxa, 0.)
        ##second wing is the part of the wing subjected to ambaint wind only
        self.wing2 = wing(Enva2, Csauma1, Csauma2, Fla2, Cambmaxa, PosCambmaxa, 0.)
        self.wing = wing(Enva1+Enva2, Cempa1, Csauma2, 0.5*(Fla1+Fla2), Cambmaxa, PosCambmaxa, 0.)
        self.elevator = wing(Envs, Cemps, Csaums, Fls, 0., 0.5, 0.)
        self.fuselage = fuselage(L, l, Xav, xFf, Sf, self.wing.Cmoy)
        self.Lf = self.fuselage.Xav+self.wing.FlCmoy+0.25*self.wing.Cmoy-self.fuselage.xFf*self.fuselage.longueur
        Aa = self.wing.Lambda / (2 + self.wing.Lambda)
        As = self.elevator.Lambda / (2 + self.elevator.Lambda)
        epsilon = (1. / (2 + self.wing.Lambda)) * (4.5 - (self.Dx + 5 * self.Dz) \
                  / (self.wing.Lambda * self.wing.Cmoy))
        self.xF = 0.25+(self.Dx*self.elevator.surface*As*(1-epsilon)- \
                  self.Lf*self.fuselage.Sf*self.fuselage.Af) \
                  /(self.wing.Cmoy*(self.wing.surface*Aa+self.fuselage.Sf*self.fuselage.Af+ \
                  self.elevator.surface*As*(1-epsilon)))
        self.XF=self.wing.FlCmoy + self.xF * self.wing.Cmoy
        self.XCG=XCG
        self.ms=self.XCG-self.XF
        self.pitchAngle=0.

    def set_calageStab(self, calageStab):
        self.elevator.calage=calageStab

    def set_alphaPlane(self, alphaPlane):
        self.pitchAngle=alphaPlane

    def compute_torseur(self, thrust):

        Fz = 100*(-0.001*self.mass*9.8 + thrust)

        return [Fz]

    def afficher(self):
        print '%%%%%%%%%%%%%%%%%%'
        print 'general parameters'
        print 'masse', self.mass
        print 'distance horizontale foyer aile - foyer plan horizontal', self.Dx
        print 'distance verticale foyer aile - foyer plan horizontal', self.Dz
        print 'distance bord attaque aile - helice', self.Xav
        print 'distance foyer fuselage - foyer aile (m)', self.Lf
        print 'position foyer avion en fraction de la corde moyenne', self.xF
        print 'distance foyer avion - bord d attaque emplanture (m)', self.XF
        print 'marge statique CG - foyer', self.ms
        print 'distance CG - bord d attaque emplanture (m)', self.XCG
        print 'pitchAngle relative to ground', self.pitchAngle*180./np.pi
        print 'thrust', self.thrust
        print '%%%%%%%%%%%%%%%%%%'
        print ''
        print '%%%%%%%%%%%%%%%%%%'
        print 'wing parameters'
        self.wing.afficher()
        print '%%%%%%%%%%%%%%%%%%'
        print 'wing1 parameters'
        self.wing1.afficher()
        print 'wing2 parameters'
        self.wing2.afficher()
        print '%%%%%%%%%%%%%%%%%%'
        print ''
        print '%%%%%%%%%%%%%%%%%%'
        print 'plan horizontal parameters'
        self.elevator.afficher()
        print '%%%%%%%%%%%%%%%%%%'
        print ''
        print '%%%%%%%%%%%%%%%%%%'
        print 'fuselage parameters'
        self.fuselage.afficher()
        print '%%%%%%%%%%%%%%%%%%'
        print ''


class State():

    def __init__(self, nb_ite):
        self.z=np.zeros((nb_ite))
        self.z_dot=np.zeros((nb_ite))
        self.z_dot_dot=np.zeros((nb_ite))

    def init(self, z, z_dot):
        self.z[:]=z
        self.z_dot[:]=z_dot


class PID():

    def __init__(self, kp, ki, natural_frequency):
        self.kp=kp
        self.ki=ki
        self.ddt_coef=2*np.pi*natural_frequency
        self.input=[]
        self.output=[]
        self.error=[]
        self.error_integral=[]

    def compute(self, x, target, error_integral):
        
        output=0;
        tmp=0;

        error=x-target
        error_integral=error_integral+(error/self.ddt_coef)
        tmp = -error*self.kp
        tmp = tmp - error_integral*self.ki
        output = tmp
        output = output*self.ddt_coef;

        self.input.append(x)
        self.output.append(output)
        self.error.append(error)
        self.error_integral.append(error_integral)


class Sensor():

    def __init__(self, offset, drift, noise, delay, accuracy, nb_samples_MP, valicity_range_min=0., valicity_range_max=15000., nb_normal_samples_threshold=1, out_of_range=None):
        self.accuracy=accuracy
        self.offset=offset
        self.drift=drift
        self.noise=noise
        self.delay=delay
        self.output=np.zeros((nb_samples_MP))
        self.count=0
        self.nb_count=0
        self.accuracy_offset=0
        self.valicity_range_min=valicity_range_min
        self.valicity_range_max=valicity_range_max
        self.nb_normal_samples_threshold=nb_normal_samples_threshold
        self.nb_normal_samples=0
        self.is_valid=True
        self.out_of_range=out_of_range

    def compute_response(self, x, ite, delta_t, count):

##        print('sensor response')
##        print(self.count)
##        print(self.nb_count)
##        print(self.accuracy_offset)

        ##after a random number of iteration, apply a random offset
        if self.count > self.nb_count:
            
            self.accuracy_offset=self.accuracy*(np.random.uniform()-0.5)
            self.count=0
            self.nb_count=(int)(np.random.uniform()*80)
        
        output = x[ite-round(self.delay/delta_t)] + self.accuracy_offset + self.offset + self.drift*ite*delta_t + self.noise*(np.random.uniform()-0.5)

        if output < self.valicity_range_min or output > self.valicity_range_max:
            self.nb_normal_samples=0
            self.is_valid=False
            if self.out_of_range != None:
                output=self.out_of_range
        else:
            self.nb_normal_samples+=1

            if self.nb_normal_samples > self.nb_normal_samples_threshold:
                self.is_valid=True
                self.nb_normal_samples=self.nb_normal_samples_threshold

        self.output[count]=output
        self.count+=1
            
        return output

        


class Accu():

    def __init__(self, V0, capa, max_current):
        self.V0=V0
        self.capa=capa
        self.max_current=max_current

    def compute_V(self):
        
        return self.V0


class Engine():

    def __init__(self, Ki, Kv, R, rotor_mass, rotor_internal_radius, rotor_external_radius):
        self.Ki=Ki
        self.Kv=Kv
        self.R=R
        self.rotor_mass=0.050
        self.rotor_internal_radius=0.015
        self.rotor_external_radius=0.02
        self.J=self.rotor_mass*(0.5*self.rotor_external_radius**2 + 0.5*self.rotor_internal_radius**2)
        
        
class ESC():

    def __init__(self, frequency, max_throttle):
        self.frequency=frequency
        self.max_throttle=max_throttle
        
    def compute_Vs(self, throttle, accu):

        return accu.compute_V() * throttle / self.max_throttle

class Propeller():

    def __init__(self, diameter, mean_chord, blade_mass, nb_ite):

        self.Cl=1.23
        self.Cd=1.23
        self.diameter=diameter
        self.mean_chord=mean_chord
        self.blade_mass=blade_mass
        self.Kw=2*0.25*rho*self.Cd*(0.5*diameter)**4*self.mean_chord
        self.J=2*2./3*self.blade_mass*(0.5*diameter)**2
        self.thrust=np.zeros((nb_ite))

    def compute_thrust(self, omega, ite):

        rho=1.2
        thrust=0.33*rho*self.Cl*self.mean_chord*omega**2*(0.5*self.diameter)**3
        self.thrust[ite]=thrust

    def calibrate_Cl(self, thrust, omega):

        self.Cl=np.sqrt(thrust/(0.33*rho*self.Cl*self.mean_chord*(0.5*self.diameter)**3))


class wing():
    """
    -Cmoy: mean chord. Valid for a trapezoidal wing
    -FlCmoy: mean fleche (BA emplanture - BA Cmoy). Valid for a trapezoidal wing
    -XF: distance foyer - BA emplanture
    """

    def __init__(self, Env, Cemp, Csaum, Fl, Cambmax, PosCambmax, calage):

        self.envergure=Env
        self.Cemp=Cemp
        self.Csaum=Csaum
        self.fleche=Fl
        self.airfoil=airfoil(Cambmax, PosCambmax)
        self.eff=Csaum/Cemp
        self.surface=Cemp*(1+self.eff)*Env/2
        self.Cmoy=(2./3) * (self.Cemp**2 + self.Cemp * self.Csaum + self.Csaum**2) / (self.Cemp + self.Csaum)
        self.Lambda=Env**2 / self.surface
        self.FlCmoy=(Fl/3) * (self.Cemp + 2 * self.Csaum) / (self.Cemp + self.Csaum)
        self.XF=self.FlCmoy + 0.25 * self.Cmoy
        self.calage=calage
        self.incidence=0.

    def compute_Cl(self, alpha):
       """compute Cl aile ou stabilisateur (symmetrical airfoil) dans repere sol"""
       alpha_abscissa=np.array([-45., -35., -25., -15., -10., -5., 0, 5, 10, 15, 25, 35, 45, 55, 70, 90, 110, 125, 135]) * np.pi / 180
       Clmax=1.2
       Cl=[-np.cos(alpha_abscissa[6+6]), \
    -np.cos(alpha_abscissa[5+6]), \
    -np.cos(alpha_abscissa[4+6]), \
    -Clmax, \
    -Clmax*10/15, \
    -Clmax*5/15, \
    0., \
    Clmax*5/15, \
    Clmax*10/15, \
    Clmax, \
    np.cos(alpha_abscissa[4+6]), \
    np.cos(alpha_abscissa[5+6]), \
    np.cos(alpha_abscissa[6+6]), \
    np.cos(alpha_abscissa[7+6]), \
    np.cos(alpha_abscissa[8+6]), \
    np.cos(alpha_abscissa[9+6]), \
    -np.cos(alpha_abscissa[8+6]), \
    -np.cos(alpha_abscissa[7+6]), \
    -np.cos(alpha_abscissa[6+6])]

       ##print('alpha', alpha)
       
       polynomial_coef = np.polyfit(alpha_abscissa, Cl, 7)
       return np.poly1d(polynomial_coef)(alpha)

    def compute_Cd(self, alpha):
       """compute Cd aile ou stabilisateur dans repere sol"""
       alpha_abscissa=np.array([-45., -35., -25., -15., -10., -5., 0, 5, 10, 15, 25, 35, 45, 55, 70, 90, 110, 125, 135]) * np.pi / 180
       CdAtCl0=0.02
       CdAtClmax=0.05
       Cd=[np.sin(alpha_abscissa[6+6]), \
    np.sin(alpha_abscissa[5+6]), \
    np.sin(alpha_abscissa[4+6]), \
    CdAtClmax, \
    (CdAtClmax-CdAtCl0)*10/15+CdAtCl0, \
    (CdAtClmax-CdAtCl0)*5/15+CdAtCl0, \
    CdAtCl0, \
    (CdAtClmax-CdAtCl0)*5/15+CdAtCl0, \
    (CdAtClmax-CdAtCl0)*10/15+CdAtCl0, \
    CdAtClmax, \
    np.sin(alpha_abscissa[4+6]), \
    np.sin(alpha_abscissa[5+6]), \
    np.sin(alpha_abscissa[6+6]), \
    np.sin(alpha_abscissa[7+6]), \
    np.sin(alpha_abscissa[8+6]), \
    np.sin(alpha_abscissa[9+6]), \
    np.sin(alpha_abscissa[8+6]), \
    np.sin(alpha_abscissa[7+6]), \
    np.sin(alpha_abscissa[6+6])]

       ##print('alpha', alpha)

       polynomial_coef = np.polyfit(alpha_abscissa, Cd, 7)
       return np.poly1d(polynomial_coef)(alpha)-np.poly1d(polynomial_coef)(0.)+CdAtCl0

    def set_incidence(self, incidence):
       self.incidence=incidence

    def compute_lift(self, incidentWindVelocity):

##       print('relative angle between wind and airfoil', incident_wind_angle - (pitchPlane+self.calage))
##       print('Cl', self.compute_Cl((pitchPlane+self.calage) - incident_wind_angle))

       return 0.5*1.2*incidentWindVelocity**2 * self.compute_Cl(self.incidence+self.calage) * self.surface

    def compute_drag(self, incidentWindVelocity):

##       print('Cd', self.compute_Cd((pitchPlane+self.calage) - incident_wind_angle))

       return -0.5*1.2*incidentWindVelocity**2 * self.compute_Cd(self.incidence+self.calage) * self.surface

    def afficher(self):       
        print 'envergure', self.envergure
        print 'surface alaire', self.surface
        print 'fleche', self.fleche
        print 'corde emplanture', self.Cemp
        print 'corde saumon', self.Csaum
        print 'corde moyenne', self.Cmoy
        print 'allongement', self.Lambda
        print 'distance foyer bord d attaque emplanture', self.XF
        print 'incidence relative to wind in deg', (self.incidence+self.calage)*180./np.pi
        print 'calage', self.calage*180./np.pi
        print 'Cl', self.compute_Cl(self.incidence+self.calage)
        print 'Cd', self.compute_Cd(self.incidence+self.calage)
        self.airfoil.afficher()
        

class airfoil():
    """
    -Cambmax=cambrure maximum en % de la corde,
    -PosCambmax=position de la cambrure max en % de la corde
    """

    def __init__(self, Cambmax, PosCambmax):
        self.Cambmax=Cambmax
        self.PosCambmax=PosCambmax
        self.alpha0=-61 * Cambmax / (100 - PosCambmax)
        self.Cm0 = -61 * Cambmax * PosCambmax / 100000

    def afficher(self):
        print '%%%%%%%%%%%%%%%%%%'
        print 'airfoil parameters'
        print 'cambrure max en % de la corde', self.Cambmax
        print 'position de la cambrure en % de la corde', self.PosCambmax
        print 'incidence de portance nulle (degre)', self.alpha0
        print 'coefficient de moment a portance nulle', self.Cm0
        print '%%%%%%%%%%%%%%%%%%'

class fuselage():

    def __init__(self, L, l, Xav, xFf, Sf, Cmoy):
        self.longueur=L
        self.largeur=l
        self.Xav=Xav
        self.xFf=xFf
        self.Sf=Sf
        self.Af=0.2*(1+Cmoy/l)

    def afficher(self):
        print 'longueur (m)', self.longueur
        print 'largeur (m)', self.largeur
        print 'distance pointe avant - bord d attaque emplanture aile (m)', self.Xav
        print 'distance pointe avant - foyer en fraction de la longueur', self.xFf
        print 'surface portante fuselage (m2)', self.Sf
        print 'coefficient d efficacite fuselage', self.Af
        

class dynamic_rotation():
    
    def __init__(self, nb_ite):
        self.omega=np.zeros((nb_ite))
        self.omega_dot=np.zeros((nb_ite))
        self.omega_eq=0.

    def init_omega(self, omega0):
        self.omega[:]=omega0

    def compute_equilibrium_omega(self, engine, propeller, Vs):

        roots=np.roots([-propeller.Kw, -engine.Ki/(engine.R*engine.Kv), engine.Ki*Vs/engine.R])
        roots_positive=roots[np.where(roots >= 0)]
        if len(roots_positive) != 1:
            print('could not find a single positive solution for equilibrium omega')
            print('roots are', roots)
            sys.exit(1)
        else:
            self.omega_eq=roots_positive[0]
            print('equilibrium omega at Vs max=', self.omega_eq*60./(2*np.pi), 'rpm')

    def compute_angular_acceleration(self, engine, propeller, Vs, ite):

        J=engine.J+propeller.J
        self.omega_dot[ite]=(1./J)*((engine.Ki*Vs/engine.R) - \
                                    (engine.Ki/(engine.R*engine.Kv))*self.omega[ite-1] - \
                                    propeller.Kw*self.omega[ite-1]**2)

    def advance_in_time(self, ite, delta_t):

        self.omega[ite]=self.omega_dot[ite] * delta_t + self.omega[ite-1]


    
def advance_in_time(ite, delta_t, state):

    ##integration of acceleration by second order approximation of z
    state.z[ite]=delta_t**2*state.z_dot_dot[ite]+2*state.z[ite-1]-state.z[ite-2]
    ##estimate of velocity by first order finite difference
    state.z_dot[ite]=(state.z[ite]-state.z[ite-1])/delta_t


class Trajectory():

    def __init__(self, nb_ite):
        self.flight_phases=[]
        self.current_phase=0
        self.altitude=np.zeros((nb_ite))
        self.velocity=np.zeros((nb_ite))

    def compute(self, time, ite):

        if self.flight_phases[self.current_phase].is_terminated(time) and self.current_phase < len(self.flight_phases)-1:
            self.current_phase+=1
            self.flight_phases[self.current_phase].start_time=time

        if self.flight_phases[self.current_phase].is_terminated(time) and self.current_phase == len(self.flight_phases)-1:
            self.altitude[ite]=self.altitude[ite-1]
            self.velocity[ite]=0.
        else:
            self.altitude[ite]=self.flight_phases[self.current_phase].compute_z(time)
            self.velocity[ite]=self.flight_phases[self.current_phase].velocity




class Flight_phase():

    def __init__(self, velocity, start_altitude, end_altitude, duration, manoeuvre_index):
        self.velocity=velocity
        self.start_altitude=start_altitude
        self.end_altitude=end_altitude
        self.duration=duration
        self.start_time=0.
        self.manoeuvre_index=manoeuvre_index
        
    def compute_z(self, time):
        
        return self.velocity * 0.001*(time-self.start_time) + self.start_altitude

    def is_terminated(self, time):

        if self.duration==0:
            if self.velocity > 0. and self.compute_z(time) > self.end_altitude \
               or self.velocity < 0. and self.compute_z(time) < self.end_altitude:
                return True
            else:
                return False
        else:
            if time - self.start_time > self.duration:
                return True
            else:
                return False
                

class Manoeuvre():

    def __init__(self, channel, start_time, end_time, value, nb_ite):
        self.channel=channel
        self.start_time=start_time
        self.end_time=end_time
        self.value=value
        self.init_time=0.
        self.initialized=0
        self.output=np.zeros((nb_ite))

    def update_manoeuvre(self, time, ite):

        if(time-self.init_time > self.start_time and time-self.init_time < self.end_time):
            self.output[ite]=self.value
        
        

        

##MAIN##

##MatrixPilot args
MAX_THRUST=17. ##in N
AIRCRAFT_MASS=1100. ##in g
HEARTBEAT_HZ=80 ##udb loop frequency in Hz
ZKP0=0.93
ZKI=0.
VZKP0=0.8
VZKI=0.
ACCZKP0=0.6
ACCZKI=0.
LIMIT_VZ=450.
LIMIT_ACCZ=2500.
THROTTLE_OFFSET=0.65; ##in fraction of 1
THROTTLE_MIN=0.2
THROTTLE_MAX=1.
START_HEIGHT=130. ##in cm
INV_DELTA_FILTER_SONAR=80
INV_DELTA_FILTER_BARO=2 ##if equal to HEARTBEAT_HZ, no filtering
INV_DELTA_FILTER_VZ=2
INV_DELTA_FILTER_ACCEL=5
hoverwaitvzstable=40 ##in number of udb iterations
SERVORANGE=1000.
EXPECTED_TO_REAL_ACCZ_K=1.
EXPECTED_TO_REAL_VZ_K=1.
THROTTLE_TO_THRUST_DELAY=0. ##in seconds
MAX_SONAR_DISTANCE=450.
NATURAL_FREQUENCY=1.
TARGET_TYPE=1 ##0=z, 1=vz

##propulsion args
rho=1.2
Cd=1.23
blade_radius=0.5*13.5*0.0254
chord=0.015
moment_aero_coef=2*0.25*rho*Cd*blade_radius**4*chord
rotor_mass=0.050
rotor_internal_radius=0.015
rotor_external_radius=0.02
blade_mass=0.010
omega_max=6519*6.28/60
Ki=5.3*moment_aero_coef*omega_max**2/31
Kv=68.  ##omega_max/10
R=0.5  ##10./20
V0=13.
capa=2400.
max_current=20*2.4
omega0=480.
esc_freq=40000


##time advance args
nb_ite=200000
delta_t=1.e-4
simulation_frequency=1./delta_t
simulation_time=nb_ite*delta_t
nb_samples_MP=simulation_time*HEARTBEAT_HZ-1
z0=START_HEIGHT
zdot0=0.
shoot_time = 2.5
z_shoot = -0.

##plot args
xmin=0.
xmax=None
export_file='hover.png'




##init objects

state=State(nb_ite)
state.init(z0, zdot0)

pid_z=PID(ZKP0, ZKI, NATURAL_FREQUENCY)
pid_vz=PID(VZKP0, VZKI, NATURAL_FREQUENCY)
pid_accz=PID(ACCZKP0, ACCZKI, NATURAL_FREQUENCY)

#offset, drift, noise, delay, accuracy, nb_ite, valicity_range_min, valicity_range_max, nb_normal_samples_threshold, out_of_range
sonar=Sensor(0., 0., 0., 0.009, 0., nb_samples_MP, 40., 300., 11, 550.)
baro=Sensor(0., 0., 20., 0.009, 150., nb_samples_MP)
accelero=Sensor(0., 0., 300., 0.005, 0., nb_samples_MP)

sensors=[sonar, accelero]

esc=ESC(esc_freq, 2*SERVORANGE)
accu=Accu(V0, capa, max_current)
engine=Engine(Ki, Kv, R, rotor_mass, rotor_internal_radius, rotor_external_radius)
propeller=Propeller(2*blade_radius, chord, blade_mass, nb_ite)
dyn_rot=dynamic_rotation(nb_ite)
#aircraft=Aircraft(AIRCRAFT_MASS, engine, esc, propeller)
#bras de levier foyer aile - foyer plan horizontal
Dx=0.6
#longueur fuselage (m)
L=1.
#largeur fuselage (m)
l=0.06
#distance verticale entre foyer aile et foyer plan horizontal
Dz=-0.03
#distance pointe avant fuselage - bord d attaque emplanture (m)
Xav=0.15
#position du foyer fuselage en fraction de sa longueur
xFf=0.25
#surface portante du fuselage (m2)
Sf=0.042
#distance CG - brd d'attaque emplanture
XCG=0.08
#propeller diameter in m
propDiameter=0.33

#envergure aile (m)
EnvTotal=1.2
Fleche=0.1

#part of the wing subjected to both propeller wind and ambiant air
Enva1=propDiameter*2./3
Cempa1=0.3
Csauma1=0.27
Fla1=Fleche*Enva1/EnvTotal

#part of the wing only subjected to ambiant air
Enva2=EnvTotal-Enva1
Csauma2=0.15
Fla2=Fleche*Enva2/EnvTotal

#cambrure maximum profil d aile en % de la corde moyenne
Cambmaxa=0.
#position de la cambrure maximum en % de la corde moyenne
PosCambmaxa=30

#envergure plan horizontal (m)
Envs=0.3
Cemps=0.15
Csaums=0.1
Fls=0.05

aircraft=Aircraft(AIRCRAFT_MASS, Enva1, Cempa1, Csauma1, Fla1, Csauma2, Fla2, \
                 Cambmaxa, PosCambmaxa, \
                 Envs, Cemps, Csaums, Fls, \
                 L, l, Xav, xFf, Sf, Dx, Dz, XCG)

dyn_rot.init_omega(omega0)
dyn_rot.compute_equilibrium_omega(engine, propeller, V0)
propeller.compute_thrust(dyn_rot.omega_eq, 0)
print('Thrust at equilibrium omega (%.2f rpm) = %2.f' %(dyn_rot.omega_eq*60/(2*np.pi), propeller.thrust[0]))
print('moment_aero_coef', propeller.Kw)
print('propeller drag momentum at max omega', propeller.Kw*dyn_rot.omega_eq**2, 'N.m')

##vertical trajectory definition
##vz, start_height, end_height, duration
trajecto=Trajectory(nb_samples_MP)
trajecto.flight_phases.append(Flight_phase(0., START_HEIGHT, START_HEIGHT, 3000,-1))
trajecto.flight_phases.append(Flight_phase(200., START_HEIGHT, 1000., 0,-1))
trajecto.flight_phases.append(Flight_phase(0., 1000., 1000., 1000, 0))
trajecto.flight_phases.append(Flight_phase(-200., 1000., START_HEIGHT+300, 0,-1))
trajecto.flight_phases.append(Flight_phase(-150., START_HEIGHT+300, START_HEIGHT+150, 860,-1))
trajecto.flight_phases.append(Flight_phase(-100., START_HEIGHT+150, START_HEIGHT, 1200,-1))
trajecto.flight_phases.append(Flight_phase(0., START_HEIGHT, START_HEIGHT, 3000,-1))
trajecto.flight_phases.append(Flight_phase(0., START_HEIGHT, START_HEIGHT, 1000, 1))

manoeuvres=[\
    [Manoeuvre(1, 0, 1000, 4000, nb_samples_MP),\
     Manoeuvre(2, 0, 1000, 3000, nb_samples_MP)],\
    [Manoeuvre(1, 0, 1000, 2000, nb_samples_MP),\
     Manoeuvre(2, 0, 1000, 0, nb_samples_MP)],\
    ]

filtered_z=0.
filtered_vz=0.
filtered_accz=0.

error_integral_z=0.
error_integral_vz=0.
error_integral_accz=0.

##compute max delay
max_delay=0.
for i in range(len(sensors)):
    if sensors[i].delay > max_delay:
        max_delay=sensors[i].delay
        
nb_delay_samples=(int)(max_delay*simulation_frequency)

count=0
throttle=np.zeros((nb_ite))
throttle_tmp=0.
previous_time=0.
filtered_z_minus_one=0.

##main time loop

for ite in range(2+nb_delay_samples,nb_ite,1):

    time=ite*delta_t

    if time == shoot_time:
        state.z[ite-1]+=z_shoot
        state.z[ite-2]+=z_shoot

    ##throttle to thrust response
    Vs = esc.compute_Vs(throttle[ite-1], accu)
    dyn_rot.compute_angular_acceleration(engine, propeller, Vs, ite)
    dyn_rot.advance_in_time(ite, delta_t)
    propeller.compute_thrust(dyn_rot.omega[ite], ite)
    
    [Fz]=aircraft.compute_torseur(propeller.thrust[ite])
    
    ##dynamic equation along z
    state.z_dot_dot[ite]=Fz

    ##advance state in time
    advance_in_time(ite, delta_t, state)
    
    if(round(simulation_frequency*time) % round(simulation_frequency/HEARTBEAT_HZ) == 0):

        trajecto.compute(1000*time, count)
        target_altitude=trajecto.altitude[count]

        print('%d ms' %int(1000*time))
        delta_t_MP=1000*(time-previous_time)
        ##print('delta_t_MP', delta_t_MP, 'ms')
        previous_time=time

        ##sensor inputs
        z_sensor=baro.compute_response(state.z, ite, delta_t, count)
        
        z_sonar=sonar.compute_response(state.z, ite, delta_t, count)
        if sonar.is_valid:
            z_sensor=z_sonar

        accz_sensor=accelero.compute_response(state.z_dot_dot, ite, delta_t, count)
        
        if (count==0):
    
            ##initialize filtered state
            filtered_z=state.z[ite]
            filtered_vz=state.z_dot[ite]
            filtered_accz=state.z_dot_dot[ite]
            filtered_z_minus_one=filtered_z

        ##filtering
        if count > 0: ##hoverwaitvzstable:
            filtered_z = exp_filter_instant(z_sensor, filtered_z, INV_DELTA_FILTER_SONAR, HEARTBEAT_HZ)

            vz_sensor=(filtered_z-filtered_z_minus_one)*HEARTBEAT_HZ
            filtered_z_minus_one=filtered_z
                
            filtered_vz = exp_filter_instant(vz_sensor, filtered_vz, INV_DELTA_FILTER_VZ, HEARTBEAT_HZ)
            filtered_accz = exp_filter_instant(accz_sensor, filtered_accz, INV_DELTA_FILTER_ACCEL, HEARTBEAT_HZ)

        ##PIDs
        if TARGET_TYPE==0 or sonar.is_valid:
            pid_z.compute(filtered_z, target_altitude, error_integral_z)
            pid_z.output[-1] = limit(pid_z.output[-1], LIMIT_VZ)
            target_vz=pid_z.output[-1]
            error_z = pid_z.error[-1]
            error_integral_z = pid_z.error_integral[-1]
        else:
            target_vz=trajecto.velocity[count]
            
        pid_vz.compute(filtered_vz, target_vz, error_integral_vz)
        pid_vz.output[-1] = limit(pid_vz.output[-1], LIMIT_ACCZ)
        target_accz=pid_vz.output[-1]
        error_vz = pid_vz.error[-1]
        error_integral_vz = pid_vz.error_integral[-1]
        
        pid_accz.compute(filtered_accz, target_accz, error_integral_accz)
        throttle_tmp = pid_accz.output[-1]
        error_accz = pid_accz.error[-1]
        error_integral_accz = pid_accz.error_integral[-1]

        throttle_tmp=throttle_tmp*2*AIRCRAFT_MASS/100/MAX_THRUST
        throttle_tmp=throttle_tmp+THROTTLE_OFFSET*2*SERVORANGE

        if throttle_tmp > THROTTLE_MAX*2*SERVORANGE:
            throttle_tmp=THROTTLE_MAX*2*SERVORANGE
        if throttle_tmp < THROTTLE_MIN*2*SERVORANGE:
            throttle_tmp=THROTTLE_MIN*2*SERVORANGE

        ##update manoeuvre
        manoeuvre_index=trajecto.flight_phases[trajecto.current_phase].manoeuvre_index
        if manoeuvre_index > -1:
            current_manoeuvre=manoeuvres[manoeuvre_index]
            for i in range(len(current_manoeuvre)):
                if (current_manoeuvre[i].initialized == 0):
                    current_manoeuvre[i].init_time=time
                    current_manoeuvre[i].initialized=1
                current_manoeuvre[i].update_manoeuvre(time, count)
            
        count=count+1
                

    throttle[ite] = throttle_tmp
    
    

    


nb_subplot_v = 2
nb_subplot_h = 2
time=np.linspace(0, simulation_time, nb_ite)
time_MP=np.linspace(0, simulation_time, count)
throttle_interp=np.interp(time_MP, time, throttle)
accz=np.interp(time_MP, time, state.z_dot_dot)
vz=np.interp(time_MP, time, state.z_dot)
z=np.interp(time_MP, time, state.z)
omega=np.interp(time_MP, time, dyn_rot.omega)
omega_dot=np.interp(time_MP, time, dyn_rot.omega_dot)
thrust=np.interp(time_MP, time, propeller.thrust)
#z_target=np.interp(time_MP, time, trajecto.altitude)
#vz_target=np.interp(time_MP, time, trajecto.velocity)

fontsize=12
fig = plt.figure(figsize=(16.0, 9.0))
fig.subplots_adjust(hspace=0.5)

ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 1)
ax.set_title('accz', fontweight='bold', fontsize=fontsize)

ymin=-1000.
ymax=2000.
xlabel=''
ylabel=''

ax.plot(time_MP, accz, 'k-', marker=None, label='accz')
ax.plot(time_MP, throttle_interp, 'g-', marker=None, label='throttle')
ax.plot(time_MP, pid_accz.error, 'm--', marker=None, label='error_accz')
ax.plot(time_MP, pid_vz.output, 'b-', marker=None, label='target_accz')
ax.plot(time_MP, pid_accz.input, 'r-', marker=None, label='accz_filt')
ax.plot(time_MP, accelero.output, '0.8', label='accz_measured')
ax.plot(time_MP, 100*thrust, 'y-', marker=None, label='100*thrust')

finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ymin=-400.
ymax=400.

ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 2)
ax.set_title('vz', fontweight='bold', fontsize=fontsize)
ax.plot(time_MP, vz, 'k-', marker=None, label='vz')
ax.plot(time_MP, pid_vz.error, 'm-', marker=None, label='error_vz')
ax.plot(time_MP, pid_vz.input, 'r-', marker=None, label='vz_filt')
ax.plot(time_MP, trajecto.velocity, 'b-', marker=None, label='target_vz')
aircraft_section=1.*0.02+0.2*0.02+0.5*np.pi*0.08**2/4
drag=0.5*rho*(0.01*vz)**2*aircraft_section
#print('drag', drag)

finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)

ymin=-100.
ymax=2200.

ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 3)
ax.set_title('z', fontweight='bold', fontsize=fontsize)
ax.plot(time_MP, z, 'k-', marker=None, label='z')
ax.plot(time_MP, trajecto.altitude, 'g-', marker=None, label='z_target')
#ax.plot(time_MP, pid_z.error, 'm-', marker=None, label='error_z')
#ax.plot(time_MP, pid_z.input, 'r-', marker=None, label='z_filt')
ax.plot(time_MP, sonar.output, '0.8', label='z_sonar')
ax.plot(time_MP, baro.output, 'b-', label='z_baro')

finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='.', output_file=export_file, \
              show_legend=True, legend_type='outer_right', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None)


ymin=0.
ymax=5000.

ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 4)
ax.set_title('servo output', fontweight='bold', fontsize=fontsize)
for i in range(len(manoeuvres)):
    for j in range(len(manoeuvres[i])):
        ax.plot(time_MP, manoeuvres[i][j].output, label='manoeuvres %d %d' %(i,j))



finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='.', output_file=export_file, \
              show_legend=True, legend_type='outer_right', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)


##ax = fig.add_subplot(nb_subplot_v, nb_subplot_h, 4)
##ax.plot(time_MP, omega, 'b-', marker=None, label='omega')
##ax.plot(time_MP, omega_dot, 'k-', marker=None, label='omega_dot')
##
##finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
##                  show_legend=True, legend_type='outer_left', logscale_x=False, logscale_y=False, show=True, tick_fontsize=None)
a


time_movie=np.linspace(0, simulation_time, 24*simulation_time)
z=np.interp(time_movie, time, state.z)
accz=np.interp(time_movie, time, state.z_dot_dot)

fig, ax = plt.subplots()
ax.set_ylim([-1000., 1000.])
x=[0.]
line_z, = ax.plot([], [], 'bo-')
line_accz, = ax.plot([], [], 'ro-')
time=np.linspace(0, simulation_time, nb_ite)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    """initialize animation"""
    line_z.set_data([], [])
    line_accz.set_data([], [])
    time_text.set_text('')
    return line_z, line_accz, time_text

def animate(i):
    """perform animation step"""
    line_z.set_data(x, [z[i]])
    line_accz.set_data(x, [accz[i]])
    time_text.set_text('time = %.3f' % time_movie[i])
    return line_z, line_accz, time_text

ani = animation.FuncAnimation(fig, animate, np.arange(1, len(time_movie)), init_func=init,
                              interval=1000./24, blit=True)
plt.show()

##FFwriter = animation.FFMpegWriter()
##anim.save('basic_animation.mp4', fps=30, writer = FFwriter)
##plt.show()
