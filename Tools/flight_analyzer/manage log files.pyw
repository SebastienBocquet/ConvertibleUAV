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
from telemetry_lib import *



###################################
#MAIN
###################################

input_dir='E:\\projet autoentrepreneur\\prise de vue aerienne\\tests\\Gragnagues 05-2016'
line_len=65
t0=40
HEARTBEAT_UDB=10
tf=None
nb_header_lines=9

file_number=2243

#0,0,0:accz-47:tgz0:tgvz0:tgaccz0:inz0:invz0:inaccz0:ezi0:evzi0:segi0:sond0:sonhtg0:tmp324:prs99399:alt0:add10:add20:add30:add40:add50:add60:add70:add80:add90:


tf=find_final_time('LOG%05d.TXT' %file_number, input_dir, line_len, 'cpu', tf, HEARTBEAT_UDB)

header = read_header(input_dir, 'LOG%05d.TXT' %file_number, nb_header_lines)

s = np.zeros(((tf-t0)*HEARTBEAT_UDB))
c = np.zeros(((tf-t0)*HEARTBEAT_UDB))

var_name = []
var_to_write = []

var_list = ['T', 'S', 'N', 'E', 'A', 'W', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i']
for i in range(len(var_list)):
    var_name.append(var_list[i])
    var_to_write.append(extract_var('LOG%05d.TXT' %file_number, input_dir, line_len, var_list[i])[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB])
    #print(len(var_to_write[i]))
    #print(t0)
    #print(tf)

var_name.append('c')
var_to_write.append(c)

var_name.append('s')
var_to_write.append(s)

var_list = ['cpu', 'wvx', 'wvy', 'wvz', 'p1i', 'p2i', 'p3i', \
            'p4i', 'p5i', 'p6i', 'p7i', 'p1o', 'p2o', 'p3o', 'p4o', 'p5o', 'p6o', 'imx', 'imy', 'imz', 'tx', 'ty', 'tz', 'G', 'accz', 'tgz', 'tgvz', 'tgaccz', 'inz', 'invz', 'inaccz', 'ezi', 'evzi', 'segi', 'sond', \
            'sonhtg', 'tmp', 'prs', 'alt']
for i in range(len(var_list)):
    var_name.append(var_list[i])
    var_to_write.append(extract_var('LOG%05d.TXT' %file_number, input_dir, line_len, var_list[i])[t0*HEARTBEAT_UDB:tf*HEARTBEAT_UDB])
    
write_log(input_dir, 'LOG%05d_modif.TXT' %file_number, header, var_name, var_to_write)
