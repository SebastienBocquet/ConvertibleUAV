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



################################################################
##def finalize_plot(fig, ax, xmin, xmax, ymin, ymax, xlabel, ylabel, fontsize, export_dir='', output_file='', \
##                  show_legend=True, legend_type='basic', logscale_x=False, logscale_y=False, show=False, tick_fontsize=None):
##
##    if tick_fontsize == None:
##        tick_fontsize=fontsize
##
##    if xmin==None or xmax==None:
##        if not (ymin==None or ymax==None):
##            ax.set_ylim([ymin, ymax])
##    elif ymin==None or ymax==None:
##        if not (xmin==None or xmax==None):
##            ax.set_xlim([xmin, xmax])
##    else:
##        ax.axis([xmin, xmax, ymin, ymax])
##
##    if show_legend:
##        handles, labels = ax.get_legend_handles_labels()
##        if legend_type == 'outer_top':
##            ax.legend(handles, labels, prop={'size':fontsize}, bbox_to_anchor=(0., 1.02, 1., .102), loc=3, mode="expand", frameon=False)
##        elif legend_type == 'outer_right':   
##            ax.legend(handles, labels, prop={'size':fontsize}, bbox_to_anchor=(1.02, 1., 1., .102), loc=2, mode="expand", frameon=False)
##        elif legend_type == 'outer_left':   
##            ax.legend(handles, labels, prop={'size':fontsize}, bbox_to_anchor=(0., 1.02, 1., .102), loc='upper left', mode="expand", frameon=False)
##        elif legend_type == 'basic':
##            ax.legend(handles, labels, prop={'size':fontsize}, loc=0)
##        else:
##            print ('no legend shown')
##       
## 
##    if logscale_x==True:
##        ax.set_xscale('log')
##       
##    if logscale_y==True:
##        ax.set_yscale('log')
##       
##    ax.set_xlabel(xlabel, fontsize=fontsize)
##    ax.set_ylabel(ylabel, fontsize=fontsize)
##
##    gridlines = ax.get_xgridlines()
##    gridlines.extend( ax.get_ygridlines() )
##    for line in gridlines:
##        line.set_linestyle('--')
##
##    ticklabels = ax.get_xticklabels()
##    ticklabels.extend( ax.get_yticklabels() )
##    for label in ticklabels:
##        label.set_color('k')
##        label.set_fontsize(tick_fontsize)
##
##    if not (export_dir == '' or output_file == ''):
##        fig.savefig(os.path.join(export_dir, output_file), dpi=400)
##
##    if show:
##        plt.show()
        
################################################################
def read_first_line(textfilename, separator, line_len, var_name, show_header=0):

    ifile  = open(textfilename, "rb")
    bidon = ifile.readlines()

    index_var=-1
    for i in range(len(bidon)):
        line=bidon[i].split(separator)
        print ('number of variables on line is %d' %len(line))
        if len(line) == line_len:
            #print line
            #print len(line)
            if show_header:
                for j in range(line_len):
                    pass
                    #print (j, line[j])

            for i in range(len(line)):
                #print(line[i])
                if line[i].find(var_name) == 0:
                    try:
                        tmp=line[i].split(',')[0]
                        a=int(tmp[len(var_name):])
                        index_var=i
                    except:
                        pass

        if index_var >= 0:
            break
                
    if index_var==-1:
        print('could not find %s in log file %s' %(var_name, textfilename))
        print('please check line length and variable name')
        sys.exit(1)

    return index_var
                    
################################################################
def read_txt(textfilename, separator, index_col, line_len, var_index):

    ifile  = open(textfilename, "rb")
    bidon = ifile.readlines()
    unread_lines=[]

    var=[]
    for i in range(len(bidon)):
        line=bidon[i].split(separator)
        if len(line) == line_len:
            var.append(line[index_col].rstrip())
        else:
            if i > 0:
                var.append(var[i-1])
            else:
                var.append('NaN')
            unread_lines.append(i)

    return var, unread_lines

################################################################
def norm(x,y,z):

    return np.sqrt(x**2+y**2+z**2)

################################################################
def collect_column(index_start, index_end, step, index_col, data):

    col=[]
    for i in range(index_start,index_end,step):
        col.append(float(data[i][index_col]))

    return np.array(col)


################################################################
def read_header(input_dir, filename, nb_header_lines):
    
    ifile  = open(os.path.join(input_dir, filename), "rb")
    bidon = ifile.readlines()
    header = ''
    for i in range(nb_header_lines):
        #print(bidon[i])
        header+=bidon[i][:-1]

    print(header)

    return header


################################################################

def extract_var(filename, input_dir, line_len, var_name, var_index=1):

    nb_header_lines = 10
    
    print('extracting variable', var_name)
    #print input_dir
    #print filename
    #print (os.path.join(input_dir, filename))
    ##find index corresponding to var_name
    index_var=read_first_line(os.path.join(input_dir, filename), ':', line_len, var_name, show_header=0)
    print('index_var', index_var+(var_index-1))
    
    #return a list of string corresponding to index_col

    var_str, unread_lines=read_txt(os.path.join(input_dir, filename), ':', index_var, line_len, var_index)

    nb_data=find_number_of_lines(input_dir, filename)-nb_header_lines

    var_array=var_str[nb_header_lines].split(',')

    if len(var_array)==1:
        var = np.zeros((nb_data))
    else:
        var = np.zeros((nb_data, len(var_array)))
    print var.shape

    for j in range(nb_data):
        #print j
        #print(var_str[j])
        var_array=var_str[j+nb_header_lines].split(',')
        #print(var_array)

        if len(var_array) == 1:

            try:
                var[j] = float(var_array[0].replace(var_name, ''))
            except:
                var[j] = np.nan
                
        else:
  
            try:
                var[j,0] = float(var_array[0].replace(var_name, ''))
            except:
                var[j,0] = np.nan
            for k in range(len(var_array)-1):
                var[j,k+1] = float(var_array[k+1])

    print('number of unread lines is %d' %len(unread_lines))
    print('total number of samples is %d' %len(var))
    print('percentage of unread lines is %d' %int(100*len(unread_lines)/len(var)))
    #print('unread_lines index', unread_lines)

##    for i in range(nb_data):
##        print var[i]
    
    return var


###################################

def find_number_of_lines(input_dir, filename):

    ifile  = open(os.path.join(input_dir, filename), "rb")
    bidon = ifile.readlines()
    
    return(len(bidon))
    

###################################

def limit_value(value, limit):

    value[np.where(value>limit)]=limit
    value[np.where(value<-limit)]=-limit

###################################
def find_final_time(filename, input_dir, line_len, test_var, tf, HEARTBEAT_UDB):

    test=extract_var(filename, input_dir, line_len, test_var)[:]
    tf_tmp=len(test)/HEARTBEAT_UDB

    print('nb samples', len(test))

    if tf==None:
        return tf_tmp
    else:
        return tf

###################################

def write_log(input_dir, filename, header, var_list, var_to_write):

    str_tow = header
    for i in range(len(var_to_write[0])):
        line = 'F2:'
        for j in range(len(var_list)):

            try:
                var_dim2 = var_to_write[j].shape[1]
                
                line += '%s%d' %(var_list[j], var_to_write[j][i,0])
                for k in range(len(var_to_write[j][i])-1):
                    line+=',%d' %var_to_write[j][i,k+1]
            except:
                #print var_to_write[j][i]
                #print var_list[j]
                line += '%s%d' %(var_list[j], var_to_write[j][i])
                
            line += ':'
        line += '\n'
        str_tow+=line
    #print(str_tow)
    f=open(os.path.join(input_dir, filename), 'w')
    f.write(str_tow)
    f.close()
    
    
###################################
