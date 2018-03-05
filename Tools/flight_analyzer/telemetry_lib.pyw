import numpy as np
import toolbox as tool
import os


################################################################
def read_first_line(bidon, separator, line_len, var_name, show_header=0):

    index_var = -1
    for i in range(1, len(bidon)):
        line = bidon[i].split(separator)
        print('number of variables on line is %d' % len(line))
        if len(line) == line_len:
            # print line
            # print len(line)
            if show_header:
                for j in range(line_len):
                    pass

            for i in range(len(line)):
                # print(line[i])
                if line[i].find(var_name) == 0:
                    try:
                        tmp = line[i].split(',')[0]
                        index_var = i
                        break
                    except:
                        pass

        if index_var >= 0:
            break

    if index_var == -1:
        print('could not find %s in log file' % (var_name))
        print('please check line length and variable name')

    return index_var

################################################################


def read_txt(bidon, separator, index_col, line_len, var_index):

    unread_lines = []

    var = []
    for i in range(len(bidon)):
        line = bidon[i].split(separator)
        if len(line) == line_len:
            var.append(line[index_col].rstrip())
        else:
            if i > 0:
                var.append(var[i - 1])
            else:
                var.append('NaN')
            unread_lines.append(i)

    return var, unread_lines

################################################################


def norm(x, y, z):

    return np.sqrt(x**2+y**2+z**2)

################################################################


def collect_column(index_start, index_end, step, index_col, data):

    col = []
    for i in range(index_start, index_end, step):
        col.append(float(data[i][index_col]))

    return np.array(col)


################################################################
def read_header(input_dir, filename, nb_header_lines):

    ifile = open(os.path.join(input_dir, filename), "rb")
    bidon = ifile.readlines()
    header = ''
    for i in range(nb_header_lines):
        # print(bidon[i])
        header += bidon[i][:-1]

    print(header)

    return header


################################################################

def extract_var(bidon, line_len, var_name, var_index=1, nb_header_lines = 10):

    print('extracting variable', var_name)

    nb_data = len(bidon) - nb_header_lines

    index_var = read_first_line(bidon, ":", line_len, var_name, show_header=0)
    if index_var == -1:
        var = np.empty((nb_data))
        var[:] = np.nan
        print('variable not found, NaN are returned', var)
        return var

    print('index_var', index_var + (var_index - 1))

    # return a list of string corresponding to index_col

    var_str, unread_lines = read_txt(bidon, ":", index_var, line_len, var_index)

    var_array = var_str[nb_header_lines].split(',')

    if len(var_array) == 1:
        var = np.zeros((nb_data))
    else:
        var = np.zeros((nb_data, len(var_array)))
    print(var.shape)

    for j in range(nb_data):
        var_array = var_str[j + nb_header_lines].split(',')

        if len(var_array) == 1:

            try:
                var[j] = float(var_array[0].replace(var_name, ''))
            except:
                var[j] = np.nan

        else:

            try:
                var[j, 0] = float(var_array[0].replace(var_name, ''))
            except:
                var[j, 0] = np.nan
            for k in range(len(var_array) - 1):
                var[j, k + 1] = float(var_array[k + 1])

    print('number of unread lines is %d' % len(unread_lines))
    print('total number of samples is %d' % len(var))
    print('percentage of unread lines is %d' %
          int(100 * len(unread_lines) / len(var)))

    return var


###################################

def find_number_of_lines(bidon):

    ifile = open(os.path.join(input_dir, filename), "rb")
    bidon = ifile.readlines()

    return(len(bidon))


###################################

def limit_value(value, limit):

    value[np.where(value > limit)] = limit
    value[np.where(value < -limit)] = -limit

###################################


def find_final_time(bidon, line_len, test_var, tf, HEARTBEAT_UDB):

    test = extract_var(bidon, line_len, test_var)[:]
    tf_tmp = len(test) / HEARTBEAT_UDB

    print('nb samples', len(test))

    if tf is None:
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
                line += '%s%d' % (var_list[j], var_to_write[j][i, 0])
                for k in range(len(var_to_write[j][i]) - 1):
                    line += ',%d' % var_to_write[j][i, k + 1]
            except:
                line += '%s%d' % (var_list[j], var_to_write[j][i])

            line += ':'
        line += '\n'
        str_tow += line
    # print(str_tow)
    f = open(os.path.join(input_dir, filename), 'w')
    f.write(str_tow)
    f.close()

###################################
