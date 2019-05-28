#!/usr/local/bin/python3

import os
import re
import sys


def create_log_dict(log_path):
    """
    read log file and create dict file for acc, vel, pos
    :param log_path: path to log file to read from
    :return: dictionary with acc, vel, pos keys
    """
    dic = {}
    accs = []
    vs = []
    ds = []

    log_file = open(log_path, 'r')
    measurement_regex = r'\d*:\d*:\d*.\d* INFO\[NAV\]: [\d]+: Update: a=(-?\d*\.\d*)*, z=[-?\d*\.\d*]*, v=(-?\d*\.\d*)*, d=(-?\d*\.\d*)*'

    last_a = 0.0
    last_v = 0.0
    last_d = 0.0

    for line in log_file.readlines():
        match = re.match(measurement_regex, line)
        if match:
            a, v, d = (float(match.group(1)), float(match.group(2)), float(match.group(3)))
            if not (a == last_a and v == last_v and d == last_d):
                accs.append(a)
                vs.append(v)
                ds.append(d)
                last_a = a
                last_v = v
                last_d = d

    dic['a'] = accs
    dic['v'] = vs
    dic['d'] = ds
    return dic


def write_dict_to_csv(d, csv_path):
    """
    write dict to csv file format
    :param d: dictionary to write to file (each entry is list and all lists must be equally long!)
    :param csv_path: write csv file to this path
    """
    if os.path.isfile(csv_path):
        print(csv_path + ' already exists!')
        print('Do you want to overwrite this file? [Y/n]')
        x = input()
        if x == 'Y' or x == 'y':
            print('The file will be overwritten. Processing ...')
        elif x == 'N' or x == 'n':
            print('The file will not be changed. Stopping ...')
            sys.exit(1)
        else:
            print('Unknown input ' + x + ': Stopping ...')
            sys.exit(1)
    with open(csv_path, 'w') as csv_file:
        # write header line
        h = ''
        for k in d.keys():
            h += str(k) + ','
        csv_file.write(h[:-1] + '\n')
        
        l = len(d[list(d.keys())[0]])
        for i in range(l):
            line = ''
            for k in d.keys():
                line += str(d[k][i]) + ','
            csv_file.write(line[:-1] + '\n')


def main(argv):
    if len(argv) != 2:
        print("Usage: ./log_to_csv.py <path/to/log/file>")
        sys.exit(1)
    log_path = argv[1]
    if not os.path.isfile(log_path):
        print(log_path + " is not a valid file path!")
        sys.exit(1)

    d = create_log_dict(log_path)
    csv_path = log_path[:-3] + "csv"
    write_dict_to_csv(d, csv_path)

if __name__ == "__main__":
    main(sys.argv)
