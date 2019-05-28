#!/usr/local/bin/python3

import math
import os
import re
import sys


def read_file(log_path, regex):
    ts = []
    accs = []
    vs = []
    ds = []

    first_t = None
    last_t = None
    last_a = 0.0
    last_v = 0.0
    last_d = 0.0

    with open(log_path, 'r') as log_file:
        for line in log_file.readlines():
            match = re.match(regex, line)
            if match:
                h, m, s, ms = (int(match.group(1)), int(match.group(2)), int(match.group(3)), int(match.group(4)))
                a, v, d = (float(match.group(5)), float(match.group(6)), float(match.group(7)))
                t = h * 3600 + m * 60 + s + ms/1000
                while last_t is not None and (t < last_t or abs(t - last_t) >= 1):
                    # unrealistic time jump -> correct this inconsistency
                    t = correct(t, last_t)
                last_t = t
                if not (a == last_a and v == last_v and d == last_d):
                    if first_t is None:
                        first_t = t

                    ts.append(t - first_t)
                    accs.append(a)
                    vs.append(v)
                    ds.append(d)
                    last_a = a
                    last_v = v
                    last_d = d

    return ts, accs, vs, ds


def correct(t, last_t):
    """
    correct time value (inconsistency through log time update delays)
    :param t: current timestamp
    :param last_t: last timestamp
    :return: corrected current timestamp
    """
    if t < last_t:
        # some update is delayed
        if (last_t - t) < 1.1:
            # second update delayed
            return t + 1
        elif (last_t - t) < 60.1:
            # minute upate delayed
            return t + 60
        elif (last_t - t) < 3600.1:
            # hour update delayed
            return t + 3600
        elif t < 1.0:
            # 24h jump --> add required hours on top of t
            return t + round(last_t / 3600) * 3600
    else:
        # some update is too early
        if (t - last_t) < 1.1:
            # second update too early
            return t - 1
        elif (t - last_t) < 60.1:
            # minute upate too early
            return t - 60
        elif (t - last_t) < 3600.1:
            # hour update too early
            return t - 3600
        else:
            print("WTF", t, last_t)


def create_log_dict(log_path):
    """
    read log file and create dict file for acc, vel, pos
    :param log_path: path to log file to read from
    :return: dictionary with counts, acc, vel, pos keys
    """
    dic = {}

    regex = r'(\d*):(\d*):(\d*).(\d*) INFO\[NAV\]: [\d]+: Update: a=(-?\d*\.\d*)*, z=[-?\d*\.\d*]*, v=(-?\d*\.\d*)*, d=(-?\d*\.\d*)*'

    ts, accs, vs, ds = read_file(log_path, regex)
    
    if len(ts) == 0:
        # no matches found --> attempt other output format regex
        regex = r'(\d*):(\d*):(\d*).(\d*) INFO\[NAV\]: [\d]+: Update: a=(-?\d*\.\d*)*, v=(-?\d*\.\d*)*, d=(-?\d*\.\d*)*'
        ts, accs, vs, ds = read_file(log_path, regex)
        if len(ts) == 0:
            # no matches found --> attempt last output format regex
            regex = r'(\d*):(\d*):(\d*).(\d*) INFO\[NAV\]: Update: a=(-?\d*\.\d*)*, v=(-?\d*\.\d*)*, d=(-?\d*\.\d*)*'
            ts, accs, vs, ds = read_file(log_path, regex)

    dic['t'] = ts
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
