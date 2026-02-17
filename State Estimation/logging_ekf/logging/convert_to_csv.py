# -*- coding: utf-8 -*-
"""
Convert USD log to CSV file
"""
import cfusdlog
import argparse
import numpy as np

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("file_usd")
    parser.add_argument("file_csv")
    args = parser.parse_args()

    # decode binary log data
    data_usd = cfusdlog.decode(args.file_usd)

    # find start time
    start_time = None
    for k, (event_name, data) in enumerate(data_usd.items()):
        if start_time is None:
            start_time = data['timestamp'][0]
        else:
            start_time = min(start_time, data['timestamp'][0])

    # find all columns and size
    vars = set()
    num_entries = 0
    for event_name, data in data_usd.items():
        num_rows = len(data["timestamp"])
        num_entries += num_rows
        for key in data.keys():
            vars.add(key)
    vars.remove("timestamp")
    columns = ["timestamp"] + sorted(list(vars))
    print(columns)

    # parse data
    result = np.zeros((num_entries, len(columns))) * np.nan
    idx = 0
    for event_name, data in data_usd.items():
        num_rows = len(data["timestamp"])
        for c, column in enumerate(columns):
            if column in data:
                # print(c, column, data[column])
                result[idx:idx+num_rows, c] = data[column]
        idx += num_rows

    # sort by timestamp
    result = result[result[:, 0].argsort()]

    # conversions
    result[:,0] = (result[:,0] - start_time) / 1000

    # motion's don't match the internal firmware variables
    # fix it here
    motion_deltaX = result[:,7].copy()
    motion_deltaY = result[:,8].copy()
    result[:,7] = -motion_deltaY
    result[:,8] = -motion_deltaX

    np.savetxt(args.file_csv, result, delimiter=',', header=",".join(columns))
