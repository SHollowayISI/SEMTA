import numpy as np
import csv

a = np.eye(7)

with open('testCSV.csv', mode='w') as csv_out:
    csvWriter = csv.writer(csv_out, delimiter=',', quotechar='"')

    csvWriter.writerows(a)