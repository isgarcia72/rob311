import pandas

data = pandas.read_csv('data/drive_test_1.txt', sep=' ', header=None, names=["iteration", "time", "psi_1", "psi_2", "psi_3", "dphi"])
print(max(data.dphi))