import pandas as pd
import matplotlib.pyplot as plt
from scipy.interpolate import make_interp_spline, BSpline
import numpy as np

df = pd.read_csv('test.txt', sep=" ", dtype={
                 'ws_size': 'int64'}, header=None, error_bad_lines=False)
df.columns = ["sorter", "ws_size"]

df['ws_size'] = df['ws_size'].astype('int64')
print(df.head(3))

sorters = [0, 0, 0, 0]

sorters[0] = df[df['sorter'] == 'bubble'].reset_index(drop=True).head(5000)
sorters[1] = df[df['sorter'] == 'quick'].reset_index(drop=True).head(5000)
sorters[2] = df[df['sorter'] == 'merge'].reset_index(drop=True).head(5000)
sorters[3] = df[df['sorter'] == 'index'].reset_index(drop=True).head(5000)


fig, axs = plt.subplots(2, 2)
axs[0, 0].set_title('Bubble Sort')
axs[0, 1].set_title('Quick Sort')
axs[1, 0].set_title('Merge Sort')
axs[1, 1].set_title('Index Sort')

indices = [(0, 0), (0, 1), (1, 0), (1, 1)]
colors = ['tab:orange', 'tab:green', 'tab:red', 'tab:blue']

for i in range(0, 4):
    sorter = sorters[i]
    sorter['k'] = pd.Series(list(range(len(df))))
    x = sorter['k'].to_numpy()
    y = sorter['ws_size'].to_numpy()

    xnew = np.linspace(x.min(), x.max(), 50)
    spl = make_interp_spline(x, y, k=3)
    smooth = spl(xnew)

    axs[indices[i]].set_xticklabels([])
    axs[indices[i]].set_yticklabels([])
    axs[indices[i]].set_xlabel('k')
    axs[indices[i]].set_ylabel('w(k,t)')
    axs[indices[i]].plot(xnew, smooth, colors[i])


plt.show()
