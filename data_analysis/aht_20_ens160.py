import pandas as pd
import matplotlib.dates as mdates

import os
import numpy as np

data = "aht20_ens160.txt"
basename, _ = os.path.splitext(data)

def fileGenerator(path):
    with open(path, 'r') as f:
        for line in f:
            yield line

dataGen = fileGenerator(data)
df_dict = {}

for line in dataGen:
    timestamp, data = line.split(" -> ")
    timestamp = pd.to_datetime(timestamp)
    names = []
    values = []
    for column in data.split(","):
        name, value = column.split(":")
        names.append(name)
        values.append(float(value))
    df_dict[timestamp] = values

df = pd.DataFrame.from_dict(df_dict, orient='index')
df.columns = names
df.index.name = 'Time'
axes = df.plot(subplots=True, sharex=True, title=basename, colormap='cividis', grid=True, legend=False)
fig = axes[0].get_figure()

for ax, col in zip(axes, names):
    # Setting title
    ax.title.set_text(col)
    # Setting a better dateformatter
    ax.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M"))
    # Adding extra tick in middle
    yticks_current = ax.get_yticks()[1:-1]
    if len(yticks_current) == 2:
        yticks_new = np.linspace(*yticks_current, 3)
        ax.set_yticks(yticks_new)

fig.tight_layout()
fig.savefig(f'{basename}.png', dpi=300)