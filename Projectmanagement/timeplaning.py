import os
import pandas as pd
import numpy as np
import matplotlib
import matplotlib.pyplot as plt
import datetime as dt

title = 'Projekt KERS: Zeitplan'
mech = 'Konstruktion'
elec = 'Elektrotechnik'
sw = 'Software'
pj = 'Projektmanagement'
team_colors = {mech: '#4e95d9', elec: '#f2aa84', sw: '#b4e5a2', pj: '#ffe141'}

df = pd.DataFrame({'task': ['Konstruktion', 'Bauteile beschaffen', 'Fertigung', 'Endmontage', 
                            'Entwicklung', 'Komponenten beschaffen', 'Verkabelung', 'Stromlaufplan', 
                            'Kontroller definieren', 'Implementierung', 'Testen', 'Inbetriebnahme',
                            'Dokumentation'],
                  'team': [mech, mech, mech, mech, 
                           elec, elec, elec, elec, 
                           sw, sw, sw, sw,
                           pj],
                  'start': pd.to_datetime(['2024-09-07', '2024-09-07', '2024-10-05', '2024-10-26', 
                                           '2024-09-07', '2024-09-28', '2024-11-09', '2024-11-16',
                                           '2024-09-07', '2024-10-05', '2024-11-09', '2024-11-30',
                                           '2024-09-07']),
                  'end': pd.to_datetime(['2024-12-07', '2024-11-09', '2024-11-16', '2025-02-01',
                                         '2024-11-02', '2024-11-09', '2025-03-02', '2025-03-02',
                                         '2024-10-05', '2024-11-30', '2024-12-21', '2025-03-02',
                                         '2025-03-02'])})

df['days_to_start'] = (df['start'] - df['start'].min()).dt.days
df['days_to_end'] = (df['end'] - df['start'].min()).dt.days
df['task_duration'] = df['days_to_end'] - df['days_to_start'] + 1  # to include also the end date

patches = []
for team in team_colors:
    patches.append(matplotlib.patches.Patch(color=team_colors[team]))

fig, ax = plt.subplots()
for index, row in df.iterrows():
    # Adding a lower bar - for the overall task duration
    plt.barh(y=row['task'], width=row['task_duration'], left=row['days_to_start'] + 1, color=team_colors[row['team']], alpha=1)

plt.title(title, fontsize=15)
plt.gca().invert_yaxis()
xticks = np.arange(5, df['days_to_end'].max() + 2, 7)
xticklabels = pd.date_range(start=df['start'].min() + dt.timedelta(days=4), end=df['end'].max()).strftime("%d.%m")
ax.set_xticks(xticks)
ax.set_xticklabels(xticklabels[::7])
ax.xaxis.grid(True, alpha=0.5)
ax.legend(handles=patches, labels=team_colors.keys(), fontsize=11)

figure = plt.gcf()
figure.set_size_inches(18,10)
#plt.show()

# Change to current directory and save file
name_of_current_directory = os.path.dirname(__file__)
os.chdir(name_of_current_directory)
figure.savefig('timeplaning.png')
#plt.savefig('timeplaning.png')

