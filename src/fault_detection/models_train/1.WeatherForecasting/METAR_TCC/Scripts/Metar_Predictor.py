# %%
"""
![image.png](attachment:image.png)


# Predictive System Using Metar Data

Main database: https://mesonet.agron.iastate.edu/request/download.phtml?network=BR__ASOS

The IEM maintains an ever growing archive of automated airport weather observations from around the world! These observations are typically called 'ASOS' or sometimes 'AWOS' sensors. A more generic term may be METAR data, which is a term that describes the format the data is transmitted as. This archive simply provides the as-is collection of historical observations, very little quality control is done. More details on this dataset are here: https://mesonet.agron.iastate.edu/info/datasets/metar.html

ASOS User's Guide: https://www.weather.gov/media/asos/aum-toc.pdf

Tools/Libaries: Here (https://github.com/akrherz/iem/blob/main/scripts/asos/iem_scraper_example.py) is a python script example that automates the download of data from this interface. A community user has contributed R language version of the python script. There is also a riem R package allowing for easy access to this archive.
"""

# %%
"""
# Main Libraries
"""

# %%
import warnings
warnings.simplefilter(action='ignore', category=FutureWarning)

# Numerical libraries
import pandas as pd
import numpy as np

# Visualization
import plotly.express as px
import plotly.graph_objects as go
import matplotlib.pyplot as plt
import seaborn as sns
sns.set(style="ticks", color_codes=True)

# Statistics and Machine Learning
import statistics
from sklearn.model_selection import train_test_split
from sklearn import metrics

# %%
"""
# Main Database: 2017-01-01 to 2021-12-31
"""

# %%
"""
## 1- Loading Data


**Metadata:**
- station: three or four character site identifier
- valid: timestamp of the observation
- tmpf: Air Temperature in Fahrenheit, typically @ 2 meters
- dwpf: Dew Point Temperature in Fahrenheit, typically @ 2 meters
- relh: Relative Humidity in %
- drct: Wind Direction in degrees from true north
- sknt: Wind Speed in knots
- p01i: One hour precipitation for the period from the observation time to the time of the previous hourly precipitation reset. This varies slightly by site. Values are in inches. This value may or may not contain frozen precipitation melted by some device on the sensor or estimated by some other means. Unfortunately, we do not know of an authoritative database denoting which station has which sensor.
- alti: Pressure altimeter in inches
- mslp: Sea Level Pressure in millibar
- vsby: Visibility in miles
- gust: Wind Gust in knots
- skyc1: Sky Level 1 Coverage
- skyc2: Sky Level 2 Coverage
- skyc3: Sky Level 3 Coverage
- skyc4: Sky Level 4 Coverage
- skyl1: Sky Level 1 Altitude in feet
- skyl2: Sky Level 2 Altitude in feet
- skyl3: Sky Level 3 Altitude in feet
- skyl4: Sky Level 4 Altitude in feet
- wxcodes: Present Weather Codes (space seperated)
- feel: Apparent Temperature (Wind Chill or Heat Index) in Fahrenheit
- ice_accretion_1hr: Ice Accretion over 1 Hour (inches)
- ice_accretion_3hr: Ice Accretion over 3 Hours (inches)
- ice_accretion_6hr: Ice Accretion over 6 Hours (inches)
- peak_wind_gust: Peak Wind Gust (from PK WND METAR remark) (knots)
- peak_wind_drct: Peak Wind Gust Direction (from PK WND METAR remark) (deg)
- peak_wind_time: Peak Wind Gust Time (from PK WND METAR remark)
- metar: unprocessed reported observation in METAR format

**Obs: for Sky Coverage values:**

- FEW - few clouds
- SCT - sparse
- BKN - cloudy
- OVC - overcast
"""

# %%
"""
### (i) Raw Data
"""

# %%
df_raw = pd.read_csv('SBRP.csv')
df_raw

# %%
df_raw.info(verbose=True)

# %%
"""
### (ii) Filtering by Important Features, Filling Missing Values and Changing DTypes
"""

# %%
"""
* **Selecting Features**
"""

# %%
df = df_raw.drop(['station','mslp','gust','p01i',
                  'skyc2','skyl2','skyc3','skyl3',
                  'skyc4','skyl4','wxcodes','ice_accretion_1hr',
                  'ice_accretion_3hr','ice_accretion_6hr','peak_wind_gust',
                  'peak_wind_drct','peak_wind_time','snowdepth'],axis=1)
df

# %%
"""
* **Missing Values (NaN)**
"""

# %%
pd.DataFrame({'Column Name':list(df.columns),
                            'Total of NaN':df.isna().sum().values,
                            'Percentage of NaN':[round((100*x/len(df)),3) for x in df.isna().sum().values]})

# %%
df_raw.skyc1.value_counts()

# %%
# Changing DType
df = df.astype({
    'valid':'datetime64[ns]',
    'skyc1':'category',
    'metar':'string'
})

# Filling Missing Values --> https://pandas.pydata.org/docs/reference/api/pandas.DataFrame.fillna.html
values = {'tmpf':df.tmpf.mean(),
          'dwpf':df.dwpf.mean(),
          'relh':df.relh.mean(),
          'drct':statistics.mode(df.drct),
          'sknt':df.sknt.mean(),
          'alti':df.alti.mean(),
          'vsby':df.vsby.mean(),
          'skyc1':statistics.mode(df_raw.skyc1.value_counts().index),
          'skyl1':df.skyl1.mean(),
          'feel':df.feel.mean()
         }

df = df.fillna(value=values)

df.info()

# %%
"""
## 2- Exploratory Data Analysis (EDA) and Feature Engineering
"""

# %%
"""
* **EDA**
"""

# %%
"""
### (i) Data Distribution
"""

# %%
round(df.describe(percentiles=[0.001,0.01,0.1,0.25,0.5,0.75,0.9,0.99,0.999]),3)

# %%
"""
### (ii) Histogram and BoxPlot
"""

# %%
contColNames = list(df.select_dtypes(include='number').columns) #-> seleciona apenas colunas numéricas
ncols = 3 #-> número de colunas que armazenarão os plots na figura 
nrows = int(np.ceil(len(contColNames)/(1.0 * ncols))) #-> número de linhas que armazenarão os plots na figure 


fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(25,12))

counter = 0 

for i in range(nrows):
  for j in range(ncols):

    ax = axes[i][j]

    # Plotar somente quando houver dados
    if counter < len(contColNames):

      ax.hist(df.select_dtypes(include='number')[contColNames[counter]], bins=30)
      ax.set_xlabel(contColNames[counter])
      ax.set_ylabel('Frequency')

    else:
      ax.set_axis_off()

    counter += 1

plt.show()

# %%
contColNames = list(df.select_dtypes(include='number').columns) #-> seleciona apenas colunas numéricas
ncols = 3 #-> número de colunas que armazenarão os plots na figure 
nrows = int(np.ceil(len(contColNames)/(1.0 * ncols))) #-> número de linhas que armazenarão os plots na figure 


fig, axes = plt.subplots(nrows=nrows, ncols=ncols, figsize=(25,12))

counter = 0 

for i in range(nrows):
  for j in range(ncols):

    ax = axes[i][j]

    # Plotar somente quando houver dados
    if counter < len(contColNames):
      red_square = dict(markerfacecolor='red', marker='s')
      ax.boxplot(df.select_dtypes(include='number')[contColNames[counter]], flierprops=red_square, vert=False, whis=0.75)
      ax.set_xlabel(contColNames[counter])

    else:
      ax.set_axis_off()

    counter += 1
plt.show()

# %%
"""
### (iii) Removing Outliers: Interquartile Range (IQR) Analysis
***Obs: for numerical features only***

As we could see previously, there are some data distributions that are not coherent. There are some visible gaps and expressive outliers in our mass - for this reason we are going to treat these data via an IQR approach.
"""

# %%
"""
* **dwpf: Dew Point Temperature in Fahrenheit**
"""

# %%
Q1 = df['dwpf'].quantile(0.25)
Q3 = df['dwpf'].quantile(0.75)
IQR = Q3 - Q1

lower_bound = Q1 - 1.5*IQR
upper_bound = Q3 + 1.5*IQR

print(df.dwpf.describe(percentiles=[0.01,0.1,0.25,0.5,0.75,0.9,0.99,0.999]),'\n\n')
print(f'Q1 = {round(Q1,3)}\nQ3 = {round(Q3,3)}\nIQR = {round(IQR,3)}')
print(f'Normal Range: ({round(lower_bound,3)} ~ {round(upper_bound,3)})')

# %%
new_values = []
for val in df.dwpf.values:
  if val < lower_bound:
    new_values.append(lower_bound)
  elif val > upper_bound:
    new_values.append(upper_bound)
  else:
    new_values.append(val)

sns.distplot(pd.DataFrame(new_values))

# %%
df.dwpf = new_values
df.dwpf.describe()

# %%
"""
* **relh: Relative Humidity in %***
"""

# %%
Q1 = df['relh'].quantile(0.25)
Q3 = df['relh'].quantile(0.75)
IQR = Q3 - Q1

lower_bound = Q1 - 1.5*IQR
upper_bound = Q3 + 1.5*IQR

print(df.relh.describe(percentiles=[0.01,0.1,0.25,0.5,0.75,0.9,0.99,0.999]),'\n\n')
print(f'Q1 = {round(Q1,3)}\nQ3 = {round(Q3,3)}\nIQR = {round(IQR,3)}')
print(f'Normal Range: ({round(lower_bound,3)} ~ {round(upper_bound,3)})')

# %%
new_values = []
for val in df.relh.values:
  if val < 0:
    new_values.append(0)
  elif val > 100:
    new_values.append(100)
  else:
    new_values.append(val)
  
sns.distplot(pd.DataFrame(new_values))

# %%
df.relh = new_values
df.relh.describe()

# %%
"""
* **alti:**
"""

# %%
Q1 = df['alti'].quantile(0.25)
Q3 = df['alti'].quantile(0.75)
IQR = Q3 - Q1

lower_bound = Q1 - 1.5*IQR
upper_bound = Q3 + 1.5*IQR

print(df.alti.describe(percentiles=[0.01,0.1,0.25,0.5,0.75,0.9,0.99,0.999]),'\n\n')
print(f'Q1 = {round(Q1,3)}\nQ3 = {round(Q3,3)}\nIQR = {round(IQR,3)}')
print(f'Normal Range: ({round(lower_bound,3)} ~ {round(upper_bound,3)})')

# %%
new_values = []
for val in df.alti.values:
  if val < lower_bound:
    new_values.append(lower_bound)
  elif val > upper_bound:
    new_values.append(upper_bound)
  else:
    new_values.append(val)

sns.distplot(pd.DataFrame(new_values))

# %%
df.alti = new_values
df.alti.describe()

# %%
"""
* **skyl1: Sky Level 1 Altitude in feet**
"""

# %%
Q1 = df['skyl1'].quantile(0.25)
Q3 = df['skyl1'].quantile(0.75)
IQR = Q3 - Q1

lower_bound = Q1 - 1.5*IQR
upper_bound = Q3 + 1.5*IQR

print(df.skyl1.describe(percentiles=[0.01,0.1,0.25,0.5,0.75,0.9,0.99,0.999]),'\n\n')
print(f'Q1 = {round(Q1,3)}\nQ3 = {round(Q3,3)}\nIQR = {round(IQR,3)}')
print(f'Normal Range: ({round(lower_bound,3)} ~ {round(upper_bound,3)})')

# %%
new_values = []
for val in df.skyl1.values:
  if val > upper_bound:
    new_values.append(upper_bound)
  else:
    new_values.append(val)
  
sns.distplot(pd.DataFrame(new_values))

# %%
df.skyl1 = new_values
df.skyl1.describe()

# %%
"""
### (iv) Feature Engineering

**Flight category definitions:** https://www.aviationweather.gov/taf/help?page=plot
"""

# %%
# Remembering our dataset
df.head()

# %%
"""
* **Creating new features**
"""

# %%
# 1- Time Series Decomposition:
df['year'] = df.valid.dt.year.values
df['month'] = df.valid.dt.month.values
df['dayofweek_flag'] = df.valid.dt.day_of_week.values #-> from 0 (sundays) to 7 (saturdays) 
df['day'] = df.valid.dt.day.values
df['hour'] = df.valid.dt.hour
df['min'] = df.valid.dt.minute

season_flag = []
for m in df.month:
  if m in (12,1,2):
    season_flag.append('summer')
  elif m in (3,4,5):
    season_flag.append('autumn')
  elif m in (6,7,8):
    season_flag.append('winter')
  else:
    season_flag.append('spring')
df['season_flag'] = season_flag


# 2- Flight Rules Code:
fr_code =[]
for v, slv in zip(df.vsby.values, df.skyl1.values):
  if (v > 5.) and (slv > 3e+03): # VFR: visibility > 5 miles and ceiling > 3000ft
    fr_code.append('VFR')
  elif (3. < v <= 5.) or (1e+03 < slv <= 3e+03): # MVFR: visibility between 3 and 5 miles and/or ceiling between 1000ft and 3000ft
    fr_code.append('MVFR')
  elif (1. < v <= 3.) or (5e+02 < slv <= 1e+03): # IFR: visibility between 1 and 3 miles and/or ceiling between 1000ft and 3000ft
    fr_code.append('IFR')
  elif (v <= 1.) or (slv <= 5e+02): # LIFR: visibility less than 1 mile and/or ceiling below 500ft
    fr_code.append('LIFR')
df['fr_code'] = fr_code

# Changimg DTypes
df = df.astype({
    'year':'int32',
    'month':'int32',
    'dayofweek_flag':'int32',
    'day':'int32',
    'season_flag':'category',
    'fr_code':'category'
})


df # checking for updates

# %%
"""
### (v) Correlation Matrix (Pearson's) and KDE Plot
"""

# %%
plt.figure(figsize=(15,10))
sns.heatmap(df[['tmpf', 'dwpf', 'relh', 'drct',
                               'sknt','alti','vsby','skyl1']].corr(), annot=True, cmap='Blues')

# %%
fig, axs = plt.subplots(figsize=(15,10))
plt.title("Relation 'Relative Humidity [%]' Vs. 'Air Temperature [°F]' Coloured by Their Density\n")
sns.kdeplot(
    data=df, x="tmpf", y="relh",
    fill=True, thresh=0, levels=100, cmap="inferno", cbar = True)

#https://seaborn.pydata.org/generated/seaborn.kdeplot.html

# %%
"""
### (vi) Pairplot colored by Flight Rule Code
"""

# %%
reduced_features = df[['tmpf','dwpf','relh','drct','sknt']]
cod = df.fr_code.values
reduced_features['Flight Condition'] = cod


# Make the pair plot with a some aesthetic changes
plt.figure(figsize=(30,25))
sns.pairplot(reduced_features, hue = 'Flight Condition', diag_kind = 'kde', palette='rainbow', plot_kws=dict(alpha = 0.7),
                   diag_kws=dict(shade=True))

# %%
"""
### (vii) Quantitative Analysis
"""

# %%
"""
* **Sky Level 1 Coverage**
"""

# %%
plt.figure(figsize=(10,8))
plt.pie(df.skyc1.value_counts(), labels = list(df.skyc1.value_counts().index), autopct='%1.1f%%', radius = 1.15, textprops = {"fontsize" : 12}) 
plt.show()

# %%
"""
* **Flight Rules Code**
"""

# %%
plt.figure(figsize=(10,8))
plt.pie(df.fr_code.value_counts(), labels = list(df.fr_code.value_counts().index), autopct='%1.1f%%', radius = 1.15, textprops = {"fontsize" : 12}) 
plt.show()

# %%
"""
### (viii) Aggregated Analysis
"""

# %%
"""
* **by Year**
"""

# %%
df_agg_year = df[['year','tmpf', 'dwpf', 'relh', 'drct','sknt','alti','vsby','skyl1']].groupby(by='year').agg(['min','mean','max'])
df_agg_year

# %%
plt.figure(figsize=(25,15))

# matplotlib colors: https://matplotlib.org/3.5.0/_images/sphx_glr_named_colors_003.png

plt.subplot(2, 3, 1)
plt.plot(df_agg_year.index.values, df_agg_year.tmpf['max'],'-o',color='b')
plt.xlabel('year')
plt.ylabel('[°F]')
plt.legend(['max tmpf'],loc='best')
plt.xticks(ticks=df_agg_year.index.values)

plt.subplot(2, 3, 2)
plt.plot(df_agg_year.index.values, df_agg_year.dwpf['max'],'-o',color='springgreen')
plt.xlabel('year')
plt.ylabel('[°F]')
plt.legend(['max dwpf'],loc='best')
plt.xticks(ticks=df_agg_year.index.values)

plt.subplot(2, 3, 3)
plt.plot(df_agg_year.index.values, df_agg_year.relh['max'],'-o',color='black')
plt.xlabel('year')
plt.ylabel('%')
plt.legend(['max relh'],loc='best')
plt.xticks(ticks=df_agg_year.index.values)

plt.subplot(2, 3, 4)
plt.plot(df_agg_year.index.values, df_agg_year.vsby['mean'],'-o',color='darkviolet')
plt.xlabel('year')
plt.ylabel('miles')
plt.legend(['mean vsby'],loc='best')
plt.xticks(ticks=df_agg_year.index.values)

plt.subplot(2, 3, 5)
plt.plot(df_agg_year.index.values, df_agg_year.skyl1['mean'],'-o',color='red')
plt.xlabel('year')
plt.ylabel('ft')
plt.legend(['mean skyl1'],loc='best')
plt.xticks(ticks=df_agg_year.index.values)

plt.subplot(2, 3, 6)
plt.plot(df_agg_year.index.values, df_agg_year.alti['mean'],'-o',color='orange')
plt.xlabel('year')
plt.ylabel('inches')
plt.legend(['mean alti'],loc='best')
plt.xticks(ticks=df_agg_year.index.values)

# %%
"""
* **by Month**
"""

# %%
df_agg_month = df[['month','tmpf', 'dwpf', 'relh', 'drct','sknt','alti','vsby','skyl1']].groupby(by='month').agg(['min','mean','max'])
df_agg_month

# %%
plt.figure(figsize=(25,15))

# matplotlib colors: https://matplotlib.org/3.5.0/_images/sphx_glr_named_colors_003.png

plt.subplot(2, 3, 1)
plt.plot(df_agg_month.index.values, df_agg_month.tmpf['max'],'-o',color='b')
plt.xlabel('month')
plt.ylabel('[°F]')
plt.legend(['max tmpf'],loc='best')
plt.xticks(ticks=df_agg_month.index.values)

plt.subplot(2, 3, 2)
plt.plot(df_agg_month.index.values, df_agg_month.dwpf['max'],'-o',color='springgreen')
plt.xlabel('month')
plt.ylabel('[°F]')
plt.legend(['max dwpf'],loc='best')
plt.xticks(ticks=df_agg_month.index.values)

plt.subplot(2, 3, 3)
plt.plot(df_agg_month.index.values, df_agg_month.relh['max'],'-o',color='black')
plt.xlabel('month')
plt.ylabel('%')
plt.legend(['max relh'],loc='best')
plt.xticks(ticks=df_agg_month.index.values)

plt.subplot(2, 3, 4)
plt.plot(df_agg_month.index.values, df_agg_month.vsby['mean'],'-o',color='darkviolet')
plt.xlabel('month')
plt.ylabel('miles')
plt.legend(['mean vsby'],loc='best')
plt.xticks(ticks=df_agg_month.index.values)

plt.subplot(2, 3, 5)
plt.plot(df_agg_month.index.values, df_agg_month.skyl1['mean'],'-o',color='red')
plt.xlabel('month')
plt.ylabel('ft')
plt.legend(['mean skyl1'],loc='best')
plt.xticks(ticks=df_agg_month.index.values)

plt.subplot(2, 3, 6)
plt.plot(df_agg_month.index.values, df_agg_month.alti['mean'],'-o',color='orange')
plt.xlabel('month')
plt.ylabel('inches')
plt.legend(['mean alti'],loc='best')
plt.xticks(ticks=df_agg_month.index.values)

# %%
"""
* **by Day of the Week**
"""

# %%
df_agg_dayofweek = df[['dayofweek_flag','tmpf', 'dwpf', 'relh', 'drct','sknt','alti','vsby','skyl1']].groupby(by='dayofweek_flag').agg(['min','mean','max'])
df_agg_dayofweek

# %%
plt.figure(figsize=(25,15))

# matplotlib colors: https://matplotlib.org/3.5.0/_images/sphx_glr_named_colors_003.png

plt.subplot(2, 3, 1)
plt.plot(df_agg_dayofweek.index.values, df_agg_dayofweek.tmpf['max'],'-o',color='b')
plt.xlabel('day of the week')
plt.ylabel('[°F]')
plt.legend(['max tmpf'],loc='best')
plt.xticks(ticks=df_agg_dayofweek.index.values)

plt.subplot(2, 3, 2)
plt.plot(df_agg_dayofweek.index.values, df_agg_dayofweek.dwpf['max'],'-o',color='springgreen')
plt.xlabel('day of the week')
plt.ylabel('[°F]')
plt.legend(['max dwpf'],loc='best')
plt.xticks(ticks=df_agg_dayofweek.index.values)

plt.subplot(2, 3, 3)
plt.plot(df_agg_dayofweek.index.values, df_agg_dayofweek.relh['max'],'-o',color='black')
plt.xlabel('day of the week')
plt.ylabel('%')
plt.legend(['max relh'],loc='best')
plt.xticks(ticks=df_agg_dayofweek.index.values)

plt.subplot(2, 3, 4)
plt.plot(df_agg_dayofweek.index.values, df_agg_dayofweek.vsby['mean'],'-o',color='darkviolet')
plt.xlabel('day of the week')
plt.ylabel('miles')
plt.legend(['mean vsby'],loc='best')
plt.xticks(ticks=df_agg_dayofweek.index.values)

plt.subplot(2, 3, 5)
plt.plot(df_agg_dayofweek.index.values, df_agg_dayofweek.skyl1['mean'],'-o',color='red')
plt.xlabel('day of the week')
plt.ylabel('ft')
plt.legend(['mean skyl1'],loc='best')
plt.xticks(ticks=df_agg_dayofweek.index.values)

plt.subplot(2, 3, 6)
plt.plot(df_agg_dayofweek.index.values, df_agg_dayofweek.alti['mean'],'-o',color='orange')
plt.xlabel('day of the week')
plt.ylabel('inches')
plt.legend(['mean alti'],loc='best')
plt.xticks(ticks=df_agg_dayofweek.index.values)

# %%
"""
* **by Season**
"""

# %%
df_agg_season = df[['season_flag','tmpf', 'dwpf', 'relh', 'drct','sknt','alti','vsby','skyl1']].groupby(by='season_flag').agg(['min','mean','max'])
df_agg_season

# %%
plt.figure(figsize=(25,15))

# matplotlib colors: https://matplotlib.org/3.5.0/_images/sphx_glr_named_colors_003.png

plt.subplot(2, 3, 1)
plt.plot(df_agg_season.index.values, df_agg_season.tmpf['max'],'-o',color='b')
plt.xlabel('season')
plt.ylabel('[°F]')
plt.legend(['max tmpf'],loc='best')
plt.xticks(ticks=df_agg_season.index.values)

plt.subplot(2, 3, 2)
plt.plot(df_agg_season.index.values, df_agg_season.dwpf['max'],'-o',color='springgreen')
plt.xlabel('season')
plt.ylabel('[°F]')
plt.legend(['max dwpf'],loc='best')
plt.xticks(ticks=df_agg_season.index.values)

plt.subplot(2, 3, 3)
plt.plot(df_agg_season.index.values, df_agg_season.relh['max'],'-o',color='black')
plt.xlabel('season')
plt.ylabel('%')
plt.legend(['max relh'],loc='best')
plt.xticks(ticks=df_agg_season.index.values)

plt.subplot(2, 3, 4)
plt.plot(df_agg_season.index.values, df_agg_season.vsby['mean'],'-o',color='darkviolet')
plt.xlabel('season')
plt.ylabel('miles')
plt.legend(['mean vsby'],loc='best')
plt.xticks(ticks=df_agg_season.index.values)

plt.subplot(2, 3, 5)
plt.plot(df_agg_season.index.values, df_agg_season.skyl1['mean'],'-o',color='red')
plt.xlabel('season')
plt.ylabel('ft')
plt.legend(['mean skyl1'],loc='best')
plt.xticks(ticks=df_agg_season.index.values)

plt.subplot(2, 3, 6)
plt.plot(df_agg_season.index.values, df_agg_season.alti['mean'],'-o',color='orange')
plt.xlabel('season')
plt.ylabel('inches')
plt.legend(['mean alti'],loc='best')
plt.xticks(ticks=df_agg_season.index.values)

# %%
"""
### (ix) Time Series
"""

# %%
fig = px.area(df[['fr_code','tmpf', 'dwpf', 'relh', 'drct','sknt','alti','vsby','skyl1']], facet_col="fr_code",facet_col_wrap=2)
fig.show()

# %%
fig = px.line(df, x='valid', y='tmpf', title='Air Temperature [°F]')

fig.update_xaxes(
    rangeslider_visible=True,
    rangeselector=dict(
        buttons=list([
            dict(count=1, label="1m", step="month", stepmode="backward"),
            dict(count=6, label="6m", step="month", stepmode="backward"),
            dict(count=1, label="YTD", step="year", stepmode="todate"),
            dict(count=1, label="1y", step="year", stepmode="backward"),
            dict(step="all")
        ])
    )
)
fig.show()

# %%
fig = px.line(df, x='valid', y='sknt', title='Wind Speed [knots]')

fig.update_xaxes(
    rangeslider_visible=True,
    rangeselector=dict(
        buttons=list([
            dict(count=1, label="1m", step="month", stepmode="backward"),
            dict(count=6, label="6m", step="month", stepmode="backward"),
            dict(count=1, label="YTD", step="year", stepmode="todate"),
            dict(count=1, label="1y", step="year", stepmode="backward"),
            dict(step="all")
        ])
    )
)
fig.show()

# %%
fig = px.line(df, x='valid', y='alti', title='Pressure altimeter [inches]')

fig.update_xaxes(
    rangeslider_visible=True,
    rangeselector=dict(
        buttons=list([
            dict(count=1, label="1m", step="month", stepmode="backward"),
            dict(count=6, label="6m", step="month", stepmode="backward"),
            dict(count=1, label="YTD", step="year", stepmode="todate"),
            dict(count=1, label="1y", step="year", stepmode="backward"),
            dict(step="all")
        ])
    )
)
fig.show()

# %%
"""
## 3- Machine Learning Algorythm
"""

# %%
"""
### (i) Creating a new DataFrame
"""

# %%
# Creating a copy from the previous DF
df_model = df.copy()
df_model.head()

# %%
"""
* **First we need to filter the data that was logged hourly**
"""

# %%
df_model.loc[df['min']!=0] # Checking the values that don't match with our 'hourly' logged data

# %%
# Counting the frequency registered by each minute log
df_model['min'].value_counts()

# %%
"""
* **Hourly Logged DataFrame**
"""

# %%
df_model = df_model.loc[df['min']==0] # values registered from hour to hour
df_model

# %%
# Confirming the changes
df_model['min'].value_counts()

# %%
"""
* **Selecting variables of interest (summarized data)**
"""

# %%
df_model_s = df_model.drop(['valid','vsby','skyl1','feel',
                            'metar','year','day','hour','min'],axis=1)
df_model_s

# %%
"""
### (ii) One-Hot encoding
"""

# %%
"""
* **Changing month and day_of_week values**
"""

# %%
new_val_month = []
new_val_dow = []

for m,d in zip(df_model_s.month.values, df_model_s.dayofweek_flag.values):
  
  # for months
  if m==1:
    new_val_month.append('Jan')
  elif m==2:
    new_val_month.append('Feb')
  elif m==3:
    new_val_month.append('Mar')
  elif m==4:
    new_val_month.append('Apr')
  elif m==5:
    new_val_month.append('May')
  elif m==6:
    new_val_month.append('Jun')
  elif m==7:
    new_val_month.append('Jul')
  elif m==8:
    new_val_month.append('Aug')
  elif m==9:
    new_val_month.append('Sep')
  elif m==10:
    new_val_month.append('Oct')
  elif m==11:
    new_val_month.append('Nov')
  else:
    new_val_month.append('Dec')

  # for days of the week:
  if d==0:
    new_val_dow.append('Sun')
  elif d==1:
    new_val_dow.append('Mon')
  elif d==2:
    new_val_dow.append('Tues')
  elif d==3:
    new_val_dow.append('Wed')
  elif d==4:
    new_val_dow.append('Thurs')
  elif d==5:
    new_val_dow.append('Fri')
  else:
    new_val_dow.append('Sat')


df_model_s['month'] = new_val_month
df_model_s['dayofweek_flag'] = new_val_dow
df_model_s.head()

# %%
"""
* **Encoding**
"""

# %%
df_model_s = pd.get_dummies(df_model_s, columns=['skyc1','month','dayofweek_flag','season_flag','fr_code'])
df_model_s

# %%
df_model_s.columns

# %%
"""
### (iii) Creating X,y matrices

For this analysis, we are going to consider an 'hourly' prediction, e.g, we use the previous data to predict the next hour variables of interest. Our data will be structured as following:

- X: has all the variables, including only 'even' rows;
    - columns: **['tmpf', 'dwpf', 'relh', 'drct', 'alti', 'skyc1_BKN',
       'skyc1_FEW', 'skyc1_NSC', 'skyc1_OVC', 'skyc1_SCT', 'skyc1_VV ',
       'month_Apr', 'month_Aug', 'month_Dec', 'month_Feb', 'month_Jan',
       'month_Jul', 'month_Jun', 'month_Mar', 'month_May', 'month_Nov',
       'month_Oct', 'month_Sep', 'dayofweek_flag_Fri', 'dayofweek_flag_Mon',
       'dayofweek_flag_Sat', 'dayofweek_flag_Sun', 'dayofweek_flag_Thurs',
       'dayofweek_flag_Tues', 'dayofweek_flag_Wed', 'season_flag_autumn',
       'season_flag_spring', 'season_flag_summer', 'season_flag_winter',
       'fr_code_IFR', 'fr_code_LIFR', 'fr_code_MVFR', 'fr_code_VFR']**

- y: has the interest variables to be predicted, e.g, the float ones. Includes the 'odd' rows
    - columns: **['tmpf', 'relh', 'sknt', 'alti']**
"""

# %%
# To make an hourly prediction we need to have the total number of rows as an even number:

if len(df_model_s)%2 == 1: #division by 2 results 1 for odd numbers and 0 for even numbers
  df_model_s = df_model_s.iloc[[x for x in range(0,len(df_model_s)-1)]] #discards the last row
else:
  pass

df_model_s

# %%
"""
* **X, y matrices:**
"""

# %%
start_x, end_x = 0, len(df_model_s)-1

# iterating each number in list
x_pos = []
for num in range(start_x, end_x):
    # checking condition
    if num % 2 == 0:
        x_pos.append(num)

X = df_model_s.iloc[x_pos]
X

# %%
start_y, end_y = 1, len(df_model_s)

# iterating each number in list
y_pos = []
for num in range(start_y, end_y):
    # checking condition
    if num % 2 != 0:
        y_pos.append(num)

y = df_model_s[['tmpf', 'relh', 'sknt', 'alti']].iloc[y_pos]
y

# %%
"""
### (iv) Train/test data
"""

# %%
X_train, X_test, y_train, y_test = train_test_split(X, y, test_size = 0.25, random_state = 42)

print(X_train.shape)
print(X_test.shape)
print(y_train.shape)
print(y_test.shape)

# %%
"""
### (v) Selecting the regression model
"""

# %%
"""
* **Manifold**
"""

# %%
from sklearn.linear_model import LinearRegression, LassoLars
from sklearn import svm
from sklearn.ensemble import RandomForestRegressor
from sklearn.ensemble import GradientBoostingRegressor

# %%
from sklearn import metrics
from sklearn.multioutput import MultiOutputRegressor

linear_model = MultiOutputRegressor(LinearRegression())
linear_model.fit(X_train,y_train)
y_pred_linear = linear_model.predict(X_test)

R2_tmpf_linear = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_linear[:,0]))
R2_relh_linear = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_linear[:,1]))
R2_sknt_linear = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_linear[:,2]))
R2_alti_linear = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_linear[:,3]))
Avg_R2_linear = (1/4)*(R2_tmpf_linear + R2_relh_linear + R2_sknt_linear + R2_alti_linear)

print(y_pred_linear)

# %%
from sklearn import metrics
from sklearn.multioutput import MultiOutputRegressor

lars_model = MultiOutputRegressor(LassoLars())
lars_model.fit(X_train,y_train)
y_pred_lars = linear_model.predict(X_test)

R2_tmpf_lars = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_lars[:,0]))
R2_relh_lars = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_lars[:,1]))
R2_sknt_lars = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_lars[:,2]))
R2_alti_lars = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_lars[:,3]))
Avg_R2_lars = (1/4)*(R2_tmpf_lars + R2_relh_lars + R2_sknt_lars + R2_alti_lars)

print(y_pred_lars)

# %%
from sklearn import metrics
from sklearn.multioutput import MultiOutputRegressor

svm_model = MultiOutputRegressor(svm.SVR())
svm_model.fit(X_train,y_train)
y_pred_svm = svm_model.predict(X_test)

R2_tmpf_svm = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_svm[:,0]))
R2_relh_svm = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_svm[:,1]))
R2_sknt_svm = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_svm[:,2]))
R2_alti_svm = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_svm[:,3]))
Avg_R2_svm = (1/4)*(R2_tmpf_svm + R2_relh_svm + R2_sknt_svm + R2_alti_svm)

print(y_pred_svm)

# %%
from sklearn import metrics
from sklearn.multioutput import MultiOutputRegressor

rf_model = MultiOutputRegressor(RandomForestRegressor())
rf_model.fit(X_train,y_train)
y_pred_rf = rf_model.predict(X_test)

R2_tmpf_rf = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_rf[:,0]))
R2_relh_rf = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_rf[:,1]))
R2_sknt_rf = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_rf[:,2]))
R2_alti_rf = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_rf[:,3]))
Avg_R2_rf = (1/4)*(R2_tmpf_rf + R2_relh_rf + R2_sknt_rf + R2_alti_rf)

print(y_pred_rf)

# %%
from sklearn import metrics
from sklearn.multioutput import MultiOutputRegressor

gb_model = MultiOutputRegressor(GradientBoostingRegressor())
gb_model.fit(X_train,y_train)
y_pred_gb = gb_model.predict(X_test)

R2_tmpf_gb = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_gb[:,0]))
R2_relh_gb = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_gb[:,1]))
R2_sknt_gb = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_gb[:,2]))
R2_alti_gb = abs(metrics.r2_score(np.array(y_test.tmpf),y_pred_gb[:,3]))
Avg_R2_gb = (1/4)*(R2_tmpf_gb + R2_relh_gb + R2_sknt_gb + R2_alti_gb)

print(y_pred_rf)

# %%
metrics = pd.DataFrame({'model':['Linear','LARS Lasso','SVM','Random Forest','Gradient Boosting'],
                        'r2_tmpf':[R2_tmpf_linear,R2_tmpf_lars,R2_tmpf_svm,R2_tmpf_rf,R2_tmpf_gb],
                        'r2_relh':[R2_relh_linear,R2_relh_lars,R2_relh_svm,R2_relh_rf,R2_relh_gb],
                        'r2_sknt':[R2_sknt_linear,R2_sknt_lars,R2_sknt_svm,R2_sknt_rf,R2_sknt_gb],
                        'r2_alti':[R2_alti_linear,R2_alti_lars,R2_alti_svm,R2_alti_rf,R2_alti_gb],
                        'Avg_r2':[Avg_R2_linear,Avg_R2_lars,Avg_R2_svm,Avg_R2_rf,Avg_R2_gb]})
metrics

# %%
"""
* **Specific**
"""

# %%
from sklearn.ensemble import GradientBoostingRegressor
from sklearn.multioutput import MultiOutputRegressor
from sklearn.model_selection import RandomizedSearchCV

model = MultiOutputRegressor(GradientBoostingRegressor(learning_rate=0.1, n_estimators=100, subsample=1.0,
                                                       criterion='friedman_mse', min_samples_split=2,
                                                       min_samples_leaf=1,
                                                       min_weight_fraction_leaf=0.0, max_depth=3,
                                                       min_impurity_decrease=0.0, init=None, random_state=42,
                                                       max_features=None,
                                                       alpha=0.9, verbose=0, max_leaf_nodes=None, warm_start=False,
                                                       validation_fraction=0.1, n_iter_no_change=None, tol=0.0001,
                                                       ccp_alpha=0.0))

hyperparameters = dict(estimator__learning_rate=[0.05, 0.1, 0.2, 0.5, 0.9],
                     estimator__n_estimators=[20, 50, 100, 200, 300, 500, 700, 1000],
                     estimator__criterion=['friedman_mse', 'mse'], estimator__min_samples_split=[2, 4, 7, 10],
                     estimator__max_depth=[3, 5, 10, 15, 20, 30], estimator__min_samples_leaf=[1, 2, 3, 5, 8, 10],
                     estimator__min_impurity_decrease=[0, 0.2, 0.4, 0.6, 0.8],
                     estimator__max_leaf_nodes=[5, 10, 20, 30, 50, 100, 300])

randomized_search = RandomizedSearchCV(model, hyperparameters, random_state=42, n_iter=5, scoring=None,
                                       n_jobs=2, refit=True, cv=5, verbose=True,
                                       pre_dispatch='2*n_jobs', error_score='raise', return_train_score=True)

# %%
hyperparameters_tuning = randomized_search.fit(X_train, y_train)
print('Best Parameters = {}'.format(hyperparameters_tuning.best_params_))

# %%
reg_model = MultiOutputRegressor(GradientBoostingRegressor(n_estimators=700,min_samples_split=2,min_impurity_decrease=0,
                                                           max_leaf_nodes=20,max_depth=3,learning_rate=0.01,
                                                           criterion='friedman_mse'))

reg_model = reg_model.fit(X_train, y_train)

# %%
"""
### (vi) Results
"""

# %%
"""
* **Predicted Values (y)**
"""

# %%
y_pred = reg_model.predict(X_test)
y_pred

# %%
"""
* **Metrics**
"""

# %%
from sklearn import metrics

print('-='*30)
print('tmpf:\n')
print("MAE: {}".format(metrics.mean_absolute_error(np.array(y_test.tmpf),y_pred[:,0])))
print("RMSE: {}".format(metrics.mean_squared_error(np.array(y_test.tmpf),y_pred[:,0], squared=True)))
print("R²: {}".format(metrics.r2_score(np.array(y_test.tmpf),y_pred[:,0])))
print('\n')

print('-='*30)
print('relh:\n')
print("MAE: {}".format(metrics.mean_absolute_error(np.array(y_test.relh),y_pred[:,1])))
print("RMSE: {}".format(metrics.mean_squared_error(np.array(y_test.relh),y_pred[:,1], squared=True)))
print("R²: {}".format(metrics.r2_score(np.array(y_test.relh),y_pred[:,1])))
print('\n')

print('-='*30)
print('sknt:\n')
print("MAE: {}".format(metrics.mean_absolute_error(np.array(y_test.sknt),y_pred[:,2])))
print("RMSE: {}".format(metrics.mean_squared_error(np.array(y_test.sknt),y_pred[:,2], squared=True)))
print("R²: {}".format(metrics.r2_score(np.array(y_test.sknt),y_pred[:,2])))
print('\n')

print('-='*30)
print('alti:\n')
print("MAE: {}".format(metrics.mean_absolute_error(np.array(y_test.alti),y_pred[:,3])))
print("RMSE: {}".format(metrics.mean_squared_error(np.array(y_test.alti),y_pred[:,3], squared=True)))
print("R²: {}".format(metrics.r2_score(np.array(y_test.alti),y_pred[:,3])))
print('\n')

# %%
"""
* **Results DF**
"""

# %%
results = pd.DataFrame({'tmpf_actual':y_test.tmpf,
                        'tmpf_pred':y_pred[:,0],
                        'relh_actual':y_test.relh,
                        'relh_pred':y_pred[:,1],
                        'sknt_actual':y_test.sknt,
                        'sknt_pred':y_pred[:,2],
                        'alti_actual':y_test.alti,
                        'alti_pred':y_pred[:,3]
                       })

results

# %%
"""
### (vii) System Response Analysis
"""

# %%
"""
* **Correlation**
"""

# %%
plt.figure(figsize=(15,10))
matrix = results.corr()
mask = np.triu(np.ones_like(matrix, dtype=bool))
sns.heatmap(matrix, mask=mask, annot=True,cmap='PuBuGn')

# %%
"""
* **Air Temperature [°F]**
"""

# %%
results.tmpf_actual.hist(bins=30)
results.tmpf_pred.hist(bins=30)
plt.legend(('Actual','Predicted'))
plt.xlabel('Air Temperature [°F]')
plt.ylabel('Frequency')

# %%
plt.scatter(results.tmpf_actual,results.tmpf_pred, c='b')
plt.xlabel('Actual Air Temperature [°F]')
plt.ylabel('Predicted Air Temperature [°F]')

# %%
"""
* **Relative Humidity [%]**
"""

# %%
results.relh_actual.hist(bins=30)
results.relh_pred.hist(bins=30)
plt.legend(('Actual','Predicted'))
plt.xlabel('Relative Humidity [%]')
plt.ylabel('Frequency')

# %%
plt.scatter(results.relh_actual,results.relh_pred, c='b')
plt.xlabel('Actual Relative Humidity [%]')
plt.ylabel('Predicted Relative Humidity [%]')

# %%
"""
* **Windspeed [knots]**
"""

# %%
results.sknt_actual.hist(bins=30)
results.sknt_pred.hist(bins=30)
plt.legend(('Actual','Predicted'))
plt.xlabel('Wind Speed [knots]')
plt.ylabel('Frequency')

# %%
plt.scatter(results.sknt_actual,results.sknt_pred, c='b')
plt.xlabel('Actual Wind Speed [knots]')
plt.ylabel('Predicted Wind Speed [knots]')

# %%
"""
* **Pressure Altimeter [inches]**
"""

# %%
results.alti_actual.hist(bins=30)
results.alti_pred.hist(bins=30)
plt.legend(('Actual','Predicted'))
plt.xlabel('Pressure Altimeter [inches]')
plt.ylabel('Frequency')

# %%
plt.scatter(results.alti_actual,results.alti_pred, c='b')
plt.xlabel('Actual Pressure Altimeter [inches]')
plt.ylabel('Predicted Pressure Altimeter [inches]')

# %%
"""
* **Residuals DataFrame**
"""

# %%
residuals = pd.DataFrame({'residuals_tmpf':results.tmpf_pred - results.tmpf_actual,
                          'residuals_relh':results.relh_pred - results.relh_actual,
                          'residuals_sknt':results.sknt_pred - results.sknt_actual,
                          'residuals_alti':results.alti_pred - results.alti_actual,
                        })
residuals

# %%
plt.figure(figsize=(15,8))
plt.title('Error Distribution')
sns.distplot(residuals.residuals_tmpf,hist=False,bins=50,rug=True)
sns.distplot(residuals.residuals_relh,hist=False,bins=50,rug=True)
sns.distplot(residuals.residuals_sknt,hist=False,bins=50,rug=True)
plt.legend(['residuals_tmpf','residuals_relh','residuals_sknt'])
plt.xlabel('Value')


# %%
plt.figure(figsize=(15,8))
plt.title('Error Distribution')
sns.distplot(residuals.residuals_alti,hist=False,bins=50,rug=True)
plt.legend(['residuals_alti'])
plt.xlabel('Value')

# %%
"""
### (viii) Model Interpretability
"""

# %%
from sklearn.tree import export_graphviz
import random

y_col_random = random.randint(0, y.shape[1]-1) #selects a random column (y) for visualization
print(f'Selected y: {y.columns[y_col_random]}')
x_var, y_var = X_train, y_train.values[:,y_col_random]

model = GradientBoostingRegressor(n_estimators=700,min_samples_split=2,min_impurity_decrease=0,
                                                           max_leaf_nodes=10,max_depth=3,learning_rate=0.01,
                                                           criterion='friedman_mse').fit(x_var, y_var)

# Get the tree number
random_tree_number = random.randint(0,100) # selects a random tree number for visualization
print(f'Selected tree: {random_tree_number}')
sub_tree = model.estimators_[random_tree_number, 0]

# Visualization
# Install graphviz: https://www.graphviz.org/download/
from pydotplus import graph_from_dot_data
from IPython.display import Image
dot_data = export_graphviz(
    sub_tree,
    out_file=None, filled=True, rounded=True,
    special_characters=True,
    proportion=False, impurity=False, # enable them if you want
    feature_names = list(X.columns)
)
graph = graph_from_dot_data(dot_data)
Image(graph.create_png())

# %%
!pip install shap

# %%
# https://github.com/slundberg/shap
import shap

# %%
"""
* **tmpf**
"""

# %%
x_var, y_var = X_train, y_train.values[:,0]

model = GradientBoostingRegressor(n_estimators=700,min_samples_split=2,min_impurity_decrease=0,
                                                           max_leaf_nodes=10,max_depth=3,learning_rate=0.01,
                                                           criterion='friedman_mse').fit(x_var, y_var)

# explain the model's predictions using SHAP
# (same syntax works for LightGBM, CatBoost, scikit-learn, transformers, Spark, etc.)
explainer = shap.Explainer(model)
shap_values = explainer(x_var)

# %%
# visualize the first prediction's explanation with a force plot
shap.initjs()
shap.plots.force(shap_values[0])

# %%
# create a dependence scatter plot to show the effect of a single feature across the whole dataset
shap.initjs()
shap.plots.scatter(shap_values[:,0], color=shap_values)

# %%
#take the mean absolute value of the SHAP values for each feature to get a standard bar plot
shap.initjs()
shap.plots.bar(shap_values)

# %%
"""
* **relh**
"""

# %%
x_var, y_var = X_train, y_train.values[:,1]

model = GradientBoostingRegressor(n_estimators=700,min_samples_split=2,min_impurity_decrease=0,
                                                           max_leaf_nodes=10,max_depth=3,learning_rate=0.01,
                                                           criterion='friedman_mse').fit(x_var, y_var)

# explain the model's predictions using SHAP
# (same syntax works for LightGBM, CatBoost, scikit-learn, transformers, Spark, etc.)
explainer = shap.Explainer(model)
shap_values = explainer(x_var)

# %%
# visualize the first prediction's explanation with a force plot
shap.initjs()
shap.plots.force(shap_values[0])

# %%
# create a dependence scatter plot to show the effect of a single feature across the whole dataset
shap.initjs()
shap.plots.scatter(shap_values[:,0], color=shap_values)

# %%
#take the mean absolute value of the SHAP values for each feature to get a standard bar plot
shap.initjs()
shap.plots.bar(shap_values)

# %%
"""
* **sknt**
"""

# %%
x_var, y_var = X_train, y_train.values[:,2]

model = GradientBoostingRegressor(n_estimators=700,min_samples_split=2,min_impurity_decrease=0,
                                                           max_leaf_nodes=10,max_depth=3,learning_rate=0.01,
                                                           criterion='friedman_mse').fit(x_var, y_var)

# explain the model's predictions using SHAP
# (same syntax works for LightGBM, CatBoost, scikit-learn, transformers, Spark, etc.)
explainer = shap.Explainer(model)
shap_values = explainer(x_var)

# %%
# visualize the first prediction's explanation with a force plot
shap.initjs()
shap.plots.force(shap_values[0])

# %%
# create a dependence scatter plot to show the effect of a single feature across the whole dataset
shap.initjs()
shap.plots.scatter(shap_values[:,0], color=shap_values)

# %%
#take the mean absolute value of the SHAP values for each feature to get a standard bar plot
shap.initjs()
shap.plots.bar(shap_values)

# %%
"""
* **alti**
"""

# %%
x_var, y_var = X_train, y_train.values[:,3]

model = GradientBoostingRegressor(n_estimators=700,min_samples_split=2,min_impurity_decrease=0,
                                                           max_leaf_nodes=10,max_depth=3,learning_rate=0.01,
                                                           criterion='friedman_mse').fit(x_var, y_var)

# explain the model's predictions using SHAP
# (same syntax works for LightGBM, CatBoost, scikit-learn, transformers, Spark, etc.)
explainer = shap.Explainer(model)
shap_values = explainer(x_var)

# %%
# visualize the first prediction's explanation with a force plot
shap.initjs()
shap.plots.force(shap_values[0])

# %%
# create a dependence scatter plot to show the effect of a single feature across the whole dataset
shap.initjs()
shap.plots.scatter(shap_values[:,0], color=shap_values)

# %%
#take the mean absolute value of the SHAP values for each feature to get a standard bar plot
shap.initjs()
shap.plots.bar(shap_values)