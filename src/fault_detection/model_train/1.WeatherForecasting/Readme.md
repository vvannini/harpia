# Predictive System Using Metar Data (SBRP)

> **Main database:**  https://mesonet.agron.iastate.edu/request/download.phtml?network=BR__ASOS

![image](https://user-images.githubusercontent.com/60454486/167980061-04c1055b-6c96-42ba-8f43-40e6da65a562.png)


The IEM maintains an ever growing archive of automated airport weather observations from around the world! These observations are typically called 'ASOS' or sometimes 'AWOS' sensors. A more generic term may be METAR data, which is a term that describes the format the data is transmitted as. This archive simply provides the as-is collection of historical observations, very little quality control is done. More details on this dataset are here: https://mesonet.agron.iastate.edu/info/datasets/metar.html

* **ASOS User's Guide:** https://www.weather.gov/media/asos/aum-toc.pdf

*  **Tools/Libaries:** Here (https://github.com/akrherz/iem/blob/main/scripts/asos/iem_scraper_example.py) is a python script example  that automates the download of data from this interface. A community user has contributed R language  version of the python script. There is also a riem R package  allowing for easy access to this archive.