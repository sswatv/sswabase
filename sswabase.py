#######################################
# Station Information and Calibration #
#######################################
stationTerrainElevationMeters=536.55  #
stationPressureHeightMeters=1.55      #
stationPressureOffsethPa=-0.80        #
vaneOffset=0                          #
vaneADCMin=0                          #
vaneADCMax=1650                       #
#######################################

################################################
# Imports                                      #
################################################
import sys                                     #
import traceback                               #
import math                                    #
import time                                    #
import board                                   #
import lgpio                                   #
from adafruit_tmp117 import TMP117             #
from adafruit_shtc3 import SHTC3               #
from adafruit_bmp5xx import BMP5XX             #
from adafruit_ads1x15 import ADS1015,AnalogIn  #
################################################

########################################################
# Sensor Definitions and Configuration                 #
########################################################
i2c=board.I2C()                                        #
inth=lgpio.gpiochip_open(0)                            #
sensorT=TMP117(i2c)                                    #
sensorT.measurement_mode=0                             # =CONTINUOUS
sensorT.measurement_delay=4                            # =1S
sensorT.averaged_measurements=3                        # =64
sensorRH=SHTC3(i2c)                                    #
sensorP=BMP5XX.over_i2c(i2c)                           #
sensorP.data_ready_int_en=False                        #
sensorP.iir_flush_forced=False                         #
sensorP.mode=1                                         # =NORMAL
sensorP.output_data_rate=28                            # =1Hz
sensorP.pressure_iir_filter=3                          # =7x
sensorP.pressure_oversampling_rate=7                   # =128x
sensorP.pressure_shadow_iir=0                          # =BEFORE
sensorP.temperature_shadow_iir=0                       # =BEFORE
sensorP.temperature_iir_filter=2                       # =3x
sensorP.temperature_oversampling_rate=3                # =8x
sensorADCint=ADS1015(i2c,address=0x49,data_rate=3300)  #
sensorADCint.gain=1                                    #
sensorADCint.mode=0                                    #
sensorADC=AnalogIn(sensorADCint,0)                     #
########################################################

#### Wind Callback ####
windclicks=[]
winddirs=[]
def windCallback(a,b,c,d):
    global windclicks
    windclicks.append(round(time.time()*1000))
    try:
        winddirs.append([round(time.time()*1000),((((((sensorADC.value>>4)-vaneADCMin)/(vaneADCMax-vaneADCMin))*360)+vaneOffset)%360)])
    except:
        print(f"================ WDIR FAIL ================",file=sys.stderr)
        pass

#### Rain Callback ####
rainclicks=[]
def rainCallback(a,b,c,d):
    global rainclicks
    rainclicks.append(round(time.time()*1000))

#######################################################################
# Callback Setup                                                      #
#######################################################################
lgpio.gpio_claim_alert(inth,26,lgpio.SET_PULL_UP,lgpio.FALLING_EDGE)  #
lgpio.gpio_set_debounce_micros(inth,26,16000)                         #
lgpio.callback(inth,26,lgpio.FALLING_EDGE,windCallback)               #
lgpio.gpio_claim_alert(inth,16,lgpio.SET_PULL_UP,lgpio.FALLING_EDGE)  #
lgpio.gpio_set_debounce_micros(inth,16,800000)                        #
lgpio.callback(inth,16,lgpio.FALLING_EDGE,rainCallback)               #
#######################################################################

#### Uptime Calc ####
boottime=time.time()
def uptime(timestamp):
    return int(timestamp-boottime)//60

#### Temperature Fetch ####
def temperature():
    raw=sensorT.temperature
    reading=raw
    return reading

#### Humidity Fetch ####
def humidity():
    raw=sensorRH.relative_humidity
    reading=raw
    return reading

#### Pressure Fetch ####
def pressure():
    raw=sensorP.pressure+stationPressureOffsethPa
    reading=raw*(1-0.0065*(stationTerrainElevationMeters+stationPressureHeightMeters)/288.15)**(-9.80665/(287.05*0.0065))
    return raw,reading

#### Wind Fetch ####
def wind(timestamp):
    global windclicks,winddirs
    timestamp=int(timestamp*1000)
    cutoff5=timestamp-300000
    cutoff2=timestamp-120000
    windclicks[:]=[x for x in windclicks if x >= cutoff5]
    winddirs[:]=[x for x in winddirs if x[0] >= cutoff2]
    localwc5=windclicks[:]
    localwd2=winddirs[:]
    speed2=sum(1 for x in localwc5 if x >= cutoff2)*(2.25/120)*0.44704
    gust5=0
    left=0
    for right in range(len(localwc5)):
        while localwc5[right]-localwc5[left]>3000:
            left+=1
        clicks=right-left+1
        if clicks>gust5:
            gust5=clicks
    gust5=gust5*(2.25/3)*0.44704
    if localwd2:
        r=[math.radians(x[1]) for x in localwd2]
        dir2=(math.degrees(math.atan2(sum(math.sin(x) for x in r),sum(math.cos(x) for x in r)))+360)%360
    else:
        dir2=0
    return speed2,dir2,gust5

#### Precip Fatch ####
def precip(timestamp):
    global rainclicks
    timestamplocal=time.localtime(timestamp)
    midnightlocal=int(time.mktime(time.struct_time((timestamplocal.tm_year,timestamplocal.tm_mon,timestamplocal.tm_mday,0,0,0,timestamplocal.tm_wday,timestamplocal.tm_yday,timestamplocal.tm_isdst)))*1000)
    timestamp=int(timestamp*1000)
    cutoff24hr=timestamp-86400000
    cutoff1hr=timestamp-3600000
    rainclicks[:]=[x for x in rainclicks if x >= cutoff24hr]
    localrc=rainclicks[:]
    rain1=sum(1 for x in localrc if x >= cutoff1hr)*0.2
    rain24=len(localrc)*0.2
    rainmn=sum(1 for x in localrc if x >= midnightlocal)*0.2
    return rain1,rain24,rainmn

#### Dew Point Calculation ####
def dewpoint(temp,humi):
    if temp>=0:
        a=17.62
        b=243.12
    else:
        a=22.46
        b=272.62
    gamma=math.log(humi/100)+(a*temp)/(b+temp)
    dewp=(b*gamma)/(a-gamma)
    return dewp

#### Main Loop ####
while True:
    try:
        time.sleep(60-(time.time()%60))
        timestamp=time.time()-0.5
        upmins=uptime(timestamp)
        temp=temperature()
        humi=humidity()
        statpres,seapres=pressure()
        wspeed,wdir,wgust=wind(timestamp)
        rain1hr,rain24hr,rainmn=precip(timestamp)
        dewp=dewpoint(temp,humi)
    except KeyboardInterrupt:
        exit()
    except Exception:
        print(f"================ {time.strftime('%Y-%m-%d %H:%M:%S',time.gmtime(timestamp))} ================",file=sys.stderr)
        print(f"================ {upmins} ================",file=sys.stderr)
        traceback.print_exc(file=sys.stderr)
        pass
