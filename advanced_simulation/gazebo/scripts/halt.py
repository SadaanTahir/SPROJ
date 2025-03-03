import time
from utils import *
# from pymavlink import mavutil



conn = connect_n_drones(1)[0]
setmode(conn,'GUIDED')
arm(conn)
takeoff(conn)



lat, lon = -1,-1
newlat, newlon = move_gps(lat,lon,100,90)
moveglobal(conn,newlat,newlon,5)

print('setting mode')
setmode(conn,'BRAKE')
