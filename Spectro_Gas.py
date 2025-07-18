# This is the code that we will use to code the Raspberry pi for the spectrometer, arduino, and pressure transducer communication
# This code will include the threshold limit to begin scanning as well as the characater pointers that the spectrometer can read from its manual
# This protocal will require an ethernet connection to a computer and a USB 2.0 connection to the spectrometer. 
# The Pi can make the spectrometer do its commands using the charcter pointers for a scan, save the scan as a .csv file to the Pi, and then send it to the computer using communication protocol to send the files to the computer 
import csv
import pyusb
import datetime
import os
import pandas
import paramiko # This is to send the files via SCP/SSH protocal
import socket # This is to send the files via TCP protocal



# The commands that I am sending to the spectrometer are character pointers form the manual that do scanning, movement, measurement, transfer etc. 
print('hello world')
