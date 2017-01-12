#!/usr/bin/env python
import socket   
import sys 
import math
import numpy as np
from numpy import matrix
import time
from xml.dom import minidom
import os.path



class CrclClientSocket:
    def __init__(self, host, port):
        self.host=host
        self.port=port
        self.stopconnecting=False
        self.nextdata=''

    def connect(self):
        try:
            if(self.stopconnecting):
                return
            self.sock = socket.socket(
                    socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
        except socket.error, msg:
            print 'Failed to create socket. Error code: ' + str(msg[0]) + ' , Error message : ' + msg[1]
            time.sleep( 5 )
            self.connect()
   
    def disconnect(self):
        self.sock.close()
 
    def syncsend(self, msg):
        print  msg
        sent = self.sock.send(msg)
        if sent == 0:
            self.disconnect()
            self.connect()  
            # raise RuntimeError("socket connection broken")      
 
    # http://code.activestate.com/recipes/408859/

    def syncreceive(self, end):
        #total_data=[];
        self.End=end # '</CRCLStatus>'
        data=''
        alldata=self.nextdata
        while True:
                data=self.sock.recv(8192)
                if data == 0:
                    alldata='' # empty string
                    return 
                alldata=alldata+data
                if self.End in alldata:
                    alldata=alldata[:alldata.find(self.End)]
                    self.nextdata=data[data.find(self.End)+1:]
                    break
        return alldata  # ''.join(total_data)


#time.sleep(10)
mysocket = CrclClientSocket("localhost", 31000)
print 'Socket Created'
mysocket.connect()
print 'Socket Connected'

# q  - quit &send quit
# quit  - quit send quit
while True:    # infinite loop
    msg = raw_input("> ")
    if msg == "q":
    	mysocket.syncsend("quit\n")
        break  # stops the loop
    elif msg == "quit":
    	mysocket.syncsend("quit\n")
        break  # stops the loop
    else:
       mysocket.syncsend(msg+"\n")
mysocket.disconnect()        







