#!/usr/bin/python3

import socket

def main():
    #Start with relay state off
    relayState = False
    #Get Hostname
    hostname = socket.gethostname()
    print(socket.gethostname())

if __name__ == "__main__":
   main()
