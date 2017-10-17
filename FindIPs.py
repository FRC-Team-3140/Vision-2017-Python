#######
####### This code is to run automatically when the Pi is turned on, and will find the IP the rio is on and put it in a text document to be read by the main vision file
#######

import subprocess as sp
import re

RioMacAddress = '6C:40:08:2C:93:8A' #Put the Rio's MAC Address here

regex = ur"(\d+[.]\d+[.]\d+[.]\d+).*?(\w\w:\w\w:\w\w:\w\w:\w\w:\w\w)" #Used to pull the IPs and MAC Addresses from the console
netMask = ur"[0-9]{1,3}[.][0-9]{1,3}[.][0-9]{1,3}[.]" #Used to find the subnet mask

myIP = sp.check_output("hostname -I", shell=True) #Gets my IP
netMaskIP = re.findall(netMask, myIP) #Finds the subnet Mask
print("Finding devices on subnet mask {ip}".format(ip=netMaskIP[0]))

rioIP = ''
macAddressCorrect = False 

def Connect():
    global regex, netMask, myIP, netMaskIP, macAddressCorrect, rioIP
    macAddresses = sp.check_output("sudo nmap -sn {netmask}0/24".format(netmask=netMaskIP[0]), shell=True) #Searches all devices on the network for MacAddress and IPs
    parsedMacAddress = re.findall(regex, macAddresses, re.DOTALL) #Sorts out the MacAddresses
    print("{pIP} Mac Addresses have been found".format(pIP=len(parsedMacAddress)))

    for thisMac in parsedMacAddress:
        if thisMac[1] == RioMacAddress:
            rioIP = thisMac[0]
            print("Rio Connected, Local IP is {ip}".format(ip=rioIP))
            macAddressCorrect = True
            break

while macAddressCorrect == False:
    Connect()
    if (macAddressCorrect == False): #This is hear just to keep this message from displaying the first time the code is run
        print("Failed MAC Address Validation, trying again")

IPFile = open("ipdoc.txt", "w") #Opens the text document that the vision code will read for the IP
IPFile.write(rioIP) #Writes the IP into the document
IPFile.close()
print("Done Connecting")
