#!/bin/bash

echo "--- Comms Config ---"

echo "Enter Node Address"
echo "Deployment prefix = 2a01:348:24b:2:c30c::/64"
read node

echo "Enter POST Interval In Seconds"
read interval

echo "Enter POST Proxy Address (No ::)"
echo "Deployment = 2a01:348:24b:2:0:0:0:1"
echo "Weird private one =  aaaa:0:0:0:0:0:0:1" 
read proxy

a=`echo "$proxy" | cut -d ':' -f 1`
b=`echo "$proxy" | cut -d ':' -f 2`
c=`echo "$proxy" | cut -d ':' -f 3`
d=`echo "$proxy" | cut -d ':' -f 4`
e=`echo "$proxy" | cut -d ':' -f 5`
f=`echo "$proxy" | cut -d ':' -f 6`
g=`echo "$proxy" | cut -d ':' -f 7`
h=`echo "$proxy" | cut -d ':' -f 8`

echo "Enter POST Proxy Port"
echo "Default = 8081"
read port

if wget -O- 'http://['$node']/comsub?interval='$interval'&port='$port'&a='$a'&b='$b'&c='$c'&d='$d'&e='$e'&f='$f'&g='$g'&h='$h'&submit=Submit'
then
	echo "Successfully configured comms"
fi
