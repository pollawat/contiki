#!/bin/bash

echo "--- Comms Config ---"

echo "Enter Node Address"
read node

echo "Enter POST Interval In Seconds"
read interval

echo "Enter POST Proxy Address (No ::)"
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
read port

if wget -O- 'http://['$node']/comsub?interval='$interval'&port='$port'&a='$a'&b='$b'&c='$c'&d='$d'&e='$e'&f='$f'&g='$g'&h='$h'&submit=Submit'
then
	echo "Successfully configured comms"
fi
