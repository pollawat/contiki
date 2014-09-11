#!/bin/bash

echo "--- Clock Config ---"

echo "Enter Node Address"
read node

year=`date +"%Y"`
month=`date +"%m"`
day=`date +"%d"`
hour=`date +"%H"`
min=`date +"%M"`
sec=`date +"%S"`

if wget -O- 'http://['$node']/clock?y='$year'&mo='$month'&d='$day'&h='$hour'&mi='$min'&s='$sec'&submit=Submit'
then
	echo "Successfully configured clock"
fi
