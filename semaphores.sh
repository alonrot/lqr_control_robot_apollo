#!/bin/bash
for i in `ipcs | awk '{print $2}' | grep -e "[0-9].*"`
do
     ipcrm -m $i
     ipcrm -s $i
     ipcrm -q $i
     ipcrm -M $i
     ipcrm -S $i
     ipcrm -Q $i
done