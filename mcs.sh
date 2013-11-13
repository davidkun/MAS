#!/bin/bash

for i in `seq 1 100`;
do
  python test.py --simple --save
  python test.py --adv --save
  echo "Iteration: $i"
done 