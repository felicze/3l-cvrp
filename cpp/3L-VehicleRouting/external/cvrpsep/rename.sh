#!/bin/bash
# A script to copy and rename my source code to the public github version
# which matches the original naming convention and code layout from
# https://econ.au.dk/research/researcher-websites/jens-lysgaard/cvrpsep/.

#First I made all file names lowercase. The original had a mix.
for f in *.CPP
do
    echo $f " -> " ${f,,}
    git mv $f ${f,,}
done
for f in *.H
do
    echo $f " -> " ${f,,}
    git mv $f ${f,,}
done

