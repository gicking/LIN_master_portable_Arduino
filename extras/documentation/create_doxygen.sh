#!/bin/bash 

# change to current working directory
cd `dirname $0`

# remove old documentation
rm -fr html

# create new documentation
doxygen Doxyfile
