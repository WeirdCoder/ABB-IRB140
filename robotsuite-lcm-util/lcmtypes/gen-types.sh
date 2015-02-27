#!/bin/sh

lcm-gen -p ./*.lcm
lcm-gen -j ./*.lcm
javac -cp /usr/local/share/java/lcm.jar ./abblcm/*.java
jar cf IRB140.jar ./abblcm/*.class
