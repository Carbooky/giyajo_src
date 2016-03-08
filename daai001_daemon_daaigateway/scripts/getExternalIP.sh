#/bin/bash
data=`(curl -s http://checkip.dyndns.org/ | grep -o "[[:digit:].]\+")`
echo $data

