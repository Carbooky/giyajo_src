#/bin/bash
data=`(ls ./info_cache | grep -v @ | grep -v / > cacheFileLists.txt)`
echo $data

