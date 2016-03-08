#!/bin/sh
CMD=`(ls /media/info_backup/ |head -n$1 > backup_n_list_name.log)`
echo $CMD
