#!/bin/bash

set -e
cd $HOME
(
    date
    sudo chown -R runjig logs 
    FWLoad/scripts/compress_logs.sh
) >> cron.log 2>&1
