#!/bin/bash

set -e
cd $HOME
(
    date
    FWLoad/scripts/compress_logs.sh
) >> cron.log 2>&1

