#!/bin/bash
# compress log files ready for transfer to master server

set -e
shopt -s nullglob

cd logs || exit 1
for d in 20*; do
    pushd $d || continue
    echo "Checking $d"
    for run in run*; do
        if test $(find "$run" -name "$run" -type d -mmin +15)
        then
            echo "Compressing $d/$run"
            tar -cJf $run.tar.xz $run
            sync
            rm -rf $run
        fi
    done
    # remove empty directories that have been transferred
    popd
    rmdir --ignore-fail-on-non-empty $d || /bin/true
done
