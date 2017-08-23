#!/bin/bash

if [ -z "${PYTHON3_INTERPRETER}" ]; then
    PYTHON3_INTERPRETER="python3"
fi

cd ../python
$PYTHON3_INTERPRETER  nbody.py $*
exit $?
