#!/bin/bash

if [ -z "${$PYTHON3_INTERPRETER}" ]; then
    PYTHON3_INTERPRETER="python3"
fi

cd ../..
${PYTHON3_INTERPRETER} -m dsaam.ros.nbody $*
exit $?
