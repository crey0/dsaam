#!/bin/bash
cd ../..
$PYTHON3_INTERPRETER -m dsaam.ros.nbody $*
exit $?
