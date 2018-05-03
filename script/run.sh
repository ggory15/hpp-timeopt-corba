#!/bin/bash         

gepetto-viewer-server & 
hpp-timeopt-server &
ipython -i --no-confirm-exit ./$1

pkill -f  'gepetto-viewer-server'
pkill -f  'hpp-timeopt-server'
