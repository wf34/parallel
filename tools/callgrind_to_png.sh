#!/bin/bash
# arg1 - input callgrind.out.NNNN, ag2 - image_name.png
if [ "$#" -ne 2 ]; then
    echo "Illegal number of parameters"
    exit 1
fi
gprof2dot --format=callgrind --output=out.dot $1
dot -Tpng out.dot -o $2
