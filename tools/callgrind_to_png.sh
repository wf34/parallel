#!/bin/bash
# arg1 - input callgrind.out.NNNN, ag2 - image_name.png
gprof2dot --format=callgrind --output=out.dot $1
dot -Tpng out.dot -o $2
