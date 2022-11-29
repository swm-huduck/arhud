#!/bin/sh
DIR="$( cd "$( dirname "$0" )" && pwd -P )"
glslangValidator -V $DIR/shader.vert -o $DIR/vert.spv
glslangValidator -V $DIR/shader.frag -o $DIR/frag.spv
glslangValidator -V $DIR/shader_notex.frag -o $DIR/shader_notex_frag.spv
glslangValidator -V $DIR/shader_notex.vert -o $DIR/shader_notex_vert.spv