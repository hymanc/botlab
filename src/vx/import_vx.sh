#!/bin/bash
set -e
# Script assumes you have dngv, ngv, april checked out in HOME.

BASE_DIR=$HOME

mkdir -p $BASE_DIR/april/src/vx/gtk
mkdir -p $BASE_DIR/april/src/vx/shaders

#VX_FILES=`cd $BASE_DIR/dngv/src/vx && git ls-tree --name-only --full-name HEAD *.[ch]`
VX_FILES=`find $BASE_DIR/dngv/src/vx -maxdepth 1 -type f -regex ".*\([ch]\|Makefile\)$"`

VX_GTK_FILES=`find $BASE_DIR/dngv/src/vx/gtk -maxdepth 1 -type f -regex ".*\([ch]\|Makefile\)$"`

for f in $VX_FILES; do
    fout=`echo $f | sed s,$BASE_DIR/dngv,$BASE_DIR/april,`
    CMD="cp $f $fout"
    echo $CMD
    $CMD
done

for f in $VX_GTK_FILES; do
    fout=`echo $f | sed s,$BASE_DIR/dngv,$BASE_DIR/april,`
    CMD="cp $f $fout"
    echo $CMD
    $CMD
done

cp -r $BASE_DIR/dngv/src/vx/shaders/* $BASE_DIR/april/src/vx/shaders/
cp -r $BASE_DIR/dngv/src/vx/README $BASE_DIR/april/src/vx/
cp -r $BASE_DIR/dngv/src/vx/.gitignore $BASE_DIR/april/src/vx/
cp -r $BASE_DIR/dngv/src/vx/gtk/.gitignore $BASE_DIR/april/src/vx/gtk/


# Copy dependencies into common

#jhstrom@blt ~/dngv/src/vx master$ git grep include | grep common | cut -d" " -f 2 | sed s/\"//g | cut -d/ -f 2 | sort | uniq
COMMON_FILES="c5.h matd.h string_util.h zarray.h zhash.h"
COMMON_DNGV_FILES="task_thread.h ssocket.h ioutils.h image_u8.h image_util.h"

#getopt.h

#image_util.h
#matd_coords.h
#time_util.h

cp $BASE_DIR/dngv/src/common/math_util.h  $BASE_DIR/april/src/common/math_util.h

for f in $COMMON_FILES; do
    f2=`echo $f | sed s/h$/c/`
    CMD="cp $BASE_DIR/ngv/src/common/$f $BASE_DIR/april/src/common/"
    CMD2="cp $BASE_DIR/ngv/src/common/$f2 $BASE_DIR/april/src/common/"
    echo $CMD
    $CMD
    echo $CMD2
    $CMD2
done

for f in $COMMON_DNGV_FILES; do
    f2=`echo $f | sed s/h$/c/`
    CMD="cp $BASE_DIR/dngv/src/common/$f $BASE_DIR/april/src/common/"
    CMD2="cp $BASE_DIR/dngv/src/common/$f2 $BASE_DIR/april/src/common/"
    echo $CMD
    $CMD
    echo $CMD2
    $CMD2
done
