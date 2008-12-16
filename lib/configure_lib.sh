#!/bin/sh
SDCC_PATH=/usr/share/sdcc
cp ./inc2h.pl $SDCC_PATH
chmod a+x $SDCC_PATH/inc2h.pl
OLD_DIR=`pwd`
cd $SDCC_PATH/include/pic
for file in `ls /usr/share/gputils/header/p*.inc`
do export picstring=`echo $file | sed "s/\/usr\/share\/gputils\/header\/p//; s/.inc//"`
../../inc2h.pl $picstring /usr/share/gputils > pic$picstring.h
done
cd $OLD_DIR
