# AnyKernel2 Ramdisk Mod Script
# osm0sis @ xda-developers

## AnyKernel setup
properties() {
kernel.string=BlackThunder by kirito9
do.devicecheck=0
do.modules=0
do.cleanup=1
do.cleanuponabort=0
device.name1=yaris_m_gsm
device.name2=4032A
device.name3=mts982t
} # end properties

# shell variables
block="/dev/bootimg bs=6291456 count=1";
is_slot_device=0;

## AnyKernel methods (DO NOT CHANGE)
# import patching functions/variables - see for reference
. /tmp/anykernel/tools/ak2-core.sh;


## AnyKernel permissions
# set permissions for included ramdisk files
chmod -R 755 $ramdisk

## AnyKernel install
dump_boot;

# begin ramdisk changes

# end ramdisk changes

write_boot;

## end install

