#!/bin/bash
#Original script by hellsgod and modified with functions from MSF Jarvis

# Bash colors
green='\033[01;32m'
red='\033[01;31m'
cyan='\033[01;36m'
blue='\033[01;34m'
blink_red='\033[05;31m'
restore='\033[0m'

clear

# Paths
KERNEL_DIR=$(pwd)
RESOURCE_DIR="${KERNEL_DIR}"
ANYKERNEL_DIR="$KERNEL_DIR/Vibe-AK2-s4050ap"
REPACK_DIR="$ANYKERNEL_DIR"
ZIP_MOVE="$RESOURCE_DIR/VB-releases/s4050ap/"
ZIMAGE_DIR="$KERNEL_DIR/out/target/product/s4050ap/obj/KERNEL_OBJ/arch/arm/boot/"

OLD_DIR="$ZIP_MOVE/old"

# Resources
THREAD="-j4"
KERNEL="zImage"

# Vibe Kernel Details
KERNEL_NAME="Vibe"
VER="v1.0"
LOCALVERSION="-$( date +%Y%m%d )"
BASE_VB_VER="VB"
DEVICE="s4050ap"
#VB_VER=$BASE_VER$VER-${DEVICE}-$( TZ=MST date +%Y%m%d-%H%M )
VB_VER="$BASE_VB_VER$VER-$DEVICE${LOCALVERSION}-$( date +%H%M )"


# Vars
export ARCH=arm
export KBUILD_BUILD_USER=kirito9
export KBUILD_BUILD_HOST=aincrad
export CROSS_COMPILE=~/android/toolchain/arm-eabi-4.9/bin/arm-eabi-

# Functions
function clean_all {
		rm -rf $REPACK_DIR/zImage
		rm -rf $ZIMAGE_DIR/$KERNEL
		make clean && make mrproper
		rm -rf out/
}

function make_kernel {
		./makeMtk -t -o=TARGET_BUILD_VARIANT=user s4050ap n k
		cp -vr $ZIMAGE_DIR/$KERNEL $REPACK_DIR
		mv $REPACK_DIR/$KERNEL $REPACK_DIR/zImage
}

function make_zip {
		cd $REPACK_DIR
		zip -9 -r `echo $VB_VER`.zip .		
		if [[ ! -d "${OLD_DIR}" ]]; then
			 mkdir -p "${OLD_DIR}"
		fi				
		cd $ZIP_MOVE
		mv *.zip old
		mv "${REPACK_DIR}"/*.zip "${ZIP_MOVE}"		
		cd $KERNEL_DIR
}

DATE_START=$(date +"%s")

echo -e "${green}"
echo "Vibe Kernel Creation Script:"
echo

echo "---------------"
echo "Kernel Version:"
echo "---------------"

echo -e "${red}"; echo -e "${blink_red}"; echo "$VB_VER"; echo -e "${restore}";

echo -e "${green}"
echo "-----------------"
echo "Making Vibe Kernel:"
echo "-----------------"
echo -e "${restore}"

while read -p "Please choose your option: [1]Clean-build // [2]Dirty-build // [3]Clean source // [4]Make flashable zip // [5]Abort " cchoice
do
case "$cchoice" in
	1 )
		echo -e "${green}"
		echo
		echo "[..........Cleaning up..........]"
		echo
		echo -e "${restore}"
		clean_all
		echo -e "${green}"
		echo
		echo "[....Building `echo $VB_VER`....]"
		echo
		echo -e "${restore}"
		make_kernel
		if [ -f $ZIMAGE_DIR/$KERNEL ];
		then
			make_zip
		else
			echo -e "${red}"
			echo
			echo "Kernel build failed, check your logs!"
			echo
			echo -e "${restore}"
		fi
		echo
		echo
		echo -e "${restore}"
		break
		;;
	2 )
		echo -e "${green}"
		echo
		echo "[....Building `echo $VB_VER`....]"
		echo
		echo -e "${restore}"
		make_kernel
		if [ -f $ZIMAGE_DIR/$KERNEL ];
		then
			make_zip
		else
			echo -e "${red}"
			echo
			echo "Kernel build failed, check your logs!"
			echo
			echo -e "${restore}"
		fi
		echo		
		echo
		echo -e "${restore}"
		break
		;;
	3 )
		echo -e "${green}"
		echo
		echo "[..........Cleaning up..........]"
		echo
		echo -e "${restore}"
		clean_all
		echo -e "${green}"
		echo
		echo -e "${restore}"		
		break
		;;
	4 )
		echo -e "${green}"
		echo
		echo "[....Make `echo $VB_VER`.zip....]"
		echo
		echo -e "${restore}"
		make_zip
		echo -e "${green}"
		echo
		echo "[.....Moving `echo $VB_VER`.....]"
		echo
		echo -e "${restore}"
		break
		;;
	5 )
		break
		;;
	* )
		echo -e "${red}"
		echo
		echo "Invalid try again!"
		echo
		echo -e "${restore}"
		;;		
esac
done


if [ -f $ZIMAGE_DIR/$KERNEL ];
then
	echo -e "${green}"
	echo "-------------------"
	echo "Build Completed in:"
	echo "-------------------"
	echo -e "${restore}"
fi



DATE_END=$(date +"%s")
DIFF=$(($DATE_END - $DATE_START))
echo "Time: $(($DIFF / 60)) minute(s) and $(($DIFF % 60)) seconds."
echo
