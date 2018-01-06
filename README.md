Custom KK (3.4.67) Kernel Source  for Blu Advance 4.0 L and clones.


The Project config file is in /mediatek/config/codename/ProjectConfig.mk

##INFO##
The codename is the name of your device. Because this source support many different devices, each codename is different.
Example: Blu Advance 4.0 L codename is s4050ap. So the config file will be in /mediatek/config/s4050ap/ProjectConfig.mk

##Compiling the kernel##

      *You need a 64 bit Linux based OS (BBQLinux,Ubuntu) with a proper build environment. A compiler toolchain like GCC 4.7. 
      
      *Set the toolchain path in the build_vibe_kernel.sh script.
      
      *Type ./build_vibe_kernel.sh or bash build_vibe_kernel.sh to start the compile process. 

      *After finish building, you can find the kernel in /out/target/product/codename/obj/KERNEL_OBJ/arch/arm/boot/zImage
      
 
##Flashing the compiled kernel##

	  *I've added AnyKernel2 support so flashing the zip would work on any ROM. 


