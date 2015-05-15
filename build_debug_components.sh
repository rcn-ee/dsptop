#!/bin/bash
set -e

if [ $# -ne 4 ]; then
  echo "Usage: $0 <debug_comp_base_dir> <mcsdk_hpc_dir> <output_dir> <linux_kernel_dir>";
  exit 1;
fi

cd $1
DEBUG_COMPONENTS_DIR=`pwd`
cd - 

cd $2
MCSDK_HPC_DIR=`pwd`
cd -

cd $3
OUTPUT_DIR=`pwd`
cd -

cd $4
LINUX_KERNEL_DIR=`pwd`
cd -

[ ! -z $DEBUG_COMPONENTS_DIR ] || { echo "Error: DEBUG_COMPONENTS_DIR not set!"; exit 1; }
[ ! -z $MCSDK_HPC_DIR ] || { edcho "Error: MCSDK_HPC_DIR not set!"; exit 1; }
[ ! -z $OUTPUT_DIR ] || { echo "Error: OUTPUT_DIR not set!"; exit 1; }

cd $MCSDK_HPC_DIR/scripts
set +e
. setup_hpc_env.sh -s $TI_INSTALL_DIR:$TI_INSTALL_DIR/linaro -t $TARGET_ROOTDIR -d
set -e

export TI_OCL_CGT_INSTALL=$C6X_GEN_DIR
export PATH=$PATH:${TOOLCHAIN_DIR}/bin:$TI_OCL_CGT_INSTALL/bin
export LINUX_DEVKIT_ROOT
export LINUX_KERNEL_DIR
export BIOS_DIR
export XDAIS_DIR
export XDC_DIR
export FC_DIR

#----------------------------------------------------------------------
# Build kernel so that the kernel modules can be built.
#----------------------------------------------------------------------
cd $LINUX_KERNEL_DIR
KERNEL_VER=`grep "VERSION =" $LINUX_KERNEL_DIR/Makefile | sed -e 's|[^0-9]||g'`
KERNEL_VER=${KERNEL_VER}.`grep "PATCHLEVEL =" $LINUX_KERNEL_DIR/Makefile | sed -e 's|[^0-9]||g'`
KERNEL_VER=${KERNEL_VER}.`grep "SUBLEVEL =" $LINUX_KERNEL_DIR/Makefile | sed -e 's|[^0-9]||g'`
KERNEL_EXTRA_VER=`grep "EXTRAVERSION =" $LINUX_KERNEL_DIR/Makefile | cut -d "=" -f2`
KERNEL_EXTRA_VER=`echo $KERNEL_EXTRA_VER | tr -d ' '`
echo KERNEL_EXTRA_VER = $KERNEL_EXTRA_VER
if [ -n "$KERNEL_EXTRA_VER" ]; then
     KERNEL_VER=${KERNEL_VER}-${KERNEL_EXTRA_VER}
fi
KERNEL_VER=`echo $KERNEL_VER | tr -d ' '`

# Build kernel to generate files needed by kernel module. NOT NEEDED HERE , as it is done earlier in do_release.sh itself
#ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make mrproper
#ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make keystone2_defconfig
#ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- make

#----------------------------------------------------------------------
# Begin build and package for dsptop utility.
#----------------------------------------------------------------------
DSPTOP_ROOT_DIR=$DEBUG_COMPONENTS_DIR/dsptop
cd $DSPTOP_ROOT_DIR/dsptop/

# Obtain dsptop version from dsptop.c.
DSPTOP_VER=`grep "MAJOR_VERSION" version.h | sed -e 's|[^0-9]||g'`
DSPTOP_VER=${DSPTOP_VER}.`grep "MINOR_VERSION" version.h | sed -e 's|[^0-9]||g'`
DSPTOP_VER=${DSPTOP_VER}.`grep "PATCH_VERSION" version.h | sed -e 's|[^0-9]||g'`
echo DSPTOP_VER=$DSPTOP_VER

# Clean dsptop directory, package, and move to artifacts directory.
sudo rm -rf $DSPTOP_ROOT_DIR/dsptop_$DSPTOP_VER
sudo rm -rf $DSPTOP_ROOT_DIR/install_dsptop

# Tar up source and send to artifacts
cd $DSPTOP_ROOT_DIR
mkdir -pv dsptop_$DSPTOP_VER
cp -rf $DSPTOP_ROOT_DIR/dsptop/* dsptop_$DSPTOP_VER
tar -zcvf dsptop_$DSPTOP_VER.tar.gz --exclude='ncurses' dsptop_$DSPTOP_VER
echo mv dsptop_$DSPTOP_VER.tar.gz $OUTPUT_DIR
mv dsptop_$DSPTOP_VER.tar.gz $OUTPUT_DIR

# Build dsptop for ipk packaging.
cd $DSPTOP_ROOT_DIR/dsptop_$DSPTOP_VER
make ncurses arm release man DEVICE=C66AK2Hxx CROSS_COMPILE=arm-linux-gnueabihf-

# Install dsptop to a temp folder for ipk packaging.
mkdir -pv $DSPTOP_ROOT_DIR/install_dsptop
make install DESTDIR=$DSPTOP_ROOT_DIR/install_dsptop

# Create the ipk and move to artifacts directory.
sed -i $MCSDK_HPC_DIR/mkrel/ipk/dsptop/control -e 's|Version:.*$|Version: '$DSPTOP_VER'|g'
sudo $MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/dsptop $DSPTOP_ROOT_DIR/install_dsptop
sudo mv -v *.ipk $OUTPUT_DIR

# Clean up
sudo rm -rf $DSPTOP_ROOT_DIR/dsptop_$DSPTOP_VER
sudo rm -rf $DSPTOP_ROOT_DIR/install_dsptop

#----------------------------------------------------------------------
# Begin build and package for debugss kernel module used by dsptop.
#----------------------------------------------------------------------
cd $DEBUG_COMPONENTS_DIR/dsptop/debugss_module/

# Obtain debugss kernel module version.
cd $DEBUG_COMPONENTS_DIR/dsptop/debugss_module/debugss-mod
DSS_KM_VER=`grep "DEBUGSS_KM_MAJOR_VERSION" debugss_kmodule.h | sed -e 's|[^0-9]||g'`
DSS_KM_VER=${DSS_KM_VER}.`grep "DEBUGSS_KM_MINOR_VERSION" debugss_kmodule.h | sed -e 's|[^0-9]||g'`
DSS_KM_VER=${DSS_KM_VER}.`grep "DEBUGSS_KM_PATCH_VERSION" debugss_kmodule.h | sed -e 's|[^0-9]||g'`

# Create a directory in parent directory to build DKMS module
cd $DEBUG_COMPONENTS_DIR/dsptop
DKMS_DSS_ROOT_DIR=$DEBUG_COMPONENTS_DIR/dsptop/debugss-mod-dkms-$DSS_KM_VER
DSS_ROOT_DIR=$DEBUG_COMPONENTS_DIR/dsptop/debugss-mod-dkms-$DSS_KM_VER/debugss-mod-$DSS_KM_VER
sudo rm -rf $DKMS_DSS_ROOT_DIR
mkdir -p $DKMS_DSS_ROOT_DIR
mkdir -p $DSS_ROOT_DIR

# Assemble the tar ball directory structure.
cd $DEBUG_COMPONENTS_DIR/dsptop/debugss_module/
echo cp -rf debugss-mod/* $DSS_ROOT_DIR
cp -rf debugss-mod/* $DSS_ROOT_DIR
cp -rf debian $DKMS_DSS_ROOT_DIR

# Create tar ball for PPA.
cd $DKMS_DSS_ROOT_DIR
DSS_KM_PPA_VER=$(dpkg-parsechangelog | sed -n 's/^Version: //p')
cd $DEBUG_COMPONENTS_DIR/dsptop
tar cvfz debugss-mod-dkms-${DSS_KM_PPA_VER}.tar.gz debugss-mod-dkms-$DSS_KM_VER

# Move to artifacts
mv -f debugss-mod-dkms-${DSS_KM_PPA_VER}.tar.gz $OUTPUT_DIR

# Build kernel modules ipk
cd $DSS_ROOT_DIR
sudo rm -rf $DSPTOP_ROOT_DIR/install_debugss
sudo mkdir $DSPTOP_ROOT_DIR/install_debugss
sudo chmod 777 $DSPTOP_ROOT_DIR/install_debugss
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=$LINUX_KERNEL_DIR KVERSION=$KERNEL_VER DESTDIR=$DSPTOP_ROOT_DIR/install_debugss make install_ipk

# Create the ipk and move to artifacts directory.
sed -i $MCSDK_HPC_DIR/mkrel/ipk/debugss-mod/control -e 's|Version:.*$|Version: '$DSS_KM_VER'|g'
sudo $MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/debugss-mod $DSPTOP_ROOT_DIR/install_debugss
sudo mv -v *.ipk $OUTPUT_DIR

# Clean up
cd $DEBUG_COMPONENTS_DIR/dsptop/
sudo rm -rf $DKMS_DSS_ROOT_DIR
sudo rm -rf $DSPTOP_ROOT_DIR/install_debugss

#----------------------------------------------------------------------
# Begin build and package for temperature kernel module used by dsptop.
#----------------------------------------------------------------------
cd $DEBUG_COMPONENTS_DIR/dsptop/temperature_module/

# Obtain temperature module version.
cd $DEBUG_COMPONENTS_DIR/dsptop/temperature_module/temperature-mod
TEMP_KM_VER=`grep "TEMP_KM_MAJOR_VERSION" temperature_kmodule.c | sed -e 's|[^0-9]||g'`
TEMP_KM_VER=${TEMP_KM_VER}.`grep "TEMP_KM_MINOR_VERSION" temperature_kmodule.c | sed -e 's|[^0-9]||g'`
TEMP_KM_VER=${TEMP_KM_VER}.`grep "TEMP_KM_PATCH_VERSION" temperature_kmodule.c | sed -e 's|[^0-9]||g'`

# Create a directory in parent directory to build DKMS module
cd $DEBUG_COMPONENTS_DIR/dsptop
DKMS_TEMP_ROOT_DIR=$DEBUG_COMPONENTS_DIR/dsptop/temperature-mod-dkms-$TEMP_KM_VER
TEMP_ROOT_DIR=$DEBUG_COMPONENTS_DIR/dsptop/temperature-mod-dkms-$TEMP_KM_VER/temperature-mod-$TEMP_KM_VER
sudo rm -rf $DKMS_TEMP_ROOT_DIR
mkdir -p $DKMS_TEMP_ROOT_DIR
mkdir -p $TEMP_ROOT_DIR

# Assemble the tar ball directory structure.
cd $DEBUG_COMPONENTS_DIR/dsptop/temperature_module/
echo cp -rf temperature-mod/* $TEMP_ROOT_DIR
cp -rf temperature-mod/* $TEMP_ROOT_DIR
cp -rf debian $DKMS_TEMP_ROOT_DIR

# Create tar ball for PPA.
cd $DKMS_TEMP_ROOT_DIR
TEMP_KM_PPA_VER=$(dpkg-parsechangelog | sed -n 's/^Version: //p')
cd $DEBUG_COMPONENTS_DIR/dsptop
tar cvfz temperature-mod-dkms-${TEMP_KM_PPA_VER}.tar.gz temperature-mod-dkms-$TEMP_KM_VER

# Copy to artifacts
mv -f temperature-mod-dkms-${TEMP_KM_PPA_VER}.tar.gz $OUTPUT_DIR

# Build kernel modules ipk
cd $TEMP_ROOT_DIR
sudo rm -rf $DSPTOP_ROOT_DIR/install_temperature
sudo mkdir $DSPTOP_ROOT_DIR/install_temperature
sudo chmod 777 $DSPTOP_ROOT_DIR/install_temperature
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=$LINUX_KERNEL_DIR KVERSION=$KERNEL_VER DESTDIR=$DSPTOP_ROOT_DIR/install_temperature make install_ipk

# Create the ipk and move to artifacts directory.
sed -i $MCSDK_HPC_DIR/mkrel/ipk/debugss-mod/control -e 's|Version:.*$|Version: '$TEMP_KM_VER'|g'
sudo $MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/temperature-mod $DSPTOP_ROOT_DIR/install_temperature
sudo mv -v *.ipk $OUTPUT_DIR

# Clean up
sudo rm -rf $DKMS_TEMP_ROOT_DIR
sudo rm -rf $DSPTOP_ROOT_DIR/install_temperature

#----------------------------------------------------------------------
# Begin packaging for ulm PPA.
#----------------------------------------------------------------------
DSPTOP_ROOT_DIR=$DEBUG_COMPONENTS_DIR/dsptop
cd $DSPTOP_ROOT_DIR/dsptop/ulm

ULM_VER=`grep "MAJOR_VERSION" version.h | sed -e 's|[^0-9]||g'`
ULM_VER=${ULM_VER}.`grep "MINOR_VERSION" version.h | sed -e 's|[^0-9]||g'`
ULM_VER=${ULM_VER}.`grep "PATCH_VERSION" version.h | sed -e 's|[^0-9]||g'`
echo ULM_VER=$ULM_VER
cd $DEBUG_COMPONENTS_DIR/dsptop/
sudo rm -rf ti-ulm_$ULM_VER
sudo rm -rf build_ti-ulm_$ULM_VER
mkdir -pv ti-ulm_$ULM_VER
mkdir -pv build_ti-ulm_$ULM_VER
cp -rf $DEBUG_COMPONENTS_DIR/dsptop/dsptop/* ti-ulm_$ULM_VER
cp -rf $DEBUG_COMPONENTS_DIR/dsptop/dsptop/* build_ti-ulm_$ULM_VER

# Build and install ipk for ulm.
cd $DEBUG_COMPONENTS_DIR/dsptop/build_ti-ulm_$ULM_VER/ulm
rm -rf ./install_ulm
mkdir -pv ./install_ulm
make arm release install DEVICE=C66AK2Hxx CROSS_COMPILE=arm-linux-gnueabihf- DESTDIR=./install_ulm XPORT_ONLY
echo $PATH
C6X_C_DIR=$TI_OCL_CGT_INSTALL/include make dsp release install DEVICE=C66AK2Hxx DESTDIR=./install_ulm

# Create the ipk and move to artifacts directory.
cd $DEBUG_COMPONENTS_DIR/dsptop/build_ti-ulm_$ULM_VER/ulm
sed -i $MCSDK_HPC_DIR/mkrel/ipk/ti-ulm/control -e 's|Version:.*$|Version: '$ULM_VER'|g'
sudo $MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/ti-ulm ./install_ulm
sudo mv -v ti-ulm*.ipk $OUTPUT_DIR

# Install ipk into devkit for EVM.
dpkg -x $OUTPUT_DIR/ti-ulm*.ipk $LINUX_DEVKIT_ROOT

# Clean up
sudo rm -rf $DEBUG_COMPONENTS_DIR/dsptop/build_ti-ulm_$ULM_VER
sudo rm -rf $DEBUG_COMPONENTS_DIR/dsptop/ti-ulm_$ULM_VER

#----------------------------------------------------------------------
# Begin build and package for gdbc6x.
#----------------------------------------------------------------------
GDB_DIR=$DEBUG_COMPONENTS_DIR/gdb
C6X_GDB_DIR=$DEBUG_COMPONENTS_DIR/gdb/gdbc6x
cd $C6X_GDB_DIR

# Obtain gdbc6x version from verion file.
GDBC6X_VER=`grep "TI_GDB_MAJOR_VER" version | sed -e 's|[^0-9]||g'`
GDBC6X_VER=${GDBC6X_VER}.`grep "TI_GDB_MINOR_VER" version | sed -e 's|[^0-9]||g'`
GDBC6X_VER=${GDBC6X_VER}.`grep "TI_GDB_PATCH_VER" version | sed -e 's|[^0-9]||g'`
echo GDBC6X_VER=$GDBC6X_VER

# Clean gdb directory, package, and move to artifacts directory.
rm -rf $GDB_DIR/install_gdb
rm -rf $GDB_DIR/gdbc6x_$GDBC6X_VER

# Tar up source and send to artifacts.
cd $GDB_DIR
mkdir -pv gdbc6x_$GDBC6X_VER
cp -rf $C6X_GDB_DIR/* gdbc6x_$GDBC6X_VER
tar -zcvf gdbc6x_$GDBC6X_VER.tar.gz gdbc6x_$GDBC6X_VER 
echo mv gdbc6x_$GDBC6X_VER.tar.gz $OUTPUT_DIR
mv gdbc6x_$GDBC6X_VER.tar.gz $OUTPUT_DIR

# Build c6x_gdb for ipk packaging.
# Build in directory that was used to create upstream.
cd $GDB_DIR/gdbc6x_$GDBC6X_VER
echo "configure gdb"
./configure --host=arm-linux-gnueabihf --target=tic6x-elf-tirtos --prefix=$GDB_DIR/install_gdb --program-suffix=c6x CC=arm-linux-gnueabihf-gcc CFLAGS="-g -O2 -D _LITTLE_ENDIAN" LDFLAGS="-L${LINUX_DEVKIT_ROOT}/usr/lib"
make

# Install dsptop to a temp folder for ipk packaging.
cd $GDB_DIR/gdbc6x_$GDBC6X_VER
echo "install gdbc6x"
make install

# Skipping usr/share folder as this prevents installing on EVM
# due to the already existing gdb installation.
rm -rf $GDB_DIR/install_gdb/usr/share

# Create the ipk and move to artifacts directory.
sed -i $MCSDK_HPC_DIR/mkrel/ipk/gdbc6x/control -e 's|Version:.*$|Version: '$GDBC6X_VER'|g'
$MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/gdbc6x $GDB_DIR/install_gdb
mv -v *.ipk $OUTPUT_DIR

# Clean up
sudo rm -rf $GDB_DIR/gdbc6x_$GDBC6X_VER
sudo rm -rf $GDB_DIR/install_gdb

#----------------------------------------------------------------------
# Begin build and package for gdbproxy kernel module.
#----------------------------------------------------------------------
cd $DEBUG_COMPONENTS_DIR/gdb/kernel_module/

# Obtain gdbproxy kernel module version.
cd $DEBUG_COMPONENTS_DIR/gdb/kernel_module/gdbproxy-mod
GDB_KM_VER=`grep "GDB_KM_MAJOR_VERSION" gdbserverproxy.c | sed -e 's|[^0-9]||g'`
GDB_KM_VER=${GDB_KM_VER}.`grep "GDB_KM_MINOR_VERSION" gdbserverproxy.c | sed -e 's|[^0-9]||g'`
GDB_KM_VER=${GDB_KM_VER}.`grep "GDB_KM_PATCH_VERSION" gdbserverproxy.c | sed -e 's|[^0-9]||g'`
echo GDB_KM_VER=$GDB_KM_VER

# Create a directory in parent directory to build DKMS module
cd $DEBUG_COMPONENTS_DIR/gdb
DKMS_GDB_ROOT_DIR=$DEBUG_COMPONENTS_DIR/gdb/gdbproxy-mod-dkms-$GDB_KM_VER
GDB_ROOT_DIR=$DEBUG_COMPONENTS_DIR/gdb/gdbproxy-mod-dkms-$GDB_KM_VER/gdbproxy-mod-$GDB_KM_VER
sudo rm -rf $DKMS_GDB_ROOT_DIR
mkdir -p $DKMS_GDB_ROOT_DIR
mkdir -p $GDB_ROOT_DIR

# Assemble the tar ball directory structure.
cd $DEBUG_COMPONENTS_DIR/gdb/kernel_module/
echo cp -rf gdbproxy-mod/* $GDB_ROOT_DIR
cp -rf gdbproxy-mod/* $GDB_ROOT_DIR
cp -rf debian $DKMS_GDB_ROOT_DIR

# Create tar ball for PPA.
cd $DKMS_GDB_ROOT_DIR
GDB_KM_PPA_VER=$(dpkg-parsechangelog | sed -n 's/^Version: //p')
cd $DEBUG_COMPONENTS_DIR/gdb
tar cvfz gdbproxy-mod-dkms-${GDB_KM_PPA_VER}.tar.gz gdbproxy-mod-dkms-$GDB_KM_VER

# Copy to artifacts
mv -f gdbproxy-mod-dkms-${GDB_KM_PPA_VER}.tar.gz $OUTPUT_DIR

# Build kernel modules ipk
cd $GDB_ROOT_DIR
sudo rm -rf $DSPTOP_ROOT_DIR/install_gdbproxy
sudo mkdir $DSPTOP_ROOT_DIR/install_gdbproxy
sudo chmod 777 $DSPTOP_ROOT_DIR/install_gdbproxy
ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KDIR=$LINUX_KERNEL_DIR KVERSION=$KERNEL_VER DESTDIR=$DEBUG_COMPONENTS_DIR/gdb/install_gdbproxy make install_ipk

# Create the ipk and move to artifacts directory.
sed -i $MCSDK_HPC_DIR/mkrel/ipk/gdbproxy-mod/control -e 's|Version:.*$|Version: '$GDB_KM_VER'|g'
sudo $MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/gdbproxy-mod $DEBUG_COMPONENTS_DIR/gdb/install_gdbproxy
sudo mv -v *.ipk $OUTPUT_DIR

# Clean up
cd $DEBUG_COMPONENTS_DIR/gdb/
sudo rm -rf $GDB_ROOT_DIR
sudo rm -rf $DKMS_GDB_ROOT_DIR
sudo rm -rf $DEBUG_COMPONENTS_DIR/gdb/install_gdbproxy

#----------------------------------------------------------------------
# Begin package only for GDB server (DSP only package).
#----------------------------------------------------------------------
cd $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x/

# Obtain gdb_server version.
cd $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x/include/
GDB_SERV_VER=`grep "GDB_SERVER_MAJOR_VER" GDB_server.h | sed -e 's|[^0-9]||g'`
GDB_SERV_VER=${GDB_SERV_VER}.`grep "GDB_SERVER_MINOR_VER" GDB_server.h | sed -e 's|[^0-9]||g'`
GDB_SERV_VER=${GDB_SERV_VER}.`grep "GDB_SERVER_PATCH_VER" GDB_server.h | sed -e 's|[^0-9]||g'`
echo GDB_SERV_VER=$GDB_SERV_VER

# Build gdb server
cd $DEBUG_COMPONENTS_DIR/gdb/
sudo rm -rf $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_$GDB_SERV_VER
mkdir $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_$GDB_SERV_VER
cp -rf $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x/* $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_$GDB_SERV_VER

cd $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_$GDB_SERV_VER/src
TI_CGT_INSTALL=$TI_OCL_CGT_INSTALL make

# Build ipk
cd $DEBUG_COMPONENTS_DIR/gdb/
rm -rf $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_install
mkdir $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_install
cd $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_$GDB_SERV_VER
make install DESTDIR=$DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_install

# Create the ipk and move to artifacts directory.
sed -i $MCSDK_HPC_DIR/mkrel/ipk/gdbserver-c6x/control -e 's|Version:.*$|Version: '$GDB_SERV_VER'|g'
sudo $MCSDK_HPC_DIR/mkrel/ipk/create_ipk.sh $MCSDK_HPC_DIR/mkrel/ipk/gdbserver-c6x $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_install
sudo mv -v *.ipk $OUTPUT_DIR

# Install ipk into devkit for EVM.
dpkg -x $OUTPUT_DIR/gdbserver-c6x*.ipk $LINUX_DEVKIT_ROOT

# Zip and send to artifacts directory.
cd $DEBUG_COMPONENTS_DIR/gdb/
tar -zcvf gdbserver-c6x_$GDB_SERV_VER.tar.gz --exclude='.gitignore' --exclude='src' --exclude='.gdbserver-c6x*' gdbserver-c6x_$GDB_SERV_VER
echo mv gdbserver-c6x_$GDB_SERV_VER.tar.gz $OUTPUT_DIR
mv gdbserver-c6x_$GDB_SERV_VER.tar.gz $OUTPUT_DIR

# Clean up
sudo rm -rf $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_install
sudo rm -rf $DEBUG_COMPONENTS_DIR/gdb/gdbserver-c6x_$GDB_SERV_VER
