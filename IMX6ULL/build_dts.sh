#!/bin/sh
echo "copy dts to source directory..."
cp /home/jack/linux/IMX6ULL/imx6ull-alientek-emmc.dts /home/jack/linux/source/l\
inux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/arm/boot/dts/
echo "enter source directory..."
cd /home/jack/linux/source/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek
echo "build device tree..."
make dtbs
echo "copy dtb file to tftp directory..."
cp /home/jack/linux/source/linux-imx-rel_imx_4.1.15_2.1.0_ga_alientek/arch/arm/\
boot/dts/imx6ull-alientek-emmc.dtb /home/jack/linux/tftp/