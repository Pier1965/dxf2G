#!/bin/bash
echo "Avvio autoscan"
autoscan
echo "Modifico conficure.scan"
sed -e 's/AC_INIT(\[FULL-PACKAGE-NAME\], \[VERSION\], \[BUG-REPORT-ADDRESS\])/AC_INIT(\[dxf2G\], \[1.0.6\], \[pier@unirc.eu\])\nAM_INIT_AUTOMAKE([-Wall -Werror foreign subdir-objects])/g' < configure.scan > configure.ac
rm configure.scan
echo "Avvio autoheader"
autoheader
echo "Avvio aclocal"
aclocal
echo "Avvio automake"
automake -ac
echo "Avvio autoconf"
autoconf
