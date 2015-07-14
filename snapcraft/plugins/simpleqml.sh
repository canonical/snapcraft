#!/bin/bash
set -e
set -u

# D=/home/ubuntu/thing
D=/apps/qmlapp.sideload/0/usr/
A=x86_64-linux-gnu
export XDG_CONFIG_DIRS=$D/xdg:/etc/xdg
export QTCHOOSER_NO_GLOBAL_DIR=1
export QT_QPA_PLATFORM=ubuntumirclient
export QT_PLUGIN_PATH=$D/lib/$A/qt5/plugins
export QML2_IMPORT_PATH=$$D/lib/$A/qt5/qml

export MIR_SOCKET=/run/mir_socket
export MIR_CLIENT_PLATFORM_PATH=$D/lib/$A/mir/client-platform

export LD_LIBRARY_PATH=$D/lib/$A/mesa-egl:$D/lib/$A/mesa:$D/lib/$A

# $D/bin/$A/qmlscene

./strace -o /tmp/trace -f -s 120 $D/lib/$A/qt5/bin/qmlscene

