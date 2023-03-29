#!/bin/bash -e

create_dpkg_restore_point()
{
    dpkg --get-selections | awk '{print $1}' > /tmp/dpkg-list-before.list
}

dpkg_restore_point()
{
    dpkg --get-selections | awk '{print $1}' > /tmp/dpkg-list-after.list  
    comm -13 --nocheck-order /tmp/dpkg-list-before.list /tmp/dpkg-list-after.list | sudo xargs apt remove --yes --purge
}
