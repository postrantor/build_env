#! /bin/bash

USERNAME=$1

adduser ${USERNAME} --force-badname
usermod -aG sudo ${USERNAME}

su ${USERNAME}
