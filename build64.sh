#!/bin/bash

CWD="$(cd "$(dirname "$0")"; pwd)"

if [ "$CWD" = "$PWD" ]
then
	exit 1
fi

#ENV_FILE="env"
#if ! [ -f "$CWD/$ENV_FILE" ]
#then
#	echo "$CWD/$ENV_FILE not found"
#	exit 1
#fi
#. "$CWD/$ENV_FILE"

find "$CWD" -maxdepth 1 -type f -exec sh -c 'file="{}"; if [ "${file##*.}" = "sh" ]; then chmod a+x "$file"; else chmod a-x "$file"; fi' \;
find "$PWD"             -type f -exec sh -c 'file="{}"; if [ "${file##*.}" = "sh" ]; then chmod a+x "$file"; else chmod a-x "$file"; fi' \;

$CWD/kernel/arm64-make.sh SUBDIRS=$PWD $@
