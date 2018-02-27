#!/bin/bash

CWD="$(cd "$(dirname "$0")"; pwd)"

ENV_FILE="env"
if ! [ -f "$CWD/$ENV_FILE" ]
then
	echo "$CWD/$ENV_FILE not found"
	exit 1
fi
source "$CWD/$ENV_FILE"

CFG_FILE="release.cfg"
if ! [ -f "$PWD/$CFG_FILE" ]
then
	echo "$PWD/$CFG_FILE not found"
	exit 1
fi
source "$PWD/$CFG_FILE"

RELEASE_NAME="$DRVNAME-linux-$KERNEL_VERSION"
RELEASE_DIR="$PWD/release/$RELEASE_NAME"
echo "release into \"release/$RELEASE_NAME\""
rm -rf "$RELEASE_DIR/"
mkdir -p "$RELEASE_DIR"

if ! [ -z "$FILES" ]
then
	for src in ${FILES//\"}
	do
		echo "copy \"$src\""
		cp "$PWD/$src" "$RELEASE_DIR/"
	done
fi

while IFS='=' read -r name value
do
	if [[ $name == *'_FILES' ]]
	then
		prefix=${name%%_*}
		mkdir -p "$RELEASE_DIR/$prefix"
		for src in ${value//\"}
		do
			echo "copy \"$prefix/$src\""
			cp "$PWD/$src" "$RELEASE_DIR/$prefix/"
		done
	fi
done < "$PWD/$CFG_FILE"

echo "create \"$RELEASE_NAME.tar.gz\""
rm -f "$RELEASE_NAME.tar.gz"
tar -C "$PWD/release" -czf "$RELEASE_NAME.tar.gz" "$RELEASE_NAME/"
