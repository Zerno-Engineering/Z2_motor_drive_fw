#!/bin/bash

set -e

PROJECT_PATH=$(pwd)
BASE_PATH=$PROJECT_PATH

APP_VERSION=""
BOARD_NAME="zerno_drive"
HEADER_FILE="./hwconf/Zerno/hw_zerno_drive_core.h"

function get_app_version {
    GIT_HASH=`git rev-parse --short=8 HEAD`
    status=$?

    APP_VERSION=$GIT_HASH

    if [ $status -eq 127 ]; then
    echo "Git not installed, and it's required"
    exit 1
    elif [ $status -ne 0 ]; then
    exit 1
    fi
}

function get_fw_version {
    VERSION_FILE=$BASE_PATH"/tools/cmake/version.cmake"
    echo "Version file: $VERSION_FILE"

    VERSION_MAJOR=$(grep -oP 'set\(VERSION_INFO_MAJOR\s+\K\d+' "$VERSION_FILE")
    VERSION_MINOR=$(grep -oP 'set\(VERSION_INFO_MINOR\s+\K\d+' "$VERSION_FILE")
    VERSION_BUILD=$(grep -oP 'set\(VERSION_INFO_BUILD\s+\K\d+' "$VERSION_FILE")

    APP_VERSION=$VERSION_MAJOR.$VERSION_MINOR.$VERSION_BUILD-h-$APP_VERSION
    echo "App version: $APP_VERSION"
}

function get_project_name {
    PROJECT_NAME=$(basename "$PROJECT_PATH")

    GREEN="\033[0;32m"
    RESET="\033[0m"
    echo -e "${GREEN}Project Name is: $PROJECT_NAME${RESET}"
}

function get_board_name {

    GREEN="\033[0;32m"
    RESET="\033[0m"
    echo -e "${GREEN}Board Name is: $BOARD_NAME${RESET}"
}

function set_shunt_value {

    local value=$1
    echo "Setting Shunt Resistor to: $value"
    sed -i "s/\(#define CURRENT_SHUNT_RES\s\+\).*/\1$value/" "$HEADER_FILE"

}

function compile_fw {

    make -j8 fw_zerno_drive
}

function is_tag_build {
    CURRENT_TAG=$(git tag --points-at HEAD 2>/dev/null | head -n 1) || true
    [ -n "$CURRENT_TAG" ]
}


get_app_version
#get_fw_version
get_project_name
get_board_name

COMPILATION_TIME=$(date +"%Y-%m-%d-%H-%M-UTC%z")
RELEASE_NAME=$PROJECT_NAME-$BOARD_NAME-release-v-$APP_VERSION-$COMPILATION_TIME

for SHUNT in "0.003" "0.005"
do
    BLUE="\033[0;34m"
    RESET="\033[0m"
    echo -e "${BLUE}Building variant: $SHUNT${RESET}"

    if [ "$SHUNT" == "0.003" ]; then
        REV="Rev1"
    else
        REV="Rev2"
    fi

    set_shunt_value "$SHUNT"
    compile_fw

    CLEAN_VAL=$(echo $SHUNT | sed 's/\.//g')
    DEST_FW_PATH="$BASE_PATH/$RELEASE_NAME"

    mkdir -p "$DEST_FW_PATH"

    SRC_FW_PATH="$PROJECT_PATH/build"

    cp "$SRC_FW_PATH"/zerno_drive/*.bin "$DEST_FW_PATH/${PROJECT_NAME}_${REV}.bin"
done

zip -r "./$RELEASE_NAME.zip" "./$RELEASE_NAME/"
