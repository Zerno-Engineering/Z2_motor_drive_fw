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
    # The firmware version comes from the git tags, so tagging a commit is the
    # only step needed to set a release version:
    #
    #   FW_VERSION       1.2.1             nearest tag, leading v stripped
    #   FW_VERSION_FULL  1.2.1-3-gabc1234  same, plus the number of commits made
    #                                      since that tag and the short hash,
    #                                      when HEAD is not sitting on a tag
    FW_VERSION=$(git describe --tags --abbrev=0 2>/dev/null | sed 's/^v//')
    FW_VERSION_FULL=$(git describe --tags 2>/dev/null | sed 's/^v//')

    if [ -z "$FW_VERSION" ]; then
        echo "No git tag found, cannot determine the firmware version"
        echo "Tag the release first, e.g. git tag -a v1.2.2 -m 'Release 1.2.2'"
        exit 1
    fi

    if is_tag_build; then
        echo "Building tagged release: $CURRENT_TAG"
    else
        YELLOW="\033[0;33m"
        RESET="\033[0m"
        echo -e "${YELLOW}WARNING: HEAD is not tagged. Nearest tag is v$FW_VERSION, HEAD is $FW_VERSION_FULL${RESET}"
    fi

    # Release name carries the firmware version plus the short hash it was built
    # from, e.g. 1.2.1-194b685c
    APP_VERSION=$FW_VERSION-$APP_VERSION
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
get_fw_version
get_project_name
get_board_name

COMPILATION_TIME=$(date +"%Y-%m-%d-%H-%M-UTC%z")
RELEASE_NAME=$PROJECT_NAME-$BOARD_NAME-release-v$APP_VERSION-$COMPILATION_TIME

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

    cp "$SRC_FW_PATH"/zerno_drive/*.bin "$DEST_FW_PATH/${PROJECT_NAME}_${REV}_v_${FW_VERSION}.bin"
done

zip -r "./$RELEASE_NAME.zip" "./$RELEASE_NAME/"
