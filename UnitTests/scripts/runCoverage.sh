#!/bin/sh

# This script uses gcov/lcov to generate coverage report for unit tests.
# You need to add these compiler flags to UnitTests.pro-file to generate the necessary files:
# QMAKE_CXXFLAGS += --coverage
# QMAKE_LFLAGS += --coverage
# And to automate this a bit, add the following to the command line arguments:
#  > output.log  && (%{sourceDir}/scripts/runCoverage.sh ./)
# (Probably also create a new kit for this to differentiate from debug/release)
# Should work on linux (tested on Linux Mint 21.3 Virginia) but I couldn't get this working in Windows (10).

# Source: https://asmaloney.com/2017/01/code/code-coverage-of-unit-tests-using-qt-5-on-macos/

# ${1} is the directory containing the .gcno files (%{buildDir} in Qt Creator)
 
LCOV=lcov
GENHTML=genhtml
 
SRC_DIR="${1}"
HTML_RESULTS="${1}/html"
 
mkdir -p ${HTML_RESULTS}
 
# generate our initial info
"${LCOV}" -d "${SRC_DIR}" -c -o "${SRC_DIR}/coverage.info"
 
# remove some paths
"${LCOV}" -r "${SRC_DIR}/coverage.info" "*Qt*.framework*" "*Xcode.app*" "*.moc" "*moc_*.cpp" "*/test/*" "*/usr/*" "*Eigen/*" -o "${SRC_DIR}/coverage-filtered.info"
 
# generate our HTML
"${GENHTML}" -o "${HTML_RESULTS}" "${SRC_DIR}/coverage-filtered.info"
 
# reset our counts
"${LCOV}" -d "${SRC_DIR}" -z
