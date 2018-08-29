#!/bin/bash

# ==================================
# define some colors for output
# ==================================
COLOR_RED="\033[1;31m"
COLOR_GREEN="\033[1;32m"
COLOR_YELLOW="\033[1;33m"
COLOR_BLUE="\033[1;34m"
COLOR_CYAN="\033[1;36m"
COLOR_WHITE="\033[1;37m"
COLOR_RESET="\033[0m"

# ==================================
# initialize variables
# ==================================
VERBOSE_LEVEL=0
SEARCH_TEXT=""
REPLACE_TEXT=""
SEARCH_DIR="./"

# ==================================
# Parse the command line arguments
# ==================================
argv=($@)
argc=${#argv[@]}
for (( i=0; i<${argc}; i++ )); do

  # documentation
  if [ "${argv[$i]}" == "--help" ] ; then
    echo -e $COLOR_GREEN"Brief explanation:"$COLOR_RESET
    echo "Use to search for a specifix string or pattern and replace it with another one in all files in folders and subfoldes in a specific path." 
    echo -e $COLOR_GREEN"Example usage:"$COLOR_RESET
    echo -e $COLOR_CYAN"$ ./replace_text.sh --search apple --replace orange --path ./fruits"$COLOR_RESET
    echo "The above command wil replace all occureneces of \"apple\" with \"orange\" in all files in folders and subfolders of directory \"./fruits\""
    echo -e $COLOR_GREEN"Execution options/arguments:"$COLOR_RESET
    echo -e $COLOR_YELLOW"--search"$COLOR_RESET"   :" "Use this followed by the string or pattern you want to search and replace. (default = \"\")"
    echo -e $COLOR_YELLOW"--replace"$COLOR_RESET"  :" "Use this followed by the string or pattern which will replace the searched one. (default = \"\")"
    echo -e $COLOR_YELLOW"--path"$COLOR_RESET"     :" "Specify the path where you want to search. (default = current directory)"
    echo -e $COLOR_YELLOW"-v"$COLOR_RESET"         :" "Verbose output: prints also the number of files containing a match and the total number of matches."
    echo -e $COLOR_YELLOW"-vv"$COLOR_RESET"        :" "Very verbose output: same as \"-v\" and additionally prints the files numbers and the text of each line with the matching pattern."
    echo -e $COLOR_GREEN"NOTE:"$COLOR_RESET"The search string can be any regex type expression. So if you want for example to search a phrase like \"red apple\" you re gonna have to pass \"red\sapple\" as the search pattern, where \"\s\" stands for white space."
    exit 0
  fi

  if [ "${argv[$i]}" == "--path" ] ; then
    i=(${i}+1)
    SEARCH_DIR=${argv[$i]}
  elif [ "${argv[$i]}" == "--search" ] ; then
    i=(${i}+1)
    SEARCH_TEXT=${argv[$i]}
  elif [ "${argv[$i]}" == "--replace" ] ; then
    i=(${i}+1)
    REPLACE_TEXT=${argv[$i]}
  elif [ "${argv[$i]}" == "-vv" ] ; then
    VERBOSE_LEVEL=2
  elif [ "${argv[$i]}" == "-v" ] ; then
    VERBOSE_LEVEL=1
  fi

done


# ==================================
# Print results (verbose level = 1)
# ==================================
if [ ${VERBOSE_LEVEL} -ge 1 ]; then

  targets=($(grep -rl ${SEARCH_DIR} -e "${SEARCH_TEXT}"))
  length=${#targets[@]}
  n_matches=$(grep -rl ${SEARCH_DIR} -e "${SEARCH_TEXT}" | xargs grep -o "${SEARCH_TEXT}" | wc -l)

  echo #newline
  echo "========================================"
  echo -e "==> Searched text:   " $COLOR_RED ${SEARCH_TEXT} $COLOR_RESET
  echo -e "==> Replace text:    " $COLOR_GREEN ${REPLACE_TEXT} $COLOR_RESET
  echo -e "==> Search directory:" $COLOR_CYAN ${SEARCH_DIR} $COLOR_RESET
  echo "========================================"
  echo #newline
  echo "========================================"
  echo -e "Pattern \""$COLOR_RED${SEARCH_TEXT}$COLOR_RESET"\" was "$COLOR_GREEN"found in" $length "file(s)."$COLOR_RESET
  echo -e $COLOR_GREEN"Total occurences of"$COLOR_RESET" pattern \""$COLOR_RED${SEARCH_TEXT}$COLOR_RESET"\" found:" $COLOR_GREEN$n_matches$COLOR_RESET
  echo "========================================"
fi

# ==================================
# Print results (verbose level = 2)
# ==================================
if [ ${VERBOSE_LEVEL} -ge 2 ]; then
  echo #newline
  echo "========================================"
  echo "The exact occurrences of pattern \""${SEARCH_TEXT}"\":"
  grep -rl ${SEARCH_DIR} -e "${SEARCH_TEXT}" | xargs grep -n --color=auto -e "${SEARCH_TEXT}"
  echo "========================================"
fi

grep -rl "${SEARCH_TEXT}" ${SEARCH_DIR} | xargs sed -i "s/${SEARCH_TEXT}/${REPLACE_TEXT}/g"

