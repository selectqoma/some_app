#!/bin/bash

if [ -z $1 -o -z $2 ]
then
  echo
  echo "Syntax: $0 <source directory> <max size>"
  echo
  exit 1
fi

DELETEDIR=$1
MAXSIZE=$2


# archive unopened dirs and delete originals except the latest one
for directory in $(find $DELETEDIR -mindepth 1 -type d | tail -n +2 | sort)
do
  basename=$(echo ${directory} | sed -e "s/.*\///")
  if lsof | grep -q ${basename}
  then
    echo "Directory ${directory} seems to be in use. Skipping..."
  else
    echo "Archiving ${directory} to ${basename}.tar"
    if [ -f $1/${basename}.tar ]
    then
      echo "Warning: $1/${basename}.tar exists, but ${directory} is not deleted, archiving again."
      rm $1/${basename}.tar
    fi
    tar -C $1 -cf $1/${basename}.tar ${basename}
    mv ${directory} $1/delete-${basename}
    rm -rf $1/delete-${basename}
  fi
done

# delete archived dirs if maxsize is exceeded (latest dir is kept)
# script taken from https://stackoverflow.com/questions/11618144/bash-script-to-limit-a-directory-size-by-deleting-files-accessed-last
if [[ -z "$DELETEDIR" || -z "$MAXSIZE" || "$MAXSIZE" -lt 1 ]]; then
    echo "usage: $0 [directory] [maxsize in megabytes]" >&2
    exit 1
fi
find "$DELETEDIR" -mindepth 1 -type f -printf "%T@::%p::%s\n | tail -n +2" \
| sort -rn \
| awk -v maxbytes="$((1024 * 1024 * $MAXSIZE))" -F "::" '
  BEGIN { curSize=0; }
  { 
  curSize += $3;
  if (curSize > maxbytes) { print $2; }
  }
  ' \
  | tac | awk '{printf "%s\0",$0}' | xargs -0 -r rm
# delete empty directories
find "$DELETEDIR" -mindepth 1 -depth -type d -empty -exec rmdir "{}" \;

