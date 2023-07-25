#!/bin/bash
PS3='Please select container to connect to: '
options=($(docker ps --filter="name=$name" -q --format '{{.Names}}'| xargs) "Quit")
select opt in "${options[@]}"
do
  if [ -z $opt ]; then
    echo "Invalid selection"
  elif [ $opt == "Quit" ]; then
    echo "Quitting"
    break
  else
    for option in ${options[@]}; do
      if [[ $opt == $option ]]; then
        echo 'Connecting to container '$opt
        docker exec -it $opt /bin/bash
        break 2  # exit from the loop
      fi
    done
  fi
done
