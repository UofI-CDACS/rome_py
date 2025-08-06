#!/bin/bash
set -euo pipefail

usage() {
  echo "Usage: $0 <workspace_dir> [-u|--unicast]"
  exit 1
}

check_args() {
  if [ $# -lt 1 ]; then usage; fi
}

parse_args() {
  ws_dir=""
  use_unicast=false

  while (($#)); do
    case $1 in
    -u | --unicast)
      use_unicast=true
      shift
      ;;
    -*)
      echo "Unknown option: $1"
      usage
      ;;
    *)
      if [[ -z $ws_dir ]]; then
        ws_dir=$1
        shift
      else
        echo "Unexpected arg: $1"
        usage
      fi
      ;;
    esac
  done
  echo "$ws_dir" "$use_unicast"
}
