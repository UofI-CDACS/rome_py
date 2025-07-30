#!/bin/bash
set -euo pipefail

check_dir() {
  local dir=$1
  if [[ ! -d $dir ]]; then
    echo "Directory not found: $dir"
    exit 1
  fi
}

check_file() {
  local file=$1
  if [[ ! -f $file ]]; then
    echo "File not found: $file"
    exit 1
  fi
}
