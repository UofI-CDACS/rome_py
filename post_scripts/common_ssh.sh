#!/bin/bash
set -euo pipefail

ssh_run() {
  local ip=$1
  local user=$2
  local script=$3

  ssh -A -o StrictHostKeyChecking=no "${user}@${ip}" bash -s <<EOF
$script
EOF
}
