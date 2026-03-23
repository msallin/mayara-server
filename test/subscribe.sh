#!/bin/bash

set -euo pipefail
set -x

HOST=${1:-10.56.0.1}

BASE_URL="http://${HOST}:6502"
V2="${BASE_URL}/signalk/v2/api/vessels/self/radars"

curl -s "${V2}" | jq 
radars=($(curl -s "${V2}" | jq -r '.radars | keys[]'))
echo "Radars: ${radars}"

for radar in ${radars}
do
  streamUrl=$(curl -s "${V2}" | jq -r ".radars.${radar}.streamUrl")"?subscribe=all"
  echo "Connecting to $streamUrl"
  websocat "${streamUrl}"
done



