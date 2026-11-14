#!/bin/bash
# 6. Destroy the server: the ONLY thing that stops the bill (a powered-off server still bills).
#    Refuses unless the last pull is complete (1550 score files), or you pass --force. Asks first.
set -e; source "$(dirname "$0")/env.sh"; cd "$REPO"
KEY=$(cat ~/.config/motobat/vultr_api_key 2>/dev/null) || { echo "No API key in ~/.config/motobat/vultr_api_key: destroy it in the Vultr panel instead." ; exit 1; }
last=$(cat ~/.config/motobat/last_pull 2>/dev/null || true)
n=$(ls $last/*/*/*/*.score.json 2>/dev/null | wc -l)
if [ "$n" -lt 1550 ] && [ "$1" != --force ]; then echo "Last pull ($last) has $n/1550 results. Pull first, or --force."; exit 1; fi
ID=$(curl -s -H "Authorization: Bearer $KEY" "https://api.vultr.com/v2/instances?per_page=500" |
     python3 -c "import json,sys; print(next(i['id'] for i in json.load(sys.stdin)['instances'] if i['main_ip']=='$HOST'))")
read -p "Destroy server $HOST (instance $ID)? Its disk is deleted. Type yes: " a
[ "$a" = yes ] || { echo "not destroyed"; exit 1; }
code=$(curl -s -o /dev/null -w '%{http_code}' -X DELETE -H "Authorization: Bearer $KEY" "https://api.vultr.com/v2/instances/$ID")
[ "$code" = 204 ] && { echo "destroyed; billing stops. Check the Vultr panel shows no instances."; rm -f ~/.config/motobat/cloud_host; } || echo "API said $code: destroy it in the panel NOW"
