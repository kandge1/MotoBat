# Sourced by the cloud scripts. The server's address and the Vultr API key live outside
# the repo, in ~/.config/motobat/ (cloud_host: the IP; vultr_api_key: chmod 600).
CFG=~/.config/motobat
HOST=$(cat $CFG/cloud_host 2>/dev/null) || { echo "No server: put its IP in $CFG/cloud_host" >&2; exit 1; }
SSH="ssh -o StrictHostKeyChecking=accept-new -o ServerAliveInterval=30 root@$HOST"
REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
REMOTE=projects/MotoBat            # under /root, so ~/projects/MotoBat paths in data/*.sh hold there too
