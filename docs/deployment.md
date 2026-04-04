### How to deploy (install) mayara

The mayara server needs to run on a computer with at least 32 MB free disk and 32 free RAM, that has a wired connection to the radar. Clients can connect to the mayara server over WiFi.

There are pre-built binary images for the following platforms:

- Microsoft Windows on x86_64
- Apple MacOS on Apple Silicon
- Linux on x86_64
- Linux on aarch64

The latter supports the smallest computers, like a Raspberry 3B or better, and various internet routers such as the GL-iNet B1300.

## Docker/Podman

Pre-built images for `amd64` and `arm64` are available on GitHub Container Registry:

    docker pull ghcr.io/marineyachtradar/mayara-server:latest

### Quick start (emulator)

    docker run -p 6502:6502 ghcr.io/marineyachtradar/mayara-server:latest mayara-server --emulator

### Real radar

Radar discovery uses multicast/broadcast, so the container needs direct network access:

    docker run --net=host \
        --read-only --security-opt no-new-privileges:true --cap-drop ALL \
        --tmpfs /home/mayara/.config/mayara \
        --tmpfs /home/mayara/.local/share/mayara \
        --tmpfs /tmp \
        ghcr.io/marineyachtradar/mayara-server:latest \
        mayara-server --brand navico --interface eth0

### Persistent data

To keep configuration and recordings across restarts, mount host directories owned by UID/GID `1000`:

    sudo mkdir -p /srv/mayaraserver-data/{config,recordings}
    sudo chown 1000:1000 /srv/mayaraserver-data/config /srv/mayaraserver-data/recordings

Then add volume mounts:

    -v /srv/mayaraserver-data/config:/home/mayara/.config/mayara:rw
    -v /srv/mayaraserver-data/recordings:/home/mayara/.local/share/mayara/recordings:rw

### Docker Compose

See `docker/docker-compose.yml` for ready-made examples including emulator, real radar, TLS, and shore-based setups.