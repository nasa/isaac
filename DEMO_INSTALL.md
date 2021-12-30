Demo docker install
=====

Instructions for the full demo install

Check out
---------

Run:

    mkdir ~/ws
    cd ~/ws
    git clone ssh://git@babelfish.arc.nasa.gov:7999/isaac/isaac.git --branch develop
    ./isaac/scripts/checkout.sh
    # if you prefer to use [vcstool](https://github.com/dirk-thomas/vcstool), you can run this checkout script instead:
    ./isaac/scripts/checkout-vcstool.sh

(You can also modify the checkout location `~/ws` if you want.)

Install dependencies
---------

Install docker tools:

    sudo apt-get install docker.io docker-compose

Install nvidia-docker by following (the directions here)[https://github.com/NVIDIA/nvidia-docker].

Build
---------

Run `scripts/docker/build.sh` to build the docker images for the demo.

Install
---------

Run `scripts/docker/run.sh` to run the demo. Open `http://127.0.0.1:8080` in a web browser to see what is happening. Use
`docker ps` to see the docker containers and use `docker exec -it container_name /bin/bash` to get a shell in one.

Cancel with Ctrl+c and then run `scripts/docker/shutdown.sh` to stop the demo.
