# PHASER Deployment Scripts

The deployment scripts are used to provide a easy to use `docker` script for building PHASER.

## Getting Started

First, install `docker` using 
```
sudo apt install docker.io docker
```
then run the script
```
./phaser_deploy/build_docker.sh
```
__Note__: Depending on your docker configuration you might need to `sudo` the above. 
The script automatically creates a new `ubuntu:18.04` container, installs all the dependencies, and builds PHASER. 

## Running the Container

The PHASER container can then be run using

```
docker run -it phaser.core (bash/rosrun/...) 
```
When running the container, the workspace is automatically sourced and a new roscore within the container is also launched. 
