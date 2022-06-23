# PHASER Deployment Scripts

Install docker using 
```
sudo apt install docker.io docker
```
then run the script
```
./phaser_deploy/build_docker.sh
```
This creates a new ubuntu:18.04 container, installs all the dependencies, and builds PHASER. 

PHASER can then be run using

```
docker run -it phaser.core (bash/rosrun/...) 
```