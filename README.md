<!-- GETTING STARTED -->
## Overview
This repo contains the code and dependencies for simulation of non-prehensile planar pushing in Raisim


## Getting Started

In order to compile and execute the project, you can give the instructions shown below.


## Prerequisites
Before setting up the project, you have to install docker. If you already installed docker, go to the next section
```sh
# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

# Install the Docker packages
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Add docker user
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker
```
Then log out and log in.


## Compilation and execution

1. Clone the repo.
```sh
git clone https://github.com/prisma-lab/compliant-nonprehensile-pushing-manip.git
```

2. If you already have the Raisim license, jump to the point 4. Otherwise, proceed with point 2 and 3. 
Open a terminal in compliant-nonprehensile-pushing-manip folder. Save you machine ID for Raisim activation, obtained as output from the following command
```sh
./dependencies/raisimLib/linux_raisimCheckMyMachine
```

3. Require the academic license of Raisim compiling the form at link https://raisim.com/sections/License.html . In the form it is required also the machine ID obtained at the previous point. The license file you will receive is activation.raisim.

4. In the terminal opened in compliant-nonprehensile-pushing-manip folder, run
```sh
cp <YOUR_PATH_TO_LICENSE>/activation.raisim ./dependencies/raisimLib/.raisim/
```
to copy your license inside .raisim folder.


5. In the terminal opened in compliant-nonprehensile-pushing-manip folder, source the build script.
```sh
source docker_build.sh
```
This builds the image 'pushing_raisim'.

6. In the terminal opened in compliant-nonprehensile-pushing-manip folder, source the run script.
```sh
source docker_run.sh
```
This runs the container 'pushing_raisim_container'. The user password is 'user' if needed.

7. In the container terminal run Raisim.
```sh
~/raisimLib/raisimUnityOpengl/linux/raisimUnity.x86_64

```
If the drivers are compatible, you could run 
```sh
sudo ~/raisimLib/raisimUnity/linux/raisimUnity.x86_64

```
to get better graphics resolution.

8. On your host machine, open a new terminal, connect it to the container, navigate to raisim workspace and run the application.
```sh
docker exec -it $(docker ps -aqf "name=pushing_raisim_container") bash 
cd ~/raisimLib
~/raisimLib/build/pushing_reduced/pushing_p2p
```

9. In RaisimUnity, click connect and the simulation will start.


## Development
To develop from inside the container, it is suggested to use VsCode extension.

1. Install the extention Dev Containers

2. Once the container has been started, press Ctrl+Shift+P and run the command Dev Containers: Attach to Running Container

3. Open the workspace ~/raisimLib/pushing_reduced. Note that this folder is mounted, hence all the modifications you do from inside the container in the pushing_reduced folder are mapped also on your host machine. Also the folder rsc is mounted, so you can open it if you want to modify the urdf or others (remember to run cmake after these modification).

4. Install the extension C++ and configure the configuration files as following c_cpp_properties.json:
```sh
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**",
                "~/raisimLib/raisim/linux/include/",
                "~/raisimLib/thirdParty/Eigen3/include/eigen3/",
                "~/acados/include/",
                "~/acados/include/blasfeo/include/",
                "~/acados/include/hpipm/include/"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "cStandard": "c11",
            "cppStandard": "c++17",
            "intelliSenseMode": "linux-gcc-x64"
        }
    ],
    "version": 4
}
```
settings.json:
```sh
{
    "files.associations": {
        "*.tcc": "cpp",
        "algorithm": "cpp",
        "random": "cpp",
        "fstream": "cpp",
        "istream": "cpp",
        "ostream": "cpp",
        "utility": "cpp"
    }
}
```


   
   
   
   
   
   
   
   
   
   
   
   
   
   
   

