Kubernetes nodes have to installed with graphical user interface and we are following these [instructions](https://wiki.ros.org/docker/Tutorials/GUI)

```
cd SMILE
docker login -u "vaskostara" -p "REDACTED" docker.io

docker build -t factory-simulation:latest --build-arg name=value -f ./.devcontainer/Dockerfile ./
docker tag factory-simulation:latest vaskostara/factory-simulation:latest
docker push vaskostara/factory-simulation
k delete deployment factory-simulation
k delete deployment camera-processor
k delete service  sim-service
sleep 15
k apply -f ./k8s_deployment/simulation.yaml
k apply -f ./k8s_deployment/camera.yaml
k apply -f ./k8s_deployment/net.yaml
sleep 5
k get pods
k get services

k logs PODNAME
k  exec -ti   PODNAME  -- bash
```

For running the docker image locally (outside k8s)

```
# -v /tmp:/tmp -u ros -w /var/lib/jenkins/workspace/INFOBOT -v .:/var/lib/jenkins/workspace/INFOBOT:rw,z  -v .:/home/ros/src/:rw,z

docker run -t -d factory-simulation /bin/bash
docker exec -it `docker ps -q --filter ancestor=factory-simulation` bash
```
